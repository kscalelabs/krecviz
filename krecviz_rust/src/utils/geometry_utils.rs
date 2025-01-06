// geometry_utils.rs

use anyhow::Result;
use rerun::{
    archetypes::Mesh3D,
    components::{ImageBuffer, Position3D, TriangleIndices, Vector3D},
    datatypes::Blob,
};
use std::fs::OpenOptions;
use std::io::BufReader;
use std::path::Path;
use nalgebra as na;
use parry3d::shape::{Ball as ParrySphere, Cuboid as ParryCuboid, Cylinder as ParryCylinder};

// For loading image files
use image;

// For reading STL geometry
use stl_io;

use crate::utils::spatial_transform_utils::build_4x4_from_xyz_rpy;

/// Compute the cross product of two 3D vectors.
pub fn cross(a: [f32; 3], b: [f32; 3]) -> [f32; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

/// Length (magnitude) of a 3D vector.
fn length(v: [f32; 3]) -> f32 {
    (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt()
}

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

    // We'll store accumulative normals (area-weighted, by face cross).
    let mut accum = vec![[0.0_f32; 3]; n_verts];

    // Accumulate face normals
    for tri in triangles {
        let i0 = tri[0] as usize;
        let i1 = tri[1] as usize;
        let i2 = tri[2] as usize;
        if i0 >= n_verts || i1 >= n_verts || i2 >= n_verts {
            continue; // Out-of-bounds safety check
        }

        let p0 = mesh.vertex_positions[i0];
        let p1 = mesh.vertex_positions[i1];
        let p2 = mesh.vertex_positions[i2];
        let v10 = [p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2]];
        let v20 = [p2[0] - p0[0], p2[1] - p0[1], p2[2] - p0[2]];

        // Face normal (un-normalized)
        let face_n = cross(v10, v20);

        // Add to each vertex's accum
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
            final_normals.push(Vector3D::from([acc[0] / len, acc[1] / len, acc[2] / len]));
        } else {
            // fallback
            final_normals.push(Vector3D::from([0.0, 1.0, 0.0]));
        }
    }
    mesh.vertex_normals = Some(final_normals);
}

/// Apply a 4×4 transform in row-major order to every vertex & normal in the mesh.
pub fn apply_4x4_to_mesh3d(mesh: &mut Mesh3D, tf: [f32; 16]) {
    // Positions
    for v in &mut mesh.vertex_positions {
        let (x, y, z, w) = (v[0], v[1], v[2], 1.0);
        let xp = tf[0] * x + tf[1] * y + tf[2] * z + tf[3] * w;
        let yp = tf[4] * x + tf[5] * y + tf[6] * z + tf[7] * w;
        let zp = tf[8] * x + tf[9] * y + tf[10] * z + tf[11] * w;
        v[0] = xp;
        v[1] = yp;
        v[2] = zp;
    }
    // Normals
    if let Some(ref mut normals) = mesh.vertex_normals {
        for n in normals {
            let (nx, ny, nz, w) = (n[0], n[1], n[2], 0.0);
            let nxp = tf[0] * nx + tf[1] * ny + tf[2] * nz + tf[3] * w;
            let nyp = tf[4] * nx + tf[5] * ny + tf[6] * nz + tf[7] * w;
            let nzp = tf[8] * nx + tf[9] * ny + tf[10] * nz + tf[11] * w;
            n[0] = nxp;
            n[1] = nyp;
            n[2] = nzp;
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

    // Only .stl is handled
    let ext_lower = abs_path
        .extension()
        .and_then(|e| e.to_str())
        .map(|s| s.to_lowercase());
    if ext_lower.as_deref() != Some("stl") {
        return Err(anyhow::anyhow!("Currently only .stl handled"));
    }

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

            // Compute per-vertex normals
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
    let rgba = img.to_rgba8().into_raw();

    Ok(ImageBuffer(Blob::from(rgba)))
}

/// Create a box mesh from dimensions
pub fn create_box_mesh(size: [f64; 3]) -> Mesh3D {
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

/// Create a cylinder mesh from radius and length
pub fn create_cylinder_mesh(radius: f64, length: f64) -> Mesh3D {
    let half = (length as f32) / 2.0;
    let cyl = ParryCylinder::new(half, radius as f32);
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
    mesh
}

/// Create a sphere mesh from radius
pub fn create_sphere_mesh(radius: f64) -> Mesh3D {
    let ball = ParrySphere::new(radius as f32);
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
