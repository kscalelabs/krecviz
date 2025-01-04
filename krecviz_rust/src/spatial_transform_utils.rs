// spatial_transform_utils.rs

use nalgebra as na;
use na::{Matrix3, Matrix4, Vector3, Rotation3, Translation3, Isometry3, UnitQuaternion};
// spatial_transform_utils.rs

// -----------------------------------------------------------------------------
// OLD CODE
// -----------------------------------------------------------------------------

/// Multiply two 3×3 matrices (row-major).
pub fn mat3x3_mul(a: [f32; 9], b: [f32; 9]) -> [f32; 9] {
    let mut out = [0.0; 9];
    for row in 0..3 {
        for col in 0..3 {
            out[row * 3 + col] = a[row * 3 + 0] * b[col + 0]
                + a[row * 3 + 1] * b[col + 3]
                + a[row * 3 + 2] * b[col + 6];
        }
    }
    out
}

/// Compute a 3×3 rotation matrix from intrinsic Euler angles (XYZ order).
pub fn rotation_from_euler_xyz(rx: f64, ry: f64, rz: f64) -> [f32; 9] {
    let (cx, sx) = (rx.cos() as f32, rx.sin() as f32);
    let (cy, sy) = (ry.cos() as f32, ry.sin() as f32);
    let (cz, sz) = (rz.cos() as f32, rz.sin() as f32);

    let r_x = [1.0, 0.0, 0.0, 0.0, cx, -sx, 0.0, sx, cx];

    let r_y = [cy, 0.0, sy, 0.0, 1.0, 0.0, -sy, 0.0, cy];

    let r_z = [cz, -sz, 0.0, sz, cz, 0.0, 0.0, 0.0, 1.0];

    let ryx = mat3x3_mul(r_y, r_x);
    mat3x3_mul(r_z, ryx)
}

/// Build a 4×4 row-major transform from translation (xyz) and RPY Euler angles.
pub fn build_4x4_from_xyz_rpy(xyz: [f64; 3], rpy: [f64; 3]) -> [f32; 16] {
    let rot3x3 = rotation_from_euler_xyz(rpy[0], rpy[1], rpy[2]);
    [
        rot3x3[0],
        rot3x3[1],
        rot3x3[2],
        xyz[0] as f32,
        rot3x3[3],
        rot3x3[4],
        rot3x3[5],
        xyz[1] as f32,
        rot3x3[6],
        rot3x3[7],
        rot3x3[8],
        xyz[2] as f32,
        0.0,
        0.0,
        0.0,
        1.0,
    ]
}

// -----------------------------------------------------------------------------
// Minimal 4x4 row-major transform builder that just rotates around Z
// -----------------------------------------------------------------------------
pub fn build_z_rotation_4x4(angle_rad: f64) -> [f32; 16] {
    let cz = angle_rad.cos() as f32;
    let sz = angle_rad.sin() as f32;

    [
        cz, -sz, 0.0, 0.0, sz, cz, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
    ]
}

/// Multiply two 4×4 matrices (row-major).
pub fn mat4x4_mul(a: [f32; 16], b: [f32; 16]) -> [f32; 16] {
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

/// Convert a 4×4 row-major transform into `(translation, mat3x3)`,
/// for logging as a Rerun `Transform3D`.
pub fn decompose_4x4_to_translation_and_mat3x3(tf: [f32; 16]) -> ([f32; 3], [f32; 9]) {
    let translation = [tf[3], tf[7], tf[11]];
    let mat3x3 = [
        tf[0], tf[4], tf[8],    // First column
        tf[1], tf[5], tf[9],    // Second column
        tf[2], tf[6], tf[10],   // Third column
    ];
    (translation, mat3x3)
}

/// Returns a 4x4 identity matrix as a row-major [f32; 16] array
pub fn identity_4x4() -> [f32; 16] {
    [
        1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
    ]
}

/// Build a 3×3 rotation matrix that just rotates around Z
pub fn build_z_rotation_3x3(angle_rad: f64) -> [f32; 9] {
    let cz = angle_rad.cos() as f32;
    let sz = angle_rad.sin() as f32;

    [
        cz, -sz, 0.0,  // First row
        sz,  cz, 0.0,  // Second row
        0.0, 0.0, 1.0, // Third row
    ]
}

/// Convert rotation matrix and translation into a 4x4 transform matrix
pub fn make_4x4_from_rotation_and_translation(rotation: [f32; 9], translation: [f32; 3]) -> [f32; 16] {
    [
        rotation[0], rotation[1], rotation[2], translation[0],
        rotation[3], rotation[4], rotation[5], translation[1],
        rotation[6], rotation[7], rotation[8], translation[2],
        0.0, 0.0, 0.0, 1.0,
    ]
}

// -----------------------------------------------------------------------------
// NEW CODE (NALGEBRA-BASED), ADDED INSTEAD OF REPLACING
// -----------------------------------------------------------------------------

/// Intrinsic euler angles (X->Y->Z) -> a 3x3 rotation matrix (nalgebra version).
pub fn n_rotation_from_euler_xyz(rx: f64, ry: f64, rz: f64) -> Matrix3<f32> {
    let rot_x = Rotation3::from_axis_angle(&Vector3::x_axis(), rx as f32);
    let rot_y = Rotation3::from_axis_angle(&Vector3::y_axis(), ry as f32);
    let rot_z = Rotation3::from_axis_angle(&Vector3::z_axis(), rz as f32);

    *(rot_z * rot_y * rot_x).matrix()

}

pub fn n_rotation_from_euler_xyz_new(rx: f64, ry: f64, rz: f64) -> Matrix3<f32> {
    // 1) Call the old function, which returns row-major `[f32; 9]`.
    let old_row_major = rotation_from_euler_xyz(rx, ry, rz);

    // 2) Convert row-major array => nalgebra Matrix3 (which uses column-major internally).
    //    We'll place them carefully so that `m[(r, c)]` is correct.
    Matrix3::from_column_slice(&[
        // Column 0: old_row_major[0], old_row_major[1], old_row_major[2] are row 0..2, col 0 in old code
        old_row_major[0], old_row_major[3], old_row_major[6],

        // Column 1:
        old_row_major[1], old_row_major[4], old_row_major[7],

        // Column 2:
        old_row_major[2], old_row_major[5], old_row_major[8],
    ])
}

/// Build a full 4x4 homogeneous transform from xyz + euler (intrinsic) (nalgebra).
pub fn n_build_4x4_from_xyz_rpy_old(xyz: [f64; 3], rpy: [f64; 3]) -> Matrix4<f32> {
    let rot3 = n_rotation_from_euler_xyz(rpy[0], rpy[1], rpy[2]);
    let translation = Translation3::new(xyz[0] as f32, xyz[1] as f32, xyz[2] as f32);
    let iso = Isometry3::from_parts(translation, UnitQuaternion::from_matrix(&rot3));
    iso.to_homogeneous()
}

pub fn n_build_4x4_from_xyz_rpy(xyz: [f64; 3], rpy: [f64; 3]) -> Matrix4<f32> {
    // 1) Old row-major function
    let row_maj_16 = build_4x4_from_xyz_rpy(xyz, rpy);  // returns [f32; 16]

    // 2) Convert row-major -> column-major
    let mut col_maj_16 = [0.0; 16];
    for row in 0..4 {
        for col in 0..4 {
            col_maj_16[col * 4 + row] = row_maj_16[row * 4 + col];
        }
    }

    // 3) Build a Matrix4
    Matrix4::from_column_slice(&col_maj_16)
}

/// Return a 4x4 identity matrix (nalgebra version).
pub fn n_identity_4x4() -> Matrix4<f32> {
    Matrix4::identity()
}

/// Rotate around Z by `angle_rad` (nalgebra version).
pub fn n_build_z_rotation_4x4(angle_rad: f64) -> Matrix4<f32> {
    let r = Rotation3::from_axis_angle(&Vector3::z_axis(), angle_rad as f32);
    Isometry3::from_parts(Translation3::identity(), UnitQuaternion::from_rotation_matrix(&r))
        .to_homogeneous()
}

/// Build a 3×3 rotation matrix that just rotates around Z (nalgebra).
pub fn n_build_z_rotation_3x3(angle_rad: f64) -> Matrix3<f32> {
    *Rotation3::from_axis_angle(&Vector3::z_axis(), angle_rad as f32).matrix()
}

/// Convert a Matrix4 into (translation, 3x3 rotation) for Rerun logging (nalgebra).
pub fn n_decompose_4x4_to_translation_and_mat3x3(m: &Matrix4<f32>) -> ([f32; 3], [f32; 9]) {
    // Extract translation vector
    let t = [m[(0, 3)], m[(1, 3)], m[(2, 3)]];
    
    // Extract the 3x3 rotation matrix in column-major order
    let mut rotation_3x3 = [0.0; 9];
    for col in 0..3 {
        for row in 0..3 {
            // Store in column-major order: index = col * 3 + row
            rotation_3x3[col * 3 + row] = m[(row, col)];
        }
    }
    (t, rotation_3x3)
}

/// Convert rotation matrix and translation into a 4x4 transform matrix (nalgebra).
pub fn n_make_4x4_from_rotation_and_translation(rotation: Matrix3<f32>, translation: [f32; 3]) -> Matrix4<f32> {
    let t = Translation3::new(translation[0], translation[1], translation[2]);
    let iso = Isometry3::from_parts(t, UnitQuaternion::from_matrix(&rotation));
    iso.to_homogeneous()
}

/// Multiply two 4×4 matrices (nalgebra).
pub fn n_mat4x4_mul(a: &Matrix4<f32>, b: &Matrix4<f32>) -> Matrix4<f32> {
    a * b
}

/// Multiply two 3×3 matrices (nalgebra).
pub fn n_mat3x3_mul(a: Matrix3<f32>, b: Matrix3<f32>) -> Matrix3<f32> {
    a * b
}

pub fn n_build_4x4_from_rowmajor_3x3_and_translation(
    base_rot: Matrix3<f32>,
    base_trans: [f32; 3],
) -> Matrix4<f32> {
    // Step A: interpret base_rot in row-major order:
    // If the old code had row-major [r0, r1, r2, r3, ...],
    // that means:
    //   row0: (0,0), (0,1), (0,2)
    //   row1: (1,0), (1,1), (1,2)
    //   row2: (2,0), (2,1), (2,2)
    //
    // We'll flatten out that row-major arrangement:
    let row_maj_9 = [
        base_rot[(0, 0)], base_rot[(0, 1)], base_rot[(0, 2)],
        base_rot[(1, 0)], base_rot[(1, 1)], base_rot[(1, 2)],
        base_rot[(2, 0)], base_rot[(2, 1)], base_rot[(2, 2)],
    ];

    // Step B: Bake them into a row-major 4×4 with the translation
    let row_maj_16 = [
        row_maj_9[0], row_maj_9[1], row_maj_9[2], base_trans[0],
        row_maj_9[3], row_maj_9[4], row_maj_9[5], base_trans[1],
        row_maj_9[6], row_maj_9[7], row_maj_9[8], base_trans[2],
        0.0,          0.0,          0.0,          1.0,
    ];

    // Step C: Convert row-major => column-major for nalgebra
    let mut col_maj_16 = [0.0; 16];
    for row in 0..4 {
        for col in 0..4 {
            col_maj_16[col * 4 + row] = row_maj_16[row * 4 + col];
        }
    }

    // Finally build a Matrix4<f32>
    Matrix4::from_column_slice(&col_maj_16)
}