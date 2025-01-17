// spatial_transform_utils.rs

/// Multiply two 3×3 matrices (row-major).
pub fn mat3x3_mul(a: [f32; 9], b: [f32; 9]) -> [f32; 9] {
    let mut out = [0.0; 9];
    for row in 0..3 {
        for col in 0..3 {
            out[row * 3 + col] =
                a[row * 3] * b[col] + a[row * 3 + 1] * b[col + 3] + a[row * 3 + 2] * b[col + 6];
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
        tf[0], tf[4], tf[8], // First column
        tf[1], tf[5], tf[9], // Second column
        tf[2], tf[6], tf[10], // Third column
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
    let c = angle_rad.cos() as f32;
    let s = angle_rad.sin() as f32;
    [
        c, -s, 0.0, // row 1
        s, c, 0.0, // row 2
        0.0, 0.0, 1.0, // row 3
    ]
}

/// Convert rotation matrix and translation into a 4x4 transform matrix
pub fn make_4x4_from_rotation_and_translation(
    rotation: [f32; 9],
    translation: [f32; 3],
) -> [f32; 16] {
    [
        rotation[0],
        rotation[1],
        rotation[2],
        translation[0],
        rotation[3],
        rotation[4],
        rotation[5],
        translation[1],
        rotation[6],
        rotation[7],
        rotation[8],
        translation[2],
        0.0,
        0.0,
        0.0,
        1.0,
    ]
}
