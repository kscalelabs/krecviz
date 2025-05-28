// spatial_transform_utils_nalgebra.rs

use nalgebra::{Matrix3, Matrix4, Vector3, Rotation3, Isometry3, Translation3}; // Removed Unit for now
use std::convert::TryInto;

// Helper function to convert a nalgebra matrix to a row-major flat array.
fn matrix3_to_row_major_flat_array(matrix: &Matrix3<f32>) -> [f32; 9] {
    let mut arr = [0.0f32; 9];
    for r_idx in 0..3 {
        for c_idx in 0..3 {
            arr[r_idx * 3 + c_idx] = matrix[(r_idx, c_idx)];
        }
    }
    arr
}

// Also a helper function to convert to a row-major flat array but for 4x4.
fn matrix4_to_row_major_flat_array(matrix: &Matrix4<f32>) -> [f32; 16] {
    let mut arr = [0.0f32; 16];
    for r_idx in 0..4 {
        for c_idx in 0..4 {
            arr[r_idx * 4 + c_idx] = matrix[(r_idx, c_idx)];
        }
    }
    arr
}

/// Multiply two 3×3 matrices (input and output are row-major arrays).
pub fn mat3x3_mul(a_arr: [f32; 9], b_arr: [f32; 9]) -> [f32; 9] {
    let mat_a = Matrix3::from_row_slice(&a_arr);
    let mat_b = Matrix3::from_row_slice(&b_arr);
    let result_mat = mat_a * mat_b;
    matrix3_to_row_major_flat_array(&result_mat)
}

/// Compute a 3×3 rotation matrix from intrinsic Euler angles (XYZ order, row-major output).
pub fn rotation_from_euler_xyz(rx: f64, ry: f64, rz: f64) -> [f32; 9] {
    let rot = Rotation3::from_euler_angles(rx as f32, ry as f32, rz as f32);
    let rot_mat = rot.matrix();
    matrix3_to_row_major_flat_array(&rot_mat)
}

/// Build a 4×4 row-major transform from translation (xyz) and RPY Euler angles.
pub fn build_4x4_from_xyz_rpy(xyz_arr: [f64; 3], rpy_arr: [f64; 3]) -> [f32; 16] {
    let translation_vec = Vector3::new(xyz_arr[0] as f32, xyz_arr[1] as f32, xyz_arr[2] as f32);
    let rotation = Rotation3::from_euler_angles(rpy_arr[0] as f32, rpy_arr[1] as f32, rpy_arr[2] as f32);
    let transform = Isometry3::from_parts(Translation3::from(translation_vec), rotation.into());
    let transform_matrix4 = transform.to_homogeneous();
    matrix4_to_row_major_flat_array(&transform_matrix4)
}

/// Multiply two 4×4 matrices (input and output are row-major arrays).
pub fn mat4x4_mul(a_arr: [f32; 16], b_arr: [f32; 16]) -> [f32; 16] {
    let mat_a = Matrix4::from_row_slice(&a_arr);
    let mat_b = Matrix4::from_row_slice(&b_arr);
    let result_mat = mat_a * mat_b;
    matrix4_to_row_major_flat_array(&result_mat)
}

/// Convert a 4×4 row-major transform into `(translation_array, mat3x3_column_major_array)`,
pub fn decompose_4x4_to_translation_and_mat3x3(tf_arr: [f32; 16]) -> ([f32; 3], [f32; 9]) {
    let transform_matrix4 = Matrix4::from_row_slice(&tf_arr);
    let translation_vec: Vector3<f32> = transform_matrix4.column(3).xyz();
    let translation_out = [translation_vec.x, translation_vec.y, translation_vec.z];
    let mat3x3_part: Matrix3<f32> = transform_matrix4.fixed_view::<3, 3>(0, 0).into_owned();
    let mat3x3_col_major_array: [f32; 9] = mat3x3_part.as_slice().try_into().expect(
        "Failed to convert 3x3 matrix slice to array. This should not happen.",
    );
    (translation_out, mat3x3_col_major_array)
}

/// Returns a 4x4 identity matrix as a row-major [f32; 16] array
pub fn identity_4x4() -> [f32; 16] {
    let identity_mat = Matrix4::<f32>::identity();
    matrix4_to_row_major_flat_array(&identity_mat)
}

/// Build a 3×3 rotation matrix that just rotates around Z (row-major output).
pub fn build_z_rotation_3x3(angle_rad: f64) -> [f32; 9] {
    let rot = Rotation3::from_axis_angle(&Vector3::z_axis(), angle_rad as f32);
    let rot_mat = rot.matrix();
    matrix3_to_row_major_flat_array(&rot_mat)
}

/// Convert rotation matrix (row-major input) and translation array into a 4x4 transform matrix (row-major output).
pub fn make_4x4_from_rotation_and_translation(
    rotation_arr: [f32; 9],
    translation_arr: [f32; 3],
) -> [f32; 16] {
    let rot_mat3 = Matrix3::from_row_slice(&rotation_arr);
    let rotation = Rotation3::from_matrix_unchecked(rot_mat3);
    let translation_vec = Vector3::new(translation_arr[0], translation_arr[1], translation_arr[2]);
    let transform = Isometry3::from_parts(Translation3::from(translation_vec), rotation.into());
    let transform_matrix4 = transform.to_homogeneous();
    matrix4_to_row_major_flat_array(&transform_matrix4)
}