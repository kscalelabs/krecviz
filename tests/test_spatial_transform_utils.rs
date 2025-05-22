// tests/test_utils.rs

#[cfg(test)]
mod test_spatial_transform_utils {
    use krecviz::utils::spatial_transform_utils::{
        build_4x4_from_xyz_rpy, build_z_rotation_3x3, decompose_4x4_to_translation_and_mat3x3,
        identity_4x4, make_4x4_from_rotation_and_translation, mat3x3_mul, mat4x4_mul,
        rotation_from_euler_xyz,
    };
    use std::f64::consts::{FRAC_PI_2, FRAC_PI_6}; // 90°, 30°

    const EPSILON: f32 = 1e-6;

    // Asserts that two f32 values (`a` and `b`) are approximately equal.
    fn assert_f32_eq(a: f32, b: f32, message: &str) {
        assert!((a - b).abs() < EPSILON, "{} - Expected: {}, Got: {}. Diff: {}", message, b, a, (a-b).abs());
    }

    // Asserts that two slices of f32 values (`a` and `b`) are approximately equal, element by element.
    fn assert_arr_eq(a: &[f32], b: &[f32], message: &str) {
        assert_eq!(a.len(), b.len(), "{}: Array length mismatch. Expected: {}, Got: {}", message, b.len(), a.len());
        for i in 0..a.len() {
            assert_f32_eq(a[i], b[i], &format!("{}: Index {}", message, i));
        }
    }

    /// Test multiplying a 3x3 matrix by the identity matrix.
    #[test]
    fn test_mat3x3_mul_identity() {
        let id_3x3 = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0];
        let a_3x3 = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0];

        let result = mat3x3_mul(a_3x3, id_3x3);
        assert_arr_eq(&result, &a_3x3, "mat3x3_mul with identity");
    }

    /// Test multiplying two arbitrary 3x3 matrices (hand-computed).
    #[test]
    fn test_mat3x3_mul_arbitrary() {
        // Define two 3x3 row-major matrices
        let a = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0];
        let b = [9.0, 8.0, 7.0, 6.0, 5.0, 4.0, 3.0, 2.0, 1.0];
        // Ground truth computed as follows:
        // $ python
        // Python 3.11.11 (main, Dec 11 2024, 16:28:39) [GCC 11.2.0] on linux
        // Type "help", "copyright", "credits" or "license" for more information.
        // >>> import numpy as np
        // >>> a = np.array([[1, 2, 3], [4, 5, 6], [7, 8, 9]])
        // >>> b = np.array([[9, 8, 7], [6, 5, 4], [3, 2, 1]])
        // >>> a @ b
        // array([[ 30,  24,  18],
        //        [ 84,  69,  54],
        //        [138, 114,  90]])
        let expected = [30.0, 24.0, 18.0, 84.0, 69.0, 54.0, 138.0, 114.0, 90.0];

        let result = mat3x3_mul(a, b);
        assert_arr_eq(&result, &expected, "mat3x3_mul_arbitrary");
    }

    /// Test multiplying a 4x4 matrix by the 4x4 identity matrix.
    #[test]
    fn test_mat4x4_mul_identity() {
        let id_4x4 = identity_4x4();
        let a_4x4 = [
            1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0, 16.0,
        ];

        let result = mat4x4_mul(a_4x4, id_4x4);
        assert_arr_eq(&result, &a_4x4, "mat4x4_mul with identity");

        // Also testing the identity_4x4() function itself
        let expected_id = [
            1.0, 0.0, 0.0, 0.0,
            0.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 1.0, 0.0,
            0.0, 0.0, 0.0, 1.0,
        ];
        assert_arr_eq(&id_4x4, &expected_id, "identity_4x4 function output");
    }

    /// Test multiplying two 4x4 matrices with a known result.
    #[test]
    fn test_mat4x4_mul_arbitrary() {
        let a = [
            1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0, 16.0,
        ];
        let b = [
            16.0, 15.0, 14.0, 13.0, 12.0, 11.0, 10.0, 9.0, 8.0, 7.0, 6.0, 5.0, 4.0, 3.0, 2.0, 1.0,
        ];
        // Ground truth computed as follows:
        // $ python
        // Python 3.11.11 (main, Dec 11 2024, 16:28:39) [GCC 11.2.0] on linux
        // Type "help", "copyright", "credits" or "license" for more information.
        // >>> import numpy as np
        // >>> a = np.array([[1, 2, 3, 4], [5, 6, 7, 8], [9, 10, 11, 12], [13, 14, 15, 16]])
        // >>> b = np.array([[16, 15, 14, 13], [12, 11, 10, 9], [8, 7, 6, 5], [4, 3, 2, 1]])
        // >>> a @ b
        // array([[ 80,  70,  60,  50],
        //        [240, 214, 188, 162],
        //        [400, 358, 316, 274],
        //        [560, 502, 444, 386]])
        let expected = [
            80.0, 70.0, 60.0, 50.0, 240.0, 214.0, 188.0, 162.0, 400.0, 358.0, 316.0, 274.0, 560.0,
            502.0, 444.0, 386.0,
        ];

        let result = mat4x4_mul(a, b);
        assert_arr_eq(&result, &expected, "mat4x4_mul_arbitrary");
    }

    /// Test building a rotation matrix for a 90° turn around Z only.
    #[test]
    fn test_build_z_rotation_3x3() {
        // We'll rotate by 90 degrees around the Z axis
        // => cos(90°)=0, sin(90°)=1
        let angle_rad = FRAC_PI_2; // 1.5708...
        let rot_90 = build_z_rotation_3x3(angle_rad);

        // In row-major order, a 90° rotation around Z is:
        //   [0, -1, 0,
        //    1,  0, 0,
        //    0,  0, 1]
        let expected = [0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0];
        assert_arr_eq(&rot_90, &expected, "build_z_rotation_3x3 (90deg Z)");
    }

    /// Test rotation_from_euler_xyz for a simple 90° rotation around X only.
    #[test]
    fn test_rotation_from_euler_xyz_90deg_x() {
        // Euler angles (rx=90°, ry=0°, rz=0°) => rotate around X only
        let rx = FRAC_PI_2;
        let ry = 0.0;
        let rz = 0.0;
        let rot = rotation_from_euler_xyz(rx, ry, rz);

        // A 90° rotation around X in row-major =>
        //   [1,  0,   0,
        //    0,  0,  -1,
        //    0,  1,   0]
        let expected = [1.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 1.0, 0.0];
        assert_arr_eq(&rot, &expected, "rotation_from_euler_xyz (90deg X)");
    }

    /// Test build_4x4_from_xyz_rpy for a known translation & rotation.
    #[test]
    fn test_build_4x4_from_xyz_rpy_30deg() {
        // Suppose we want to move by (1.0, 2.0, 3.0), then do a 30° turn about Z.
        let xyz_arr = [1.0, 2.0, 3.0];
        let rpy_arr = [0.0, 0.0, FRAC_PI_6]; // 30° about Z
        let tf = build_4x4_from_xyz_rpy(xyz_arr, rpy_arr);

        // cos(30°) ≈ 0.8660254, sin(30°) = 0.5
        let c = (FRAC_PI_6.cos()) as f32;
        let s = (FRAC_PI_6.sin()) as f32;

        // Expected 4x4 (row-major):
        // Rot(Z,30) = [c -s 0; s c 0; 0 0 1]
        // Translation = [1,2,3]
        // [ c  -s   0   1.0,
        //   s   c   0   2.0,
        //   0   0   1   3.0,
        //   0   0   0   1.0 ]
        let expected_tf = [
            c,   -s,  0.0, 1.0,
            s,    c,  0.0, 2.0,
            0.0,  0.0, 1.0, 3.0,
            0.0,  0.0, 0.0, 1.0,
        ];
        assert_arr_eq(&tf, &expected_tf, "build_4x4_from_xyz_rpy (30deg Z rot, trans)");
    }

    /// Test decompose_4x4_to_translation_and_mat3x3 on a known 4x4 transform.
    #[test]
    fn test_decompose_4x4_to_translation_and_mat3x3() {
        // We'll create a transform with build_4x4_from_xyz_rpy:
        let xyz = [2.0, -3.0, 5.0];
        let rpy = [FRAC_PI_2, 0.0, 0.0]; // 90° about X
        let full_tf_row_major = build_4x4_from_xyz_rpy(xyz, rpy);

        let (translation_arr, mat3x3_col_major) =
            decompose_4x4_to_translation_and_mat3x3(full_tf_row_major);

        let expected_translation = [2.0, -3.0, 5.0];
        assert_arr_eq(&translation_arr, &expected_translation, "decompose translation");

        // 90° about X (roll) => Rotation matrix (row-major):
        // [1  0   0 ]
        // [0 cos -sin]
        // [0 sin cos ]
        // For rx = pi/2 -> cx=0, sx=1:
        // [1  0   0 ]
        // [0  0  -1 ]
        // [0  1   0 ]
        // Column-major version of this:
        // Col1: [1, 0, 0]
        // Col2: [0, 0, 1]
        // Col3: [0,-1, 0]
        // Flat column-major: [1.0, 0.0, 0.0,  0.0, 0.0, 1.0,  0.0, -1.0, 0.0]
        let expected_3x3_col_major = [1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, -1.0, 0.0];
        assert_arr_eq(&mat3x3_col_major, &expected_3x3_col_major, "decompose mat3x3 (col-major)");
    }

    /// Test make_4x4_from_rotation_and_translation to confirm it reassembles a known transform.
    #[test]
    fn test_make_4x4_from_rotation_and_translation() {
        // We'll build a small 3x3 rotation => 30° about Z
        let c = (FRAC_PI_6.cos()) as f32;
        let s = (FRAC_PI_6.sin()) as f32;
        // Row-major rotation matrix for 30° about Z
        let rotation_3x3_row_major = [c, -s, 0.0, s, c, 0.0, 0.0, 0.0, 1.0];
        let translation_3_arr = [1.0_f32, -2.0_f32, 3.0_f32];

        let final_tf_row_major =
            make_4x4_from_rotation_and_translation(rotation_3x3_row_major, translation_3_arr);

        // Expected 4x4 (row-major):
        // [ c  -s   0   1.0,
        //   s   c   0  -2.0,
        //  0.0 0.0 1.0  3.0,
        //  0.0 0.0 0.0  1.0 ]
        let expected_tf = [
            c,   -s,  0.0,  1.0,
            s,    c,  0.0, -2.0,
            0.0, 0.0,  1.0,  3.0,
            0.0, 0.0,  0.0,  1.0,
        ];
        assert_arr_eq(&final_tf_row_major, &expected_tf, "make_4x4_from_rotation_and_translation");
    }
}