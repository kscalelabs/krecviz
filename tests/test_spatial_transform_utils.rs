// tests/test_utils.rs

#[cfg(test)]
mod test_spatial_transform_utils {
    use krecviz::utils::spatial_transform_utils::{
        build_4x4_from_xyz_rpy, build_z_rotation_3x3, decompose_4x4_to_translation_and_mat3x3,
        identity_4x4, make_4x4_from_rotation_and_translation, mat3x3_mul, mat4x4_mul,
        rotation_from_euler_xyz,
    };
    use std::f64::consts::{FRAC_PI_2, FRAC_PI_6}; // 90°, 30°

    /// Test multiplying a 3x3 matrix by the identity matrix.
    #[test]
    fn test_mat3x3_mul_identity() {
        let id_3x3 = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0];
        let a_3x3 = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0];

        let result = mat3x3_mul(a_3x3, id_3x3);
        assert_eq!(
            result, a_3x3,
            "mat3x3_mul with identity should return the original matrix."
        );
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
        for i in 0..9 {
            let diff = (result[i] - expected[i]).abs();
            assert!(
                diff < 1e-6,
                "Mismatch at index {} in mat3x3_mul_arbitrary: got={}, expected={}",
                i,
                result[i],
                expected[i]
            );
        }
    }

    /// Test multiplying a 4x4 matrix by the 4x4 identity matrix.
    #[test]
    fn test_mat4x4_mul_identity() {
        let id_4x4 = identity_4x4();
        let a_4x4 = [
            1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0, 16.0,
        ];

        let result = mat4x4_mul(a_4x4, id_4x4);
        assert_eq!(
            result, a_4x4,
            "mat4x4_mul with identity should return the original matrix."
        );
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
        for i in 0..16 {
            let diff = (result[i] - expected[i]).abs();
            assert!(
                diff < 1e-6,
                "Mismatch at index {} in mat4x4_mul_arbitrary: got={}, expected={}",
                i,
                result[i],
                expected[i]
            );
        }
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

        for i in 0..9 {
            let diff = (rot_90[i] - expected[i]).abs();
            assert!(
                diff < 1e-6,
                "Index {} mismatch in build_z_rotation_3x3: got={}, expected={}",
                i,
                rot_90[i],
                expected[i]
            );
        }
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

        for i in 0..9 {
            let diff = (rot[i] - expected[i]).abs();
            assert!(
                diff < 1e-6,
                "Index {} mismatch in rotation_from_euler_xyz(90,0,0). got={}, expected={}",
                i,
                rot[i],
                expected[i]
            );
        }
    }

    /// Test build_4x4_from_xyz_rpy for a known translation & rotation.
    #[test]
    fn test_build_4x4_from_xyz_rpy_30deg() {
        // Suppose we want to move by (1.0, 2.0, 3.0), then do a 30° turn about Z.
        let xyz = [1.0, 2.0, 3.0];
        let rpy = [0.0, 0.0, FRAC_PI_6]; // 30° about Z
        let tf = build_4x4_from_xyz_rpy(xyz, rpy);

        // cos(30°) ≈ 0.8660254, sin(30°) = 0.5
        let c = FRAC_PI_6.cos() as f32;
        let s = FRAC_PI_6.sin() as f32;

        // Check translation in row-major => tf[3], tf[7], tf[11]
        assert!((tf[3] - 1.0).abs() < 1e-6, "X translation mismatch");
        assert!((tf[7] - 2.0).abs() < 1e-6, "Y translation mismatch");
        assert!((tf[11] - 3.0).abs() < 1e-6, "Z translation mismatch");

        // The top-left 3x3 should be a 30° rotation around Z:
        //   [ cos(30°), -sin(30°),  0 ]
        //   [ sin(30°),  cos(30°),  0 ]
        //   [    0,          0,     1 ]
        let row0 = [tf[0], tf[1], tf[2]];
        assert!((row0[0] - c).abs() < 1e-6 && (row0[1] + s).abs() < 1e-6 && row0[2].abs() < 1e-6);

        let row1 = [tf[4], tf[5], tf[6]];
        assert!((row1[0] - s).abs() < 1e-6 && (row1[1] - c).abs() < 1e-6 && row1[2].abs() < 1e-6);

        let row2 = [tf[8], tf[9], tf[10]];
        assert!(row2[0].abs() < 1e-6 && row2[1].abs() < 1e-6 && (row2[2] - 1.0).abs() < 1e-6);
    }

    /// Test decompose_4x4_to_translation_and_mat3x3 on a known 4x4 transform.
    #[test]
    fn test_decompose_4x4_to_translation_and_mat3x3() {
        // We'll create a transform with build_4x4_from_xyz_rpy:
        let xyz = [2.0, -3.0, 5.0];
        let rpy = [FRAC_PI_2, 0.0, 0.0]; // 90° about X
        let full_tf = build_4x4_from_xyz_rpy(xyz, rpy);

        let (translation, mat3x3) = decompose_4x4_to_translation_and_mat3x3(full_tf);
        assert!(
            (translation[0] - 2.0).abs() < 1e-6
                && (translation[1] + 3.0).abs() < 1e-6
                && (translation[2] - 5.0).abs() < 1e-6,
            "Translation mismatch in decompose_4x4_to_translation_and_mat3x3"
        );
        // 90° about X => column-major =>
        //  [ 1,  0,  0,
        //    0,  0, 1,
        //    0, -1,  0]
        let expected_3x3 = [1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, -1.0, 0.0];
        for i in 0..9 {
            let diff = (mat3x3[i] - expected_3x3[i]).abs();
            assert!(
                diff < 1e-6,
                "Matrix mismatch at index {} in decompose_4x4_to_translation_and_mat3x3. got={}, expected={}",
                i, mat3x3[i], expected_3x3[i]
            );
        }
    }

    /// Test make_4x4_from_rotation_and_translation to confirm it reassembles a known transform.
    #[test]
    fn test_make_4x4_from_rotation_and_translation() {
        // We'll build a small 3x3 rotation => 30° about Z
        let c = FRAC_PI_6.cos() as f32;
        let s = FRAC_PI_6.sin() as f32;
        let rotation_3x3 = [c, -s, 0.0, s, c, 0.0, 0.0, 0.0, 1.0];
        let translation_3 = [1.0_f32, -2.0_f32, 3.0_f32];

        let final_tf = make_4x4_from_rotation_and_translation(rotation_3x3, translation_3);
        // Verify final_tf's top-left 3x3
        assert!((final_tf[0] - c).abs() < 1e-6, "M[0,0] mismatch");
        assert!((final_tf[1] + s).abs() < 1e-6, "M[0,1] mismatch");
        assert!(final_tf[2].abs() < 1e-6, "M[0,2] mismatch");
        assert!((final_tf[4] - s).abs() < 1e-6, "M[1,0] mismatch");
        assert!((final_tf[5] - c).abs() < 1e-6, "M[1,1] mismatch");
        assert!(final_tf[6].abs() < 1e-6, "M[1,2] mismatch");
        // 2nd row => [0,0,1]
        assert!(final_tf[8].abs() < 1e-6, "M[2,0] mismatch");
        assert!(final_tf[9].abs() < 1e-6, "M[2,1] mismatch");
        assert!((final_tf[10] - 1.0).abs() < 1e-6, "M[2,2] mismatch");

        // Check translation => row-major => indices [3, 7, 11]
        assert!((final_tf[3] - 1.0).abs() < 1e-6, "X mismatch");
        assert!((final_tf[7] + 2.0).abs() < 1e-6, "Y mismatch");
        assert!((final_tf[11] - 3.0).abs() < 1e-6, "Z mismatch");

        // The last row should remain [0, 0, 0, 1]
        assert!(
            final_tf[12].abs() < 1e-6 && final_tf[13].abs() < 1e-6 && final_tf[14].abs() < 1e-6
        );
        assert!((final_tf[15] - 1.0).abs() < 1e-6, "M[3,3] mismatch");
    }
}
