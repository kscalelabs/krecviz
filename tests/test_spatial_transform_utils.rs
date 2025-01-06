// tests/test_utils.rs

#[cfg(test)]
mod test_spatial_transform_utils {
    use krecviz::utils::spatial_transform_utils::{
        mat3x3_mul, mat4x4_mul, identity_4x4, build_z_rotation_3x3,
    };
    use std::f64::consts::FRAC_PI_2;

    #[test]
    fn test_mat3x3_mul_identity() {
        // 3x3 identity
        let id = [
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0
        ];
        // Some arbitrary matrix A
        let a = [
            1.0, 2.0, 3.0,
            4.0, 5.0, 6.0,
            7.0, 8.0, 9.0
        ];

        let result = mat3x3_mul(a, id);
        // Multiplying by identity should yield A
        assert_eq!(
            result, a,
            "mat3x3_mul with identity should return the original matrix."
        );
    }

    #[test]
    fn test_mat4x4_mul_identity() {
        // 4x4 identity
        let id_4x4 = identity_4x4();
        // Some arbitrary 4x4 matrix
        let a_4x4 = [
            1.0,  2.0,  3.0,  4.0,
            5.0,  6.0,  7.0,  8.0,
            9.0, 10.0, 11.0, 12.0,
            0.0,  1.0,  2.0,  3.0,
        ];

        let result = mat4x4_mul(a_4x4, id_4x4);
        // Multiplying by identity should yield the same matrix
        assert_eq!(
            result, a_4x4,
            "mat4x4_mul with identity should return the original matrix."
        );
    }

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
        let expected = [
            0.0, -1.0,  0.0,
            1.0,  0.0,  0.0,
            0.0,  0.0,  1.0,
        ];

        // Compare element-by-element
        for i in 0..9 {
            let actual_val = rot_90[i];
            let expected_val = expected[i];
            let diff = (actual_val - expected_val).abs();

            assert!(
                diff < 1e-6,
                "Element {} mismatch in build_z_rotation_3x3.\n  \
                 Got={}, Expected={}, angle_rad={:.4}\n  \
                 Entire matrix: {:?}",
                i, actual_val, expected_val, angle_rad, rot_90
            );
        }
    }
}
