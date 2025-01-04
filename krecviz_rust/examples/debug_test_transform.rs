use nalgebra::{Matrix3, Rotation3, Vector3};
use std::f64::consts::{FRAC_PI_2, PI};

// Adjust your crate's path as needed:
use krecviz_rust::spatial_transform_utils::rotation_from_euler_xyz;

fn print_rowmajor_3x3_old(label: &str, angles: [f64; 3], mat: [f32; 9]) {
    println!(
        "\n=== {} (OLD) ===\nAngles (rx, ry, rz) = ({:.4}, {:.4}, {:.4})",
        label, angles[0], angles[1], angles[2]
    );
    for row in 0..3 {
        let start = row * 3;
        println!(
            " row {}: [{:8.4}, {:8.4}, {:8.4}]",
            row,
            mat[start],
            mat[start + 1],
            mat[start + 2]
        );
    }
    println!("======================================");
}

fn print_rowmajor_3x3_nalgebra(label: &str, angles: [f64; 3], mat: &Matrix3<f32>) {
    println!(
        "\n--- {} (NALGEBRA) ---\nAngles (rx, ry, rz) = ({:.4}, {:.4}, {:.4})",
        label, angles[0], angles[1], angles[2]
    );
    for row in 0..3 {
        println!(
            " row {}: [{:8.4}, {:8.4}, {:8.4}]",
            row,
            mat[(row, 0)],
            mat[(row, 1)],
            mat[(row, 2)]
        );
    }
    println!("--------------------------------------");
}

fn main() {
    // Some angles near ±π from your logs:
    let angle_sets: &[[f64; 3]] = &[
        [-3.142,  0.000, -3.142],
        [ 3.142, -0.000,  3.142],
        [ PI,     0.0,    PI],       // exact pi
        [-PI,     0.0,   -PI],
        [FRAC_PI_2, FRAC_PI_2, 0.0], // etc...
    ];

    for &angles in angle_sets {
        let (rx, ry, rz) = (angles[0], angles[1], angles[2]);

        // 1) Old code row-major
        let old_mat = rotation_from_euler_xyz(rx, ry, rz);
        print_rowmajor_3x3_old("Old Code rotation_from_euler_xyz", angles, old_mat);

        // 2) Let's do each nalgebra permutation with named variables to avoid ephemeral references:
        let rot_x = Rotation3::from_axis_angle(&Vector3::x_axis(), rx as f32);
        let rot_y = Rotation3::from_axis_angle(&Vector3::y_axis(), ry as f32);
        let rot_z = Rotation3::from_axis_angle(&Vector3::z_axis(), rz as f32);

        // Permutation A: (z * y * x)
        let combo_a = rot_z * rot_y * rot_x;
        let test_a  = combo_a.matrix();  // reference to the underlying Matrix3
        print_rowmajor_3x3_nalgebra("test_a => (Z * Y * X)", angles, test_a);

        // Permutation B: (x * y * z)
        let combo_b = rot_x * rot_y * rot_z;
        let test_b  = combo_b.matrix();
        print_rowmajor_3x3_nalgebra("test_b => (X * Y * Z)", angles, test_b);

        // Permutation C: (z * x * y)
        let combo_c = rot_z * rot_x * rot_y;
        let test_c  = combo_c.matrix();
        print_rowmajor_3x3_nalgebra("test_c => (Z * X * Y)", angles, test_c);

        // Permutation D: (y * z * x)
        let combo_d = rot_y * rot_z * rot_x;
        let test_d  = combo_d.matrix();
        print_rowmajor_3x3_nalgebra("test_d => (Y * Z * X)", angles, test_d);

        // Permutation E: (y * x * z)
        let combo_e = rot_y * rot_x * rot_z;
        let test_e  = combo_e.matrix();
        print_rowmajor_3x3_nalgebra("test_e => (Y * X * Z)", angles, test_e);

        // Permutation F: (x * z * y)
        let combo_f = rot_x * rot_z * rot_y;
        let test_f  = combo_f.matrix();
        print_rowmajor_3x3_nalgebra("test_f => (X * Z * Y)", angles, test_f);

        println!("\n#######################################################\n");
    }
}
