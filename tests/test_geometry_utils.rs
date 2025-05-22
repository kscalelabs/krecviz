// tests/test_geometry_utils.rs

use krecviz::utils::geometry_utils::{
    compute_vertex_normals, apply_4x4_to_mesh3d, float_rgba_to_u8, 
    create_box_mesh, create_cylinder_mesh, create_sphere_mesh,
};
use krecviz::utils::spatial_transform_utils;

use rerun::{
    archetypes::Mesh3D,
    components::{Position3D, TriangleIndices, Vector3D},
};

const EPSILON: f32 = 1e-6;

// Helper to compare slices of f32
fn assert_f32_slice_approx_eq(a: &[f32], b: &[f32], message: &str) {
    assert_eq!(a.len(), b.len(), "{}: Slice length mismatch", message);
    for i in 0..a.len() {
        assert!(
            (a[i] - b[i]).abs() < EPSILON,
            "{}: Index {} mismatch. Expected {}, Got {}",
            message,
            i,
            b[i],
            a[i]
        );
    }
}

/// Test normals are computed, the count is correct, and they point in the expected direction (+Z).
#[test]
fn test_compute_vertex_normals_simple_triangle() {
    let positions = vec![
        Position3D::from([0.0, 0.0, 0.0]),
        Position3D::from([1.0, 0.0, 0.0]),
        Position3D::from([0.0, 1.0, 0.0]),
    ];
    let indices = vec![TriangleIndices::from([0, 1, 2])];
    let mut mesh = Mesh3D::new(positions).with_triangle_indices(indices);

    compute_vertex_normals(&mut mesh);

    assert!(mesh.vertex_normals.is_some(), "Normals should be computed");
    let normals = mesh.vertex_normals.as_ref().unwrap();
    assert_eq!(normals.len(), 3, "Should have 3 normals");

    let expected_normal_val = Vector3D::from([0.0, 0.0, 1.0]);
    for n_vector_3d in normals {
        // n_vector_3d is &Vector3D. n_vector_3d.0 is datatypes::Vec3D. n_vector_3d.0.0 is [f32;3]
        assert_f32_slice_approx_eq(&n_vector_3d.0.0, &expected_normal_val.0.0, "Normal vector mismatch");
    }
}

/// Test normals are computed, have the correct count, and are approximately normalized.
#[test]
fn test_compute_vertex_normals_cube() {
    // Removed `mut` as cube_mesh is not modified after creation in this test's scope
    let cube_mesh = create_box_mesh([1.0, 1.0, 1.0]);
    // compute_vertex_normals is already called within create_box_mesh

    assert!(cube_mesh.vertex_normals.is_some(), "Cube normals should be computed");
    let normals = cube_mesh.vertex_normals.as_ref().unwrap();
    assert_eq!(normals.len(), cube_mesh.vertex_positions.len(), "Should have one normal per vertex");

    for n_vector_3d in normals {
        let n_arr = n_vector_3d.0.0;
        let len_sq = n_arr[0].powi(2) + n_arr[1].powi(2) + n_arr[2].powi(2);
        assert!(
            (len_sq.sqrt() - 1.0).abs() < EPSILON || len_sq.abs() < EPSILON,
            "Normal vector {:?} is not normalized (length sqrt({}) = {})", n_arr, len_sq, len_sq.sqrt()
        );
    }
}

/// Test vertex positions are correctly translated.
#[test]
fn test_apply_4x4_to_mesh3d_translation() {
    let mut mesh = create_box_mesh([1.0, 1.0, 1.0]);
    let original_positions: Vec<Position3D> = mesh.vertex_positions.clone();

    let translation_tf = spatial_transform_utils::build_4x4_from_xyz_rpy(
        [10.0, 20.0, 30.0],
        [0.0, 0.0, 0.0],
    );

    apply_4x4_to_mesh3d(&mut mesh, translation_tf);

    assert_eq!(mesh.vertex_positions.len(), original_positions.len());
    for i in 0..original_positions.len() {
        let op_arr = original_positions[i].0.0;
        let tp_arr = mesh.vertex_positions[i].0.0;
        assert!(
            (tp_arr[0] - (op_arr[0] + 10.0)).abs() < EPSILON,
            "X translation failed for vertex {}", i
        );
        assert!(
            (tp_arr[1] - (op_arr[1] + 20.0)).abs() < EPSILON,
            "Y translation failed for vertex {}", i
        );
        assert!(
            (tp_arr[2] - (op_arr[2] + 30.0)).abs() < EPSILON,
            "Z translation failed for vertex {}", i
        );
    }
}

/// Test both vertex positions and normals are correctly rotated.
#[test]
fn test_apply_4x4_to_mesh3d_rotation() {
    let mut mesh = Mesh3D::new(vec![Position3D::from([1.0, 0.0, 0.0])])
        .with_vertex_normals(vec![Vector3D::from([1.0,0.0,0.0])]);

    let rotation_tf = spatial_transform_utils::build_4x4_from_xyz_rpy(
        [0.0, 0.0, 0.0],
        [0.0, 0.0, std::f64::consts::FRAC_PI_2],
    );
    apply_4x4_to_mesh3d(&mut mesh, rotation_tf);

    let transformed_pos_arr = mesh.vertex_positions[0].0.0;
    assert!(
        (transformed_pos_arr[0] - 0.0).abs() < EPSILON,
        "Rotated X mismatch"
    );
    assert!(
        (transformed_pos_arr[1] - 1.0).abs() < EPSILON,
        "Rotated Y mismatch"
    );
    assert!(
        (transformed_pos_arr[2] - 0.0).abs() < EPSILON,
        "Rotated Z mismatch"
    );

    let transformed_normal_arr = mesh.vertex_normals.as_ref().unwrap()[0].0.0;
     assert!(
        (transformed_normal_arr[0] - 0.0).abs() < EPSILON,
        "Rotated Normal X mismatch"
    );
    assert!(
        (transformed_normal_arr[1] - 1.0).abs() < EPSILON,
        "Rotated Normal Y mismatch"
    );
    assert!(
        (transformed_normal_arr[2] - 0.0).abs() < EPSILON,
        "Rotated Normal Z mismatch"
    );
}

/// Test a mesh is created with positions, indices, and normals,
/// and that vertex positions are within the expected geometric bounds.
#[test]
fn test_create_box_properties() {
    let size = [2.0, 3.0, 4.0];
    let mesh = create_box_mesh(size); // No mut needed here
    assert!(!mesh.vertex_positions.is_empty());
    assert!(
        mesh.triangle_indices.is_some() && !mesh.triangle_indices.as_ref().unwrap().is_empty()
    );
    assert!(mesh.vertex_normals.is_some());
    assert_eq!(
        mesh.vertex_positions.len(),
        mesh.vertex_normals.as_ref().unwrap().len()
    );

    let hx = size[0] as f32 / 2.0;
    let hy = size[1] as f32 / 2.0;
    let hz = size[2] as f32 / 2.0;

    for p_rerun in &mesh.vertex_positions {
        let p_arr = p_rerun.0.0;
        assert!(
            p_arr[0].abs() - hx <= EPSILON,
            "X bound error: {} vs {}",
            p_arr[0], hx
        );
        assert!(
            p_arr[1].abs() - hy <= EPSILON,
            "Y bound error: {} vs {}",
            p_arr[1], hy
        );
        assert!(
            p_arr[2].abs() - hz <= EPSILON,
            "Z bound error: {} vs {}",
            p_arr[2], hz
        );
    }
}

/// Test a mesh is created with positions, indices, and normals,
/// and that vertex positions conform to the expected cylindrical shape and dimensions.
#[test]
fn test_create_cylinder_properties() {
    let radius = 1.0;
    let length = 5.0;
    let mesh = create_cylinder_mesh(radius, length); // No mut
    assert!(!mesh.vertex_positions.is_empty());
    assert!(
        mesh.triangle_indices.is_some() && !mesh.triangle_indices.as_ref().unwrap().is_empty()
    );
    assert!(mesh.vertex_normals.is_some());
    assert_eq!(
        mesh.vertex_positions.len(),
        mesh.vertex_normals.as_ref().unwrap().len()
    );

    let half_len_f32 = length as f32 / 2.0;
    let radius_f32 = radius as f32;
    for p_rerun in &mesh.vertex_positions {
        let p_arr = p_rerun.0.0;
        assert!(
            p_arr[2].abs() - half_len_f32 <= EPSILON,
            "Cylinder Z bound error: {} vs {}",
            p_arr[2], half_len_f32
        );
        let xy_dist_sq = p_arr[0].powi(2) + p_arr[1].powi(2);
        assert!(
            xy_dist_sq.sqrt() - radius_f32 <= EPSILON,
            "Cylinder XY radius error: {} vs {}",
            xy_dist_sq.sqrt(), radius_f32
        );
    }
}

/// Test a mesh is created with positions, indices, and normals,
/// and that all vertex positions are approximately at the specified radius from the origin.
#[test]
fn test_create_sphere_properties() {
    let radius = 2.5;
    let mesh = create_sphere_mesh(radius); // No mut
    assert!(!mesh.vertex_positions.is_empty());
    assert!(
        mesh.triangle_indices.is_some() && !mesh.triangle_indices.as_ref().unwrap().is_empty()
    );
    assert!(mesh.vertex_normals.is_some());
    assert_eq!(
        mesh.vertex_positions.len(),
        mesh.vertex_normals.as_ref().unwrap().len()
    );

    let radius_f32 = radius as f32;
    for p_rerun in &mesh.vertex_positions {
        let p_arr = p_rerun.0.0;
        let dist_sq = p_arr[0].powi(2) + p_arr[1].powi(2) + p_arr[2].powi(2);
        assert!(
            (dist_sq.sqrt() - radius_f32).abs() <= EPSILON,
            "Sphere radius error for point {:?}. Expected {}, got {}",
            p_arr, radius_f32, dist_sq.sqrt()
        );
    }
}

/// Test correct conversion to 8-bit RGBA values, including clamping.
#[test]
fn test_float_rgba_to_u8_conversion() {
    assert_eq!(float_rgba_to_u8([0.0, 0.0, 0.0, 0.0]), [0, 0, 0, 0]);
    assert_eq!(float_rgba_to_u8([1.0, 1.0, 1.0, 1.0]), [255, 255, 255, 255]);
    assert_eq!(float_rgba_to_u8([0.5, 0.5, 0.5, 0.5]), [127, 127, 127, 127]);
    assert_eq!(float_rgba_to_u8([0.2, 0.4, 0.6, 0.8]), [51, 102, 153, 204]);
    assert_eq!(float_rgba_to_u8([-0.1, 1.1, 0.5, 0.0]), [0, 255, 127, 0]);
}