use crate::utils::urdf_bfs_utils::LinkBfsData;
use log::debug;
use rerun::components::Position3D;
use urdf_rs::Vec3;

/// A small helper that prints a standard header/log info:
/// - A title (e.g. "debug_log_rerun_transform")
/// - A reason (e.g. "Stage2 BFS apply transform")
/// - The entity path, if any
/// - The link's BFS info, if provided
fn debug_common_header(
    title: &str,
    label: &str,
    entity_path: Option<&str>,
    link_data: Option<&LinkBfsData>,
) {
    debug!("=== {} ===", title);
    debug!("Label: {}", label);

    if let Some(path) = entity_path {
        debug!("Entity path: '{}'", path);
    }

    if let Some(ld) = link_data {
        debug!("Link name:       '{}'", ld.link_name);
        debug!("Link-only path:  '{}'", ld.link_only_path);
        debug!("Link-full path:  '{}'", ld.link_full_path);
    }
}

/// Print debug information about a link being inserted during BFS traversal
pub fn debug_log_bfs_insertion(child_data: &LinkBfsData) {
    // BFS insertion usually doesn't have an "entity path" like rec.log
    // so we pass None here.
    debug_common_header(
        "debug_log_bfs_insertion",
        "BFS insertion",
        None, // no entity_path
        Some(child_data),
    );

    debug!(
        "local RPY: [{:.3}, {:.3}, {:.3}]",
        child_data.local_rpy[0], child_data.local_rpy[1], child_data.local_rpy[2]
    );
    debug!(
        "local XYZ: [{:.3}, {:.3}, {:.3}]",
        child_data.local_translation[0],
        child_data.local_translation[1],
        child_data.local_translation[2]
    );
    debug!("---------------------------------------\n");
}

/// Print debug information about a Transform3D before logging it to Rerun
pub fn debug_log_rerun_transform(
    entity_path: &str,
    link_data: Option<&LinkBfsData>,
    original_rpy: [f64; 3],
    translation: [f32; 3],
    mat3x3: [f32; 9],
    label: &str,
) {
    // We do have an entity_path here, so we pass Some(entity_path).
    debug_common_header(
        "debug_log_rerun_transform",
        label,
        Some(entity_path),
        link_data,
    );

    debug!(
        "Original RPY:    [{:.3}, {:.3}, {:.3}]",
        original_rpy[0], original_rpy[1], original_rpy[2]
    );
    debug!(
        "Final translation: [{:.3}, {:.3}, {:.3}]",
        translation[0], translation[1], translation[2]
    );

    debug!("Rotation Matrix (3x3):");
    for row in 0..3 {
        let idx = row * 3;
        debug!(
            "  [{:8.3}, {:8.3}, {:8.3}]",
            mat3x3[idx],
            mat3x3[idx + 1],
            mat3x3[idx + 2]
        );
    }
    debug!("======================================\n");
}

/// Print debug information about a Mesh3D before logging it to Rerun
pub fn debug_log_rerun_mesh(
    entity_path: &str,
    link_data: Option<&LinkBfsData>,
    origin_rpy: Vec3,
    origin_xyz: Vec3,
    vertex_positions: &[Position3D],
    label: &str,
) {
    // Same approach: we do have an entity path, so Some(entity_path).
    debug_common_header("debug_log_rerun_mesh", label, Some(entity_path), link_data);

    debug!(
        "Original RPY: [{:.3}, {:.3}, {:.3}]",
        origin_rpy[0], origin_rpy[1], origin_rpy[2]
    );
    debug!(
        "Original XYZ: [{:.3}, {:.3}, {:.3}]",
        origin_xyz[0], origin_xyz[1], origin_xyz[2]
    );

    let n_verts = vertex_positions.len();
    debug!("Number of mesh vertices: {}", n_verts);

    if n_verts > 0 {
        debug!("First 3 vertex positions:");
        for v in vertex_positions.iter().take(3) {
            debug!("   [{:.3}, {:.3}, {:.3}]", v[0], v[1], v[2]);
        }
        if n_verts > 3 {
            debug!("Last 3 vertex positions:");
            for v in vertex_positions.iter().skip(n_verts.saturating_sub(3)) {
                debug!("   [{:.3}, {:.3}, {:.3}]", v[0], v[1], v[2]);
            }
        }
    }

    debug!("======================================\n");
}

/// Print debug information about actuator states being logged to Rerun
pub fn debug_log_actuator_state(
    frame_idx: usize,
    actuator_id: u32,
    position: Option<f64>,
    velocity: Option<f64>,
    torque: Option<f64>,
) {
    debug_common_header(
        "debug_log_actuator_state",
        "Actuator state logging",
        None,
        None,
    );

    debug!("Frame index: {:>6}", frame_idx);
    debug!("Actuator ID: {:>6}", actuator_id);
    debug!("Position:   {:>9.3} deg", position.unwrap_or(f64::NAN));
    debug!("Velocity:   {:>9.3} deg/s", velocity.unwrap_or(f64::NAN));
    debug!("Torque:     {:>9.3} Nm", torque.unwrap_or(f64::NAN));
    debug!("---------------------------------------\n");
}
