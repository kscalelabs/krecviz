use crate::spatial_transform_utils::{build_4x4_from_xyz_rpy, mat4x4_mul};
use crate::urdf_bfs_utils::{find_root_link_name, get_link_chain};
use log::debug;
use std::collections::HashMap;
use urdf_rs::Robot;

/// Print debug information about an actuator transform.
pub fn debug_print_actuator_transform(
    frame_idx: usize,
    actuator_id: u32,
    joint_name: &str,
    entity_path: &str,
    pos_deg: f64,
    angle_rad: f64,
    tf4x4: [f32; 16],
    translation: [f32; 3],
    mat3x3: [f32; 9],
) {
    debug!(
        "=== debug_print_actuator_transform [Frame {}] ===",
        frame_idx
    );
    debug!("Actuator ID: {}", actuator_id);
    debug!("Joint Name:  '{}'", joint_name);
    debug!("Entity Path: '{}'", entity_path);
    debug!("Angle: {:.3}° ({:.3} rad)", pos_deg, angle_rad);

    debug!("Decomposed Transform:");
    debug!(
        "  Translation: [{:8.3}, {:8.3}, {:8.3}]",
        translation[0], translation[1], translation[2]
    );

    debug!("  Rotation Matrix (3x3):");
    for row in 0..3 {
        let idx = row * 3;
        debug!(
            "    [{:8.3} {:8.3} {:8.3}]",
            mat3x3[idx],
            mat3x3[idx + 1],
            mat3x3[idx + 2]
        );
    }
    debug!("=====================================\n");
}

/// Print debug information about loading an STL file.
pub fn debug_print_stl_load(abs_path: &std::path::Path) {
    debug!("=== debug_print_stl_load ===");
    debug!("Loading STL file: {:?}", abs_path);
}

/// Print debug information about rerun logging a mesh.
pub fn debug_print_mesh_log(entity_path: &str, mesh: &rerun::archetypes::Mesh3D) {
    debug!("=== debug_print_mesh_log ===");
    debug!("======================");
    debug!("rerun_log log_trimesh");
    debug!("entity_path = '{entity_path}'");
    debug!("entity = rr.Mesh3D(...) with these numeric values:");
    debug!("  => vertex_positions (first 3):");
    for v in mesh.vertex_positions.iter().take(3) {
        debug!("      [{:>7.3}, {:>7.3}, {:>7.3}]", v[0], v[1], v[2]);
    }
    let timeless_val = true;
    debug!("timeless = {timeless_val}");
}

/// Print debug information about applying a joint transform.
pub fn debug_print_joint_transform(
    _joint_name: &str,
    _child_link: &str,
    local_tf_4x4: [f32; 16],
    child_path: &str,
    rpy: [f64; 3],
) {
    debug!("=== debug_print_joint_transform ===");
    debug!("rerun_log");
    debug!(
        "entity_path = entity_path_w_prefix with value '{}'",
        child_path
    );
    debug!("Original joint RPY values:");
    debug!("  => rpy = [{:.3}, {:.3}, {:.3}]", rpy[0], rpy[1], rpy[2]);

    let (translation, mat3x3) =
        crate::spatial_transform_utils::decompose_4x4_to_translation_and_mat3x3(local_tf_4x4);

    debug!("entity = rr.Transform3D with:");
    debug!(
        "  translation: ['{:>8.3}', '{:>8.3}', '{:>8.3}']",
        translation[0], translation[1], translation[2]
    );

    debug!("  mat3x3:");
    for row_i in 0..3 {
        let start = row_i * 3;
        debug!(
            "    [{:>8.3}, {:>8.3}, {:>8.3}]",
            mat3x3[start],
            mat3x3[start + 1],
            mat3x3[start + 2]
        );
    }
    debug!("======================");
}

/// Print the final accumulated transforms for each link in the robot.
pub fn print_final_link_transforms(robot: &Robot, link_paths_map: &HashMap<String, Vec<String>>) {
    debug!("=== print_final_link_transforms ===");
    if let Some(root_link) = find_root_link_name(&robot.links, &robot.joints) {
        debug!("========== FINAL ACCUMULATED TRANSFORMS PER LINK ==========");
        for link in &robot.links {
            if link.name == root_link {
                debug!(
                    "Link '{}': Root link => final transform is identity.\n",
                    link.name
                );
                continue;
            }
            if let Some(chain) = get_link_chain(link_paths_map, &link.name) {
                let mut final_tf = [
                    1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
                ];
                let mut i = 1;
                while i < chain.len() {
                    let joint_name = &chain[i];
                    i += 1;
                    let j = robot.joints.iter().find(|jj| jj.name == *joint_name);
                    if let Some(joint) = j {
                        let x = joint.origin.xyz[0];
                        let y = joint.origin.xyz[1];
                        let z = joint.origin.xyz[2];
                        let rr = joint.origin.rpy[0];
                        let pp = joint.origin.rpy[1];
                        let yy = joint.origin.rpy[2];

                        let local_tf_4x4 = build_4x4_from_xyz_rpy([x, y, z], [rr, pp, yy]);
                        final_tf = mat4x4_mul(final_tf, local_tf_4x4);
                    }
                    i += 1;
                }

                debug!("Link '{}': BFS chain = {:?}", link.name, chain);
                debug!("  => final_tf (4x4) =");
                for row_i in 0..4 {
                    let base = row_i * 4;
                    debug!(
                        "  [{:8.3} {:8.3} {:8.3} {:8.3}]",
                        final_tf[base + 0],
                        final_tf[base + 1],
                        final_tf[base + 2],
                        final_tf[base + 3]
                    );
                }
                debug!("");
            }
        }
    }
}

/// Print debug information about transform chain and matrices
pub fn debug_print_transform_chain(
    bfs_chain_for_debug: &[String],
    i: usize,
    local_tf_4x4: [f32; 16],
    global_tf: [f32; 16],
) {
    debug!("=== debug_print_transform_chain ===");
    debug!("Transform chain so far:");
    debug!("  {}", bfs_chain_for_debug[..=i].join(" -> "));

    debug!("Local transform matrix:");
    for row in 0..4 {
        let row_str = format!(
            "  [{:7.3}, {:7.3}, {:7.3}, {:7.3}]",
            (local_tf_4x4[row * 4 + 0] * 1000.0).round() / 1000.0,
            (local_tf_4x4[row * 4 + 1] * 1000.0).round() / 1000.0,
            (local_tf_4x4[row * 4 + 2] * 1000.0).round() / 1000.0,
            (local_tf_4x4[row * 4 + 3] * 1000.0).round() / 1000.0
        );
        debug!("{}", row_str);
    }

    debug!("Accumulated global transform:");
    for row in 0..4 {
        let row_str = format!(
            "  [{:7.3}, {:7.3}, {:7.3}, {:7.3}]",
            (global_tf[row * 4 + 0] * 1000.0).round() / 1000.0,
            (global_tf[row * 4 + 1] * 1000.0).round() / 1000.0,
            (global_tf[row * 4 + 2] * 1000.0).round() / 1000.0,
            (global_tf[row * 4 + 3] * 1000.0).round() / 1000.0
        );
        debug!("{}", row_str);
    }
    debug!("");
}

/// Print debug information about link transforms and mesh paths
pub fn debug_print_link_transform_info(
    link_name: &str,
    bfs_chain: &[String],
    final_tf: [f32; 16],
    mesh_entity_path: &str,
    rpy: [f64; 3],
) {
    debug!("=== debug_print_link_transform_info ===");
    debug!("");
    debug!("Link '{link_name}': BFS chain = {:?}", bfs_chain);
    debug!("  => final_tf (4x4) =");
    for row_i in 0..4 {
        let base = row_i * 4;
        debug!(
            "  [{:8.3} {:8.3} {:8.3} {:8.3}]",
            final_tf[base + 0],
            final_tf[base + 1],
            final_tf[base + 2],
            final_tf[base + 3],
        );
    }
    debug!("");
    debug!("entity_path = '{mesh_entity_path}'");
    debug!("Original joint RPY values:");
    debug!("  => rpy = [{:.3}, {:.3}, {:.3}]", rpy[0], rpy[1], rpy[2]);

    let (translation, _mat3x3) =
        crate::spatial_transform_utils::decompose_4x4_to_translation_and_mat3x3(final_tf);
    debug!("entity = rr.Transform3D with:");
    debug!(
        "  translation: ['{:>8.3}', '{:>8.3}', '{:>8.3}']",
        translation[0], translation[1], translation[2]
    );
    debug!("");
}

/// Print detailed debug information about BFS joint transforms including parent and child transforms
pub fn debug_print_bfs_joint_transforms(
    chain_str: &str,
    parent: &str,
    child_link: &str,
    rpy: [f64; 3],
    xyz: [f64; 3],
    parent_accum: [f32; 16],
    local_tf_4x4: [f32; 16],
    child_accum: [f32; 16],
    local_translation: [f32; 3],
    local_mat3x3: [f32; 9],
) {
    debug!("=== debug_print_bfs_joint_transforms ===");
    debug!("================== BFS Joint Transform Debug ==================");
    debug!("BFS chain (entity path) = '{}'", chain_str);
    debug!("Parent link='{}', child link='{}'", parent, child_link);
    debug!(
        "Original joint RPY = [{:.3}, {:.3}, {:.3}]",
        rpy[0], rpy[1], rpy[2]
    );
    debug!(
        "Original joint XYZ = [{:.3}, {:.3}, {:.3}]",
        xyz[0], xyz[1], xyz[2]
    );

    debug!("--- Parent accum transform (root->parent):");
    for row_i in 0..4 {
        debug!(
            "  [{:8.3}, {:8.3}, {:8.3}, {:8.3}]",
            parent_accum[row_i * 4 + 0],
            parent_accum[row_i * 4 + 1],
            parent_accum[row_i * 4 + 2],
            parent_accum[row_i * 4 + 3],
        );
    }

    debug!("--- Local joint transform (parent->child):");
    for row_i in 0..4 {
        debug!(
            "  [{:8.3}, {:8.3}, {:8.3}, {:8.3}]",
            local_tf_4x4[row_i * 4 + 0],
            local_tf_4x4[row_i * 4 + 1],
            local_tf_4x4[row_i * 4 + 2],
            local_tf_4x4[row_i * 4 + 3],
        );
    }

    debug!("--- child_accum (root->child):");
    for row_i in 0..4 {
        debug!(
            "  [{:8.3}, {:8.3}, {:8.3}, {:8.3}]",
            child_accum[row_i * 4 + 0],
            child_accum[row_i * 4 + 1],
            child_accum[row_i * 4 + 2],
            child_accum[row_i * 4 + 3],
        );
    }

    debug!("--- Will log to Rerun => local_tf (since we only pass local offset here) :");
    debug!(
        "    local translation = [{:.3}, {:.3}, {:.3}]",
        local_translation[0], local_translation[1], local_translation[2]
    );
    debug!("    local rotation 3x3 = {:?}", local_mat3x3);

    let (translation, mat3x3) =
        crate::spatial_transform_utils::decompose_4x4_to_translation_and_mat3x3(local_tf_4x4);
    debug!("Now rec.log('{}', &tf) => That transform3D is:", chain_str);
    debug!(
        "   translation=[{:.3}, {:.3}, {:.3}]",
        translation[0], translation[1], translation[2]
    );
    debug!("   mat3x3={:?}", mat3x3);
    debug!("==============================================================\n");
}
