use crate::spatial_transform_utils::{build_4x4_from_xyz_rpy, mat4x4_mul};
use crate::urdf_bfs_utils::{build_adjacency, find_root_link_name, get_link_chain};
use log::debug;
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
        "[frame={}] actuator_id={} => joint='{}' => entity_path='{}'",
        frame_idx, actuator_id, joint_name, entity_path
    );
    debug!(
        "  angle_deg={} => angle_rad={:.3} => transform_4x4={:?}",
        pos_deg, angle_rad, tf4x4
    );
    debug!("  => translation={:?}, mat3x3={:?}", translation, mat3x3);
}

/// Print debug information about loading an STL file.
pub fn debug_print_stl_load(abs_path: &std::path::Path) {
    debug!("Loading STL file: {:?}", abs_path);
}

/// Print debug information about rerun logging a mesh.
pub fn debug_print_mesh_log(entity_path: &str, mesh: &rerun::archetypes::Mesh3D) {
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
    joint_name: &str,
    child_link: &str,
    local_tf_4x4: [f32; 16],
    child_path: &str,
    rpy: [f64; 3],
) {
    debug!("----------------------");
    debug!(
        "Applying joint '{}' => child link '{}'",
        joint_name, child_link
    );
    debug!("child_path='{}'", child_path);

    debug!(
        "Original joint RPY values: [{:>8.3}, {:>8.3}, {:>8.3}]",
        rpy[0], rpy[1], rpy[2]
    );

    // Print full 4x4 matrix:
    for row_i in 0..4 {
        let base = row_i * 4;
        debug!(
            "[{:8.3} {:8.3} {:8.3} {:8.3}]",
            local_tf_4x4[base],
            local_tf_4x4[base + 1],
            local_tf_4x4[base + 2],
            local_tf_4x4[base + 3],
        );
    }

    debug!("mat3x3:");
    let (_translation, mat3x3) =
        crate::spatial_transform_utils::decompose_4x4_to_translation_and_mat3x3(local_tf_4x4);
    for row_i in 0..3 {
        let start = row_i * 3;
        debug!(
            "    [{:>8.3}, {:>8.3}, {:>8.3}]",
            mat3x3[start],
            mat3x3[start + 1],
            mat3x3[start + 2]
        );
    }
}

/// Print the final accumulated transforms for each link in the robot.
pub fn print_final_link_transforms(robot: &Robot) {
    if let Some(root_link) = find_root_link_name(&robot.links, &robot.joints) {
        let adjacency = build_adjacency(&robot.joints);

        debug!("\n========== FINAL ACCUMULATED TRANSFORMS PER LINK ==========");
        for link in &robot.links {
            if link.name == root_link {
                debug!(
                    "Link '{}': Root link => final transform is identity.\n",
                    link.name
                );
                continue;
            }
            if let Some(chain) = get_link_chain(&adjacency, &root_link, &link.name) {
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
