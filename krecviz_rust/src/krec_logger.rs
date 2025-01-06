// krec_logger.rs

use anyhow::Result;
use krec::KRec;
use log::info;
use rerun::RecordingStream;
use std::collections::HashMap;
use std::f64::consts::PI;

use crate::utils::debug_log_utils::{debug_log_actuator_state, debug_log_rerun_transform};
use crate::utils::spatial_transform_utils::{
    build_z_rotation_3x3, decompose_4x4_to_translation_and_mat3x3,
    make_4x4_from_rotation_and_translation, mat3x3_mul,
};
use crate::utils::urdf_bfs_utils::build_joint_name_to_joint_info;

// -----------------------------------------------------------------------------
// Actuator -> Joint map
// -----------------------------------------------------------------------------
fn build_actuator_to_urdf_joint_map() -> HashMap<u32, &'static str> {
    let mut map = HashMap::new();
    // Left Arm
    map.insert(11, "Revolute_2");
    map.insert(12, "Revolute_4");
    map.insert(13, "Revolute_5");
    map.insert(14, "Revolute_8");
    map.insert(15, "Revolute_16");
    // Right Arm
    map.insert(21, "Revolute_1");
    map.insert(22, "Revolute_3");
    map.insert(23, "Revolute_6");
    map.insert(24, "Revolute_7");
    map.insert(25, "Revolute_15");
    // Left Leg
    map.insert(31, "L_hip_y");
    map.insert(32, "L_hip_x");
    map.insert(33, "L_hip_z");
    map.insert(34, "L_knee");
    map.insert(35, "L_ankle_y");
    // Right Leg
    map.insert(41, "R_hip_y");
    map.insert(42, "R_hip_x");
    map.insert(43, "R_hip_z");
    map.insert(44, "R_knee");
    map.insert(45, "R_ankle_y");

    map
}

/// Log basic scalar values for an actuator (like position, velocity, torque) if present.
fn log_actuator_states(
    rec: &RecordingStream,
    frame_idx: usize,
    actuator_id: u32,
    position: Option<f64>,
    velocity: Option<f64>,
    torque: Option<f64>,
) -> Result<()> {
    // Add debug logging
    debug_log_actuator_state(frame_idx, actuator_id, position, velocity, torque);

    rec.set_time_sequence("frame_idx", frame_idx as i64);

    let base_path = format!("actuators/actuator_{}/state", actuator_id);

    if let Some(pos) = position {
        rec.log(
            format!("{}/position", base_path),
            &rerun::components::Scalar::from(pos),
        )?;
    }
    if let Some(vel) = velocity {
        rec.log(
            format!("{}/velocity", base_path),
            &rerun::components::Scalar::from(vel),
        )?;
    }
    if let Some(tor) = torque {
        rec.log(
            format!("{}/torque", base_path),
            &rerun::components::Scalar::from(tor),
        )?;
    }

    Ok(())
}

/// Parse and log a KREC file, optionally using URDF joint information for transforms
pub fn parse_and_log_krec(
    krec: &KRec,
    urdf_path: Option<&str>,
    rec: &RecordingStream,
) -> Result<()> {
    // We'll replicate the python actuator->joint map
    let actuator_map = build_actuator_to_urdf_joint_map();

    let joint_info_map = if let Some(urdf_path) = urdf_path {
        build_joint_name_to_joint_info(urdf_path)?
    } else {
        HashMap::new()
    };

    let mut frames_processed = 0;

    // Iterate frames
    for (frame_idx, frame) in krec.frames.iter().enumerate() {
        // Set Rerun time-sequence so transforms appear "animated"
        rec.set_time_sequence("frame_idx", frame_idx as i64);

        let mut frame_had_valid_data = false;

        for state in &frame.actuator_states {
            let actuator_id = state.actuator_id;

            // 1) Early-exit from "missing" joint_name
            let Some(joint_name) = actuator_map.get(&actuator_id) else {
                log::warn!("Frame {}: Actuator {} not found in actuator->joint map, skipping", frame_idx, actuator_id);
                continue;
            };

            // 2) Early-exit from "missing" joint_info
            let Some(joint_info) = joint_info_map.get(*joint_name) else {
                log::warn!("Frame {}: Joint '{}' not found in URDF joint info map, skipping", frame_idx, joint_name);
                continue;
            };

            // 3) Early-exit from missing position
            let Some(pos_deg) = state.position else {
                log::warn!("Frame {}: No position data for actuator {}, skipping", frame_idx, actuator_id);
                continue;
            };

            // Now do the transform logic
            let angle_rad = pos_deg * (PI / 180.0);
            let new_rotation = build_z_rotation_3x3(angle_rad);
            let final_rotation = mat3x3_mul(joint_info.base_rotation, new_rotation);
            let tf4x4 = make_4x4_from_rotation_and_translation(
                final_rotation,
                joint_info.origin_translation,
            );

            // now log the transform
            let (translation, mat3x3) = decompose_4x4_to_translation_and_mat3x3(tf4x4);
            let tf =
                rerun::archetypes::Transform3D::from_translation(translation).with_mat3x3(mat3x3);

            debug_log_rerun_transform(
                &joint_info.entity_path,
                None,
                [0.0, 0.0, angle_rad],
                translation,
                mat3x3,
                "Actuator animation transform",
            );
            rec.log(&*joint_info.entity_path, &tf)?;

            // Optionally log basic actuator states
            log_actuator_states(
                rec,
                frame_idx,
                actuator_id,
                state.position,
                state.velocity,
                state.torque,
            )?;

            frame_had_valid_data = true;
        }

        if frame_had_valid_data {
            frames_processed += 1;
        }
    }

    info!("Successfully logged {} KREC frames to rerun", frames_processed);
    Ok(())
}
