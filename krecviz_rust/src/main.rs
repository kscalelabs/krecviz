use anyhow::Result;
use clap::Parser;
use env_logger::{Builder, Env};
use rerun::RecordingStream; // for rec.set_time_sequence(...)
use std::collections::HashMap;
use std::f64::consts::PI;

mod debug_log_utils;
mod geometry_utils;
mod repl_utils;
mod spatial_transform_utils;
mod urdf_bfs_utils;
mod urdf_logger;

use debug_log_utils::debug_log_rerun_transform;
use debug_log_utils::debug_log_actuator_state;
use repl_utils::interactive_transform_repl;
use spatial_transform_utils::{
    build_z_rotation_3x3,
    make_4x4_from_rotation_and_translation,
    decompose_4x4_to_translation_and_mat3x3,
    mat3x3_mul
};
use urdf_bfs_utils::build_joint_name_to_joint_info;
use urdf_logger::parse_and_log_urdf_hierarchy;

// KREC crate: adjust your path/names if needed!
use krec::KRec;

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

// -----------------------------------------------------------------------------
// CLI
// -----------------------------------------------------------------------------
#[derive(Parser, Debug)]
#[command(name = "rust_krecviz")]
struct Args {
    /// Path to the URDF file
    #[arg(long)]
    urdf: Option<String>,

    /// Path to the KREC file
    #[arg(long)]
    krec: Option<String>,

    /// Path to .rrd output (if you want to save)
    #[arg(long)]
    output: Option<String>,
}

fn main() -> Result<()> {
    // Initialize logger with custom filter
    Builder::from_env(Env::default().default_filter_or("krecviz_rust=debug")).init();

    // Show the parsed CLI args
    let args = Args::parse();

    // 1) Start a Rerun recording
    let rec = rerun::RecordingStreamBuilder::new("rust_krecviz_hierarchy_example").spawn()?;

    // 2) If we have a URDF, parse & log it hierarchically
    if let Some(urdf_path) = &args.urdf {
        dbg!(urdf_path);
        parse_and_log_urdf_hierarchy(urdf_path, &rec)?;
    } else {
        dbg!("No URDF path provided, logging a fallback message.");
        rec.log("/no_urdf_found", &rerun::TextDocument::new("No URDF provided"))?;
    }

    // 3) If we have a KREC, load it
    if let Some(krec_path) = &args.krec {
        dbg!(krec_path);
        let loaded_krec = KRec::load(krec_path)
            .map_err(|e| anyhow::anyhow!("Failed to load KREC from {:?}: {:?}", krec_path, e))?;

        // We'll replicate the python actuator->joint map
        let actuator_map = build_actuator_to_urdf_joint_map();

        let joint_info_map = if let Some(urdf_path) = &args.urdf {
            build_joint_name_to_joint_info(urdf_path)?
        } else {
            HashMap::new()
        };

        // 4) Iterate frames
        for (frame_idx, frame) in loaded_krec.frames.iter().enumerate() {
            // Set Rerun time-sequence so transforms appear “animated”
            rec.set_time_sequence("frame_idx", frame_idx as i64);

            // 5) For each ActuatorState, look up the joint + log transform
            for state in &frame.actuator_states {
                let actuator_id = state.actuator_id;
                if let Some(joint_name) = actuator_map.get(&actuator_id) {
                    if let Some(joint_info) = joint_info_map.get(*joint_name) {
                        if let Some(pos_deg) = state.position {
                            let angle_rad = pos_deg * (PI / 180.0);
                            
                            // First build the new rotation (3x3)
                            let new_rotation = build_z_rotation_3x3(angle_rad);
                            
                            // Multiply with base rotation (3x3)
                            let final_rotation = mat3x3_mul(joint_info.base_rotation, new_rotation);
                            
                            // Make 4x4 matrix with rotation and translation
                            let tf4x4 = make_4x4_from_rotation_and_translation(
                                final_rotation,
                                joint_info.origin_translation
                            );
                            
                            // Now decompose (this will handle the row->column major conversion)
                            let (translation, mat3x3) = decompose_4x4_to_translation_and_mat3x3(tf4x4);

                            // Log the transform
                            let tf = rerun::archetypes::Transform3D::from_translation(translation)
                                .with_mat3x3(mat3x3);

                            debug_log_rerun_transform(
                                &joint_info.entity_path,
                                None,  // No BFS data for actuator transforms
                                [0.0, 0.0, angle_rad],  // Only rotation around Z
                                translation,
                                mat3x3,
                                "Actuator animation transform"
                            );
                            rec.log(&*joint_info.entity_path, &tf)?;
                        }
                    }
                }

                // Optionally, log basic actuator states
                log_actuator_states(
                    &rec,
                    frame_idx,
                    actuator_id,
                    state.position,
                    state.velocity,
                    state.torque,
                )?;
            }
        }
    } else {
        dbg!("No KREC path provided, so no frame-by-frame animation is logged.");
    }

    // REPL to apply transforms to the URDF
    // For debugging
    interactive_transform_repl(&rec)?;
    Ok(())
}
