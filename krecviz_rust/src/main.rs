use anyhow::Result;
use clap::Parser;
use rerun::RecordingStream; // for rec.set_time_sequence(...)
use std::collections::HashMap;
use std::f64::consts::PI;

mod urdf_logger;

use urdf_logger::{
    parse_and_log_urdf_hierarchy,
    build_joint_name_to_entity_path, // our new BFS-based function
};

// KREC crate: adjust your path/names if needed!
use krec::KRec;
use krec::KRecFrame;
use urdf_rs::Joint;

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

// -----------------------------------------------------------------------------
// Minimal 4x4 row-major transform builder that just rotates around Z
// -----------------------------------------------------------------------------
fn build_z_rotation_4x4(angle_rad: f64) -> [f32; 16] {
    let cz = angle_rad.cos() as f32;
    let sz = angle_rad.sin() as f32;

    [
        cz,  -sz, 0.0, 0.0,
        sz,   cz, 0.0, 0.0,
        0.0,  0.0, 1.0, 0.0,
        0.0,  0.0, 0.0, 1.0,
    ]
}

/// Convert a 4x4 into (translation, 3x3) for logging as a `Transform3D`.
fn decompose_4x4_to_translation_and_mat3x3(tf: [f32; 16]) -> ([f32; 3], [f32; 9]) {
    let translation = [tf[3], tf[7], tf[11]];
    let mat3x3 = [
        tf[0], tf[1], tf[2],
        tf[4], tf[5], tf[6],
        tf[8], tf[9], tf[10],
    ];
    (translation, mat3x3)
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
    // Show the parsed CLI args
    let args = dbg!(Args::parse());

    // 1) Start a Rerun recording
    let rec = dbg!(rerun::RecordingStreamBuilder::new("rust_krecviz_hierarchy_example"))
        .spawn()?;

    // 2) If we have a URDF, parse & log it hierarchically
    let mut joint_name_to_entity_path = HashMap::new();
    if let Some(urdf_path) = &args.urdf {
        dbg!(urdf_path);
        // Log geometry + BFS transforms (stage1/stage2) from urdf_logger
        parse_and_log_urdf_hierarchy(urdf_path, &rec)?;

        // Also build the BFS-based map from each joint's name => "link-only" path
        // (this is used later to animate transforms from the KREC data)
        joint_name_to_entity_path = build_joint_name_to_entity_path(urdf_path)?;
    } else {
        dbg!("No URDF path provided, logging a fallback message.");
        rec.log("/urdf_info", &rerun::TextDocument::new("No URDF provided"))?;
    }

    // 3) If we have a KREC, load it
    if let Some(krec_path) = &args.krec {
        dbg!(krec_path);
        let loaded_krec = KRec::load(krec_path)
            .map_err(|e| anyhow::anyhow!("Failed to load KREC from {:?}: {:?}", krec_path, e))?;

        // We'll replicate the python actuator->joint map
        let actuator_map = build_actuator_to_urdf_joint_map();

        // 4) Iterate frames
        for (frame_idx, frame) in loaded_krec.frames.iter().enumerate() {
            // Set Rerun time-sequence so transforms appear “animated”
            rec.set_time_sequence("frame_idx", frame_idx as i64);

            // 5) For each ActuatorState, look up the joint + log transform
            for state in &frame.actuator_states {
                let actuator_id = state.actuator_id;
                if let Some(joint_name) = actuator_map.get(&actuator_id) {
                    if let Some(entity_path) = joint_name_to_entity_path.get(*joint_name) {
                        if let Some(pos_deg) = state.position {
                            let angle_rad = pos_deg * (PI / 180.0);
                            let tf4x4 = build_z_rotation_4x4(angle_rad);
                            let (translation, mat3x3) = decompose_4x4_to_translation_and_mat3x3(tf4x4);

                            // Use helper function to print debug info
                            debug_print_actuator_transform(
                                frame_idx,
                                actuator_id,
                                joint_name,
                                entity_path,
                                pos_deg,
                                angle_rad,
                                tf4x4,
                                translation,
                                mat3x3,
                            );

                            // Log the transform
                            let tf = rerun::archetypes::Transform3D::from_translation(translation)
                                .with_mat3x3(mat3x3);

                            rec.log(entity_path.clone(), &tf)?;
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

    // Sleep so we can see the result in the viewer
    std::thread::sleep(std::time::Duration::from_secs(5));

    Ok(())
}

// -----------------------------------------------------------------------------
// Debug-print Helper Functions
// -----------------------------------------------------------------------------

fn debug_print_actuator_transform(
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
    println!(
        "[frame={}] actuator_id={} => joint='{}' => entity_path='{}'",
        frame_idx, actuator_id, joint_name, entity_path
    );
    println!(
        "  angle_deg={} => angle_rad={:.3} => transform_4x4={:?}",
        pos_deg, angle_rad, tf4x4
    );
    println!(
        "  => translation={:?}, mat3x3={:?}",
        translation, mat3x3
    );
}
