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

use debug_log_utils::debug_print_actuator_transform;
use repl_utils::interactive_transform_repl;
use spatial_transform_utils::{
    build_z_rotation_4x4, decompose_4x4_to_translation_and_mat3x3, mat4x4_mul, mat4x4_mul_point,
};
use urdf_bfs_utils::build_joint_name_to_entity_path;
use urdf_logger::{parse_and_log_urdf_hierarchy, BakedUrdfInfo, JointBakedInfo};

// KREC crate: adjust your path/names if needed!
use krec::KRec;

/// A simple map of actuator => URDF joint name.
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
// Approach B: World-space pivot rotation
// -----------------------------------------------------------------------------
fn build_translation(tx: f32, ty: f32, tz: f32) -> [f32; 16] {
    [
        1.0, 0.0, 0.0, tx, 0.0, 1.0, 0.0, ty, 0.0, 0.0, 1.0, tz, 0.0, 0.0, 0.0, 1.0,
    ]
}

fn apply_krec_angle_in_world_space(
    frame_idx: usize,
    actuator_id: u32,
    angle_deg: f64,
    joint_name: &str,
    baked_info: &BakedUrdfInfo,
    rec: &RecordingStream,
) -> Result<()> {
    use log::debug;

    // 1) Look up the BakedJointInfo for this joint
    let joint_info = match baked_info.joint_info.get(joint_name) {
        Some(j) => j,
        None => {
            debug!("No joint info for {joint_name}");
            return Ok(());
        }
    };
    let child_link = &joint_info.child_link;
    let parent_link = &joint_info.parent_link;

    // 2) The child's baked transform in world coords ("rest" transform)
    let child_rest_world_tf = match baked_info.link_to_world.get(child_link) {
        Some(m) => *m,
        None => {
            debug!("No link_to_world for child link {child_link}");
            return Ok(());
        }
    };

    // 3) The parent's baked transform in world coords
    let parent_rest_world_tf = match baked_info.link_to_world.get(parent_link) {
        Some(m) => *m,
        None => {
            debug!("No link_to_world for parent link {parent_link}");
            return Ok(());
        }
    };

    // 4) Pivot is assumed to be (0,0,0) in the parent's local frame.
    //    Transform that to a world-space pivot:
    let pivot_local = [0.0, 0.0, 0.0];
    let pivot_world = mat4x4_mul_point(parent_rest_world_tf, pivot_local);

    // 5) Build a world-space rotation about +Z at pivot_world
    let angle_rad = angle_deg * (std::f64::consts::PI / 180.0);
    let rot_z_local = build_z_rotation_4x4(angle_rad);

    let pivot_neg = build_translation(
        -pivot_world[0] as f32,
        -pivot_world[1] as f32,
        -pivot_world[2] as f32,
    );
    let pivot_pos = build_translation(
        pivot_world[0] as f32,
        pivot_world[1] as f32,
        pivot_world[2] as f32,
    );

    // delta_world = T(+pivot) * R_z(angle) * T(-pivot)
    let delta_world = mat4x4_mul(pivot_pos, mat4x4_mul(rot_z_local, pivot_neg));

    // 6) new_child_world_tf = delta_world * child_rest_world_tf
    //    => the child's "rotated" transform in world coords
    let child_new_world_tf = mat4x4_mul(delta_world, child_rest_world_tf);

    // 7) Decompose both transforms:
    //    - The "child_new_world_tf" to get the new rotation
    //    - The original "child_rest_world_tf" to get the original translation
    let (original_translation, _orig_rest_rot) =
        decompose_4x4_to_translation_and_mat3x3(child_rest_world_tf);

    let (_, new_mat3x3) = decompose_4x4_to_translation_and_mat3x3(child_new_world_tf);

    // Debug prints
    debug_print_actuator_transform(
        frame_idx,
        actuator_id,
        joint_name,
        &joint_info.entity_path,
        angle_deg,
        angle_rad,
        child_new_world_tf,
        original_translation, // We'll show the original translation in the debug logs
        new_mat3x3,
    );

    // 8) Construct the final transform to send to Rerun:
    //    Keep the original baked translation, apply the newly computed rotation.
    let tf = rerun::archetypes::Transform3D::from_translation(original_translation)
        .with_mat3x3(new_mat3x3);

    // 9) Actually log the transform to Rerun
    rec.log(&*joint_info.entity_path, &tf)?;

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
    let args = dbg!(Args::parse());

    // 1) Start a Rerun recording
    let rec = dbg!(rerun::RecordingStreamBuilder::new(
        "rust_krecviz_hierarchy_example"
    ))
    .spawn()?;

    // 2) If we have a URDF, parse & log it hierarchically
    let mut joint_name_to_entity_path = HashMap::new();
    let mut baked_urdf_info = None;

    if let Some(urdf_path) = &args.urdf {
        dbg!(urdf_path);

        // parse_and_log_urdf_hierarchy returns a BakedUrdfInfo
        let info: BakedUrdfInfo = parse_and_log_urdf_hierarchy(urdf_path, &rec)?;

        // Show debug info
        log::debug!("\nBaked URDF Info:");
        log::debug!("Link to world transforms:");
        for (link_name, transform) in &info.link_to_world {
            log::debug!("  {}: [", link_name);
            for row in 0..4 {
                log::debug!(
                    "    {:7.3} {:7.3} {:7.3} {:7.3}",
                    transform[row * 4],
                    transform[row * 4 + 1],
                    transform[row * 4 + 2],
                    transform[row * 4 + 3]
                );
            }
            log::debug!("  ]");
        }

        log::debug!("\nJoint info:");
        for (joint_name, jinfo) in &info.joint_info {
            log::debug!("  {}:", joint_name);
            log::debug!("    entity_path: {}", jinfo.entity_path);
            log::debug!("    parent_link: {}", jinfo.parent_link);
            log::debug!("    child_link: {}", jinfo.child_link);
            log::debug!("    local_tf: [");
            for row in 0..4 {
                log::debug!(
                    "      {:7.3} {:7.3} {:7.3} {:7.3}",
                    jinfo.local_tf[row * 4],
                    jinfo.local_tf[row * 4 + 1],
                    jinfo.local_tf[row * 4 + 2],
                    jinfo.local_tf[row * 4 + 3]
                );
            }
            log::debug!("    ]");
        }

        baked_urdf_info = Some(info);

        // Also build a BFS-based map from each joint’s name => path of *links only*
        joint_name_to_entity_path = build_joint_name_to_entity_path(urdf_path)?;
    } else {
        dbg!("No URDF path provided, logging a fallback message.");
        rec.log("/urdf_info", &rerun::TextDocument::new("No URDF provided"))?;
    }

    // 3) If we have a KREC, load it => animate
    if let Some(krec_path) = &args.krec {
        dbg!(krec_path);
        let loaded_krec = KRec::load(krec_path)
            .map_err(|e| anyhow::anyhow!("Failed to load KREC from {:?}: {:?}", krec_path, e))?;

        let actuator_map = build_actuator_to_urdf_joint_map();

        // We'll need the BakedUrdfInfo to get each joint’s local_tf
        let info = match &baked_urdf_info {
            Some(i) => i,
            None => {
                log::error!("No URDF was loaded, but we have a KREC? We won't animate anything.");
                return Ok(());
            }
        };

        // 4) Iterate frames
        for (frame_idx, frame) in loaded_krec.frames.iter().enumerate() {
            // Tell Rerun which "time" this frame corresponds to
            rec.set_time_sequence("frame_idx", frame_idx as i64);

            // 5) For each ActuatorState => find the URDF joint => apply new angle
            for state in &frame.actuator_states {
                let actuator_id = state.actuator_id;
                if let Some(joint_name) = actuator_map.get(&actuator_id) {
                    // Old approach (commented out):
                    /*
                    if let Some(joint_info) = info.joint_info.get(*joint_name) {
                        if let Some(pos_deg) = state.position {
                            let angle_rad = pos_deg * (PI / 180.0);
                            let rot_tf_4x4 = build_z_rotation_4x4(angle_rad);
                            let new_local_tf = mat4x4_mul(joint_info.local_tf, rot_tf_4x4);

                            let (translation, mat3x3) = decompose_4x4_to_translation_and_mat3x3(new_local_tf);
                            debug_print_actuator_transform(
                                frame_idx,
                                actuator_id,
                                joint_name,
                                &joint_info.entity_path,
                                pos_deg,
                                angle_rad,
                                new_local_tf,
                                translation,
                                mat3x3,
                            );
                            let tf = rerun::archetypes::Transform3D::from_translation(translation)
                                .with_mat3x3(mat3x3);
                            rec.log(&*joint_info.entity_path, &tf)?;
                        }
                    }
                    */

                    // Approach B: pivot in world space:
                    if let Some(pos_deg) = state.position {
                        apply_krec_angle_in_world_space(
                            frame_idx,
                            actuator_id,
                            pos_deg,
                            joint_name,
                            info,
                            &rec,
                        )?;
                    }
                }

                // Optionally, also log basic actuator states
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

    // 6) If we have an output path, let's save to .rrd
    if let Some(rrd_path) = &args.output {
        // Save to .rrd
        rec.save(rrd_path)?;
        log::info!("Saved Rerun data to {}", rrd_path);
    }

    // 7) Interactive REPL if you want to tweak transforms manually
    interactive_transform_repl(&rec)?;
    Ok(())
}
