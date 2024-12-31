use anyhow::Result;
use env_logger;
use log::{debug, info};
use std::collections::HashMap;
use std::path::Path;

// Import the necessary functions from your crate
use krecviz_rust::parse_and_log_urdf_hierarchy;

const BASE_PATH: &str = env!("CARGO_MANIFEST_DIR");
const URDF_BASE_PATH: &str = "../tests/assets/urdf_examples";
const KREC_BASE_PATH: &str = "../tests/assets/krec_examples";

lazy_static::lazy_static! {
    static ref EXAMPLE_KREC_PATHS: HashMap<&'static str, String> = build_krec_paths();
    static ref EXAMPLE_URDF_PATHS: HashMap<&'static str, String> = build_urdf_paths();
}

fn build_krec_paths() -> HashMap<&'static str, String> {
    let mut paths = HashMap::new();

    paths.insert(
        "gpr",
        format!("{BASE_PATH}/{KREC_BASE_PATH}/actuator_22_right_arm_shoulder_roll_movement.krec"),
    );

    paths
}

fn build_urdf_paths() -> HashMap<&'static str, String> {
    let mut paths = HashMap::new();

    // Define the URDF file paths
    let urdf_files = [
        ("gpr", "gpr/robot.urdf"),
        ("simple", "simple/example.urdf"),
        ("manual_urdf", "manual_urdf/manual_example.urdf"),
        (
            "simple_onshape_2_joints",
            "simple_onshape_2_joints/assembly_1.urdf",
        ),
        (
            "simple_onshape_2_joints_asymmetrical",
            "simple_onshape_2_joints_asymmetrical/assembly_1.urdf",
        ),
        (
            "simple_onshape_4_joints",
            "simple_onshape_4_joints/assembly_1.urdf",
        ),
        ("xbot", "XBot/urdf/XBot-L.urdf"),
    ];

    // Insert all paths with consistent formatting
    for (key, path) in urdf_files {
        paths.insert(key, format!("{BASE_PATH}/{URDF_BASE_PATH}/{path}"));
    }

    paths
}

// #[test]
// fn test_run_visualization_gpr_adhoc() -> Result<()> {
//     let _ = env_logger::try_init();
//     let rec = rerun::RecordingStreamBuilder::new("test_visualization_adhoc").spawn()?;

//     parse_and_log_urdf_hierarchy(&EXAMPLE_URDF_PATHS["gpr"], &rec)?;
//     Ok(())
// }

// #[test]
// fn test_run_visualization_simple_adhoc() -> Result<()> {
//     let rec = rerun::RecordingStreamBuilder::new("test_visualization_adhoc").spawn()?;
//     parse_and_log_urdf_hierarchy(&EXAMPLE_URDF_PATHS["simple"], &rec)?;
//     Ok(())
// }

#[test]
fn test_run_manual_urdf_adhoc() -> Result<()> {
    let _ = env_logger::try_init();
    let rec = rerun::RecordingStreamBuilder::new("test_visualization_adhoc").spawn()?;
    parse_and_log_urdf_hierarchy(&EXAMPLE_URDF_PATHS["manual_urdf"], &rec)?;
    Ok(())
}

// #[test]
// fn test_run_visualization_simple_onshape_2_joints_adhoc() -> Result<()> {
//     let rec = rerun::RecordingStreamBuilder::new("test_visualization_adhoc").spawn()?;
//     parse_and_log_urdf_hierarchy(&EXAMPLE_URDF_PATHS["simple_onshape_2_joints"], &rec)?;
//     Ok(())
// }

// #[test]
// fn test_run_visualization_simple_onshape_2_joints_asymmetrical_adhoc() -> Result<()> {
//     let _ = env_logger::try_init();
//     let rec = rerun::RecordingStreamBuilder::new("test_visualization_adhoc").spawn()?;

//     parse_and_log_urdf_hierarchy(
//         &EXAMPLE_URDF_PATHS["simple_onshape_2_joints_asymmetrical"],
//         &rec,
//     )?;
//     Ok(())
// }

// #[test]
// fn test_run_visualization_simple_onshape_4_joints_adhoc() -> Result<()> {
//     let rec = rerun::RecordingStreamBuilder::new("test_visualization_adhoc").spawn()?;
//     parse_and_log_urdf_hierarchy(&EXAMPLE_URDF_PATHS["simple_onshape_4_joints"], &rec)?;
//     Ok(())
// }

// #[test]
// fn test_run_visualization_xbot_adhoc() -> Result<()> {
//     let rec = rerun::RecordingStreamBuilder::new("test_visualization_adhoc").spawn()?;
//     parse_and_log_urdf_hierarchy(&EXAMPLE_URDF_PATHS["xbot"], &rec)?;
//     Ok(())
// }
