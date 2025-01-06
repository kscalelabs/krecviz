use anyhow::Result;
use env_logger;

// Import the necessary functions from your crate
use krecviz::parse_and_log_urdf_hierarchy;

const BASE_PATH: &str = env!("CARGO_MANIFEST_DIR");
const URDF_BASE_PATH: &str = "tests/assets/urdf_examples";
// const KREC_BASE_PATH: &str = "tests/assets/krec_examples";

// fn get_krec_path(key: &str) -> String {
//     match key {
//         "gpr" => format!(
//             "{BASE_PATH}/{KREC_BASE_PATH}/actuator_22_right_arm_shoulder_roll_movement.krec"
//         ),
//         _ => panic!("Unknown KREC key: {}", key),
//     }
// }

fn get_urdf_path(key: &str) -> String {
    match key {
        "gpr" => format!("{BASE_PATH}/{URDF_BASE_PATH}/gpr/robot.urdf"),
        "simple" => format!("{BASE_PATH}/{URDF_BASE_PATH}/simple/example.urdf"),
        "manual_urdf" => format!("{BASE_PATH}/{URDF_BASE_PATH}/manual_urdf/manual_example.urdf"),
        "simple_onshape_2_joints" => {
            format!("{BASE_PATH}/{URDF_BASE_PATH}/simple_onshape_2_joints/assembly_1.urdf")
        }
        "simple_onshape_2_joints_asymmetrical" => format!(
            "{BASE_PATH}/{URDF_BASE_PATH}/simple_onshape_2_joints_asymmetrical/assembly_1.urdf"
        ),
        "simple_onshape_4_joints" => {
            format!("{BASE_PATH}/{URDF_BASE_PATH}/simple_onshape_4_joints/assembly_1.urdf")
        }
        "xbot" => format!("{BASE_PATH}/{URDF_BASE_PATH}/XBot/urdf/XBot-L.urdf"),
        _ => panic!("Unknown URDF key: {}", key),
    }
}

#[test]
fn test_run_visualization_gpr_adhoc() -> Result<()> {
    let _ = env_logger::try_init();
    let rec = rerun::RecordingStreamBuilder::new("test_visualization_adhoc").spawn()?;

    parse_and_log_urdf_hierarchy(&get_urdf_path("gpr"), &rec)?;
    Ok(())
}

// #[test]
// fn test_run_visualization_simple_adhoc() -> Result<()> {
//     let rec = rerun::RecordingStreamBuilder::new("test_visualization_adhoc").spawn()?;
//     parse_and_log_urdf_hierarchy(&get_urdf_path("simple"), &rec)?;
//     Ok(())
// }

// #[test]
// fn test_run_manual_urdf_adhoc() -> Result<()> {
//     let _ = env_logger::try_init();
//     let rec = rerun::RecordingStreamBuilder::new("test_visualization_adhoc").spawn()?;
//     parse_and_log_urdf_hierarchy(&get_urdf_path("manual_urdf"), &rec)?;
//     Ok(())
// }

// #[test]
// fn test_run_visualization_simple_onshape_2_joints_adhoc() -> Result<()> {
//     let rec = rerun::RecordingStreamBuilder::new("test_visualization_adhoc").spawn()?;
//     parse_and_log_urdf_hierarchy(&get_urdf_path("simple_onshape_2_joints"), &rec)?;
//     Ok(())
// }

// #[test]
// fn test_run_visualization_simple_onshape_2_joints_asymmetrical_adhoc() -> Result<()> {
//     let _ = env_logger::try_init();
//     let rec = rerun::RecordingStreamBuilder::new("test_visualization_adhoc").spawn()?;
//     parse_and_log_urdf_hierarchy(&get_urdf_path("simple_onshape_2_joints_asymmetrical"), &rec)?;
//     Ok(())
// }

// #[test]
// fn test_run_visualization_simple_onshape_4_joints_adhoc() -> Result<()> {
//     let rec = rerun::RecordingStreamBuilder::new("test_visualization_adhoc").spawn()?;
//     parse_and_log_urdf_hierarchy(&get_urdf_path("simple_onshape_4_joints"), &rec)?;
//     Ok(())
// }

// #[test]
// fn test_run_visualization_xbot_adhoc() -> Result<()> {
//     let rec = rerun::RecordingStreamBuilder::new("test_visualization_adhoc").spawn()?;
//     parse_and_log_urdf_hierarchy(&get_urdf_path("xbot"), &rec)?;
//     Ok(())
// }
