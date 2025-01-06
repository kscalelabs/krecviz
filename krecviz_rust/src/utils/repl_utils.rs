use crate::utils::debug_log_utils::debug_log_rerun_transform;
use crate::utils::spatial_transform_utils::{
    build_4x4_from_xyz_rpy, decompose_4x4_to_translation_and_mat3x3,
};
use anyhow::Result;
use rerun::RecordingStream;
use std::io::{self, BufRead, Write};

/// Interactive REPL for applying transforms to entities in the Rerun visualization.
///
/// Note that these are absolute transforms relative to the parent.
/// Meaning that, if you do a 90 0 0 transform, and then a -90 0 0,
/// it wont go back to 0 0 0, it will be at -90 0 0
///
/// Prompts the user for:
/// - Entity path to transform
/// - Translation (tx, ty, tz) in meters
/// - Euler angles (roll, pitch, yaw) in degrees
///
/// Applies the specified transform to the entity at the given path and logs it to Rerun.
/// Continues prompting until an empty path is entered.
///
/// # Arguments
/// * `rec` - The Rerun RecordingStream to log transforms to
///
/// # Returns
/// * `anyhow::Result<()>` - Ok if all transforms were applied successfully
pub fn interactive_transform_repl(rec: &RecordingStream) -> Result<()> {
    let stdin = io::stdin();
    let mut lines = stdin.lock().lines();

    println!("\n--- Entering interactive transform REPL ---");
    println!("Type an empty path to exit.\n");

    loop {
        print!("Apply transform to path (or empty to quit): ");
        io::stdout().flush()?;
        let Some(Ok(path)) = lines.next() else { break };

        let path = path.trim();
        if path.is_empty() {
            println!("Exiting REPL...");
            break;
        }

        // Ask for translation
        println!("Enter translation as 'tx ty tz' (in meters) or press Enter to skip:");
        print!("> ");
        io::stdout().flush()?;
        let Some(Ok(translation_line)) = lines.next() else {
            break;
        };
        let translation_line = translation_line.trim();
        // default to 0,0,0 if empty
        let (tx, ty, tz) = if translation_line.is_empty() {
            (0.0, 0.0, 0.0)
        } else {
            let coords: Vec<_> = translation_line.split_whitespace().collect();
            if coords.len() != 3 {
                println!("Invalid input! Setting translation to (0,0,0).");
                (0.0, 0.0, 0.0)
            } else {
                let tx = coords[0].parse::<f64>().unwrap_or(0.0);
                let ty = coords[1].parse::<f64>().unwrap_or(0.0);
                let tz = coords[2].parse::<f64>().unwrap_or(0.0);
                (tx, ty, tz)
            }
        };

        // Ask for Euler angles (in degrees)
        println!("Enter euler angles as 'roll pitch yaw' in degrees, or press Enter for (0,0,0):");
        print!("> ");
        io::stdout().flush()?;
        let Some(Ok(rpy_line)) = lines.next() else {
            break;
        };
        let rpy_line = rpy_line.trim();
        let (roll_deg, pitch_deg, yaw_deg) = if rpy_line.is_empty() {
            (0.0, 0.0, 0.0)
        } else {
            let angles: Vec<_> = rpy_line.split_whitespace().collect();
            if angles.len() != 3 {
                println!("Invalid input! Setting euler angles to (0,0,0).");
                (0.0, 0.0, 0.0)
            } else {
                let rr = angles[0].parse::<f64>().unwrap_or(0.0);
                let pp = angles[1].parse::<f64>().unwrap_or(0.0);
                let yy = angles[2].parse::<f64>().unwrap_or(0.0);
                (rr, pp, yy)
            }
        };

        // Convert degrees => radians
        let (roll, pitch, yaw) = (
            roll_deg.to_radians(),
            pitch_deg.to_radians(),
            yaw_deg.to_radians(),
        );

        // Build a 4x4
        let transform_4x4 = build_4x4_from_xyz_rpy([tx, ty, tz], [roll, pitch, yaw]);
        let (translation_f32, mat3x3_f32) = decompose_4x4_to_translation_and_mat3x3(transform_4x4);

        // Debug print before logging
        debug_log_rerun_transform(
            path,
            None, // No BFS data in REPL
            [roll_deg, pitch_deg, yaw_deg],
            translation_f32,
            mat3x3_f32,
            "REPL interactive transform",
        );

        // Log it to Rerun
        let transform_3d = rerun::archetypes::Transform3D::from_translation(translation_f32)
            .with_mat3x3(mat3x3_f32);
        rec.log(path, &transform_3d)?;

        println!(
            "Logged transform to '{}': translation=({:.3},{:.3},{:.3}), rpy_deg=({:.1},{:.1},{:.1})",
            path, tx, ty, tz, roll_deg, pitch_deg, yaw_deg
        );
    }

    Ok(())
}
