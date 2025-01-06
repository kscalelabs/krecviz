// src/lib.rs

use anyhow::Result;
use krec::KRec;
use log::{info, warn};
use rerun::RecordingStreamBuilder;

// Re-export other functions/types if you want them public
pub use crate::krec_logger::parse_and_log_krec;
pub use crate::urdf_logger::parse_and_log_urdf_hierarchy;

mod krec_logger;
mod urdf_logger;
pub mod utils;

pub fn viz(
    urdf_path: Option<&str>,
    krec_path: Option<&str>,
    output_path: Option<&str>,
) -> Result<()> {
    // 1) Start a Rerun recording
    let builder = RecordingStreamBuilder::new("rust_krecviz_hierarchy_example");
    let rec = if let Some(path) = output_path {
        info!("Creating recording that will be saved to {}", path);
        builder.save(path)?
    } else {
        builder.spawn()?
    };

    // 2) If we have a URDF, parse & log it
    if let Some(path) = urdf_path {
        info!("Loading URDF from {}", path);
        parse_and_log_urdf_hierarchy(path, &rec)?;
    } else {
        warn!("No URDF path provided!");
        rec.log(
            "/no_urdf_found",
            &rerun::TextDocument::new("No URDF provided"),
        )?;
    }

    // 3) If we have a KREC, parse it
    if let Some(path) = krec_path {
        info!("Loading KREC from {}", path);
        let loaded_krec = KRec::load(path)
            .map_err(|e| anyhow::anyhow!("Failed to load KREC from {:?}: {:?}", path, e))?;
        info!("Loaded KREC with {} frames", loaded_krec.frames.len());
        parse_and_log_krec(&loaded_krec, urdf_path, &rec)?;
    } else {
        warn!("No KREC path provided, no telemetry will be logged!");
    }

    if output_path.is_some() {
        info!("Successfully saved recording");
    }

    Ok(())
}
