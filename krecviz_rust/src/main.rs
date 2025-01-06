use anyhow::Result;
use clap::Parser;
use env_logger::{Builder, Env};
use krec::KRec;
use log::{info, warn};

use crate::krec_logger::parse_and_log_krec;
use crate::urdf_logger::parse_and_log_urdf_hierarchy;
use crate::utils::repl_utils::interactive_transform_repl;

mod krec_logger;
mod urdf_logger;
mod utils;

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

    /// Enable interactive REPL mode
    #[arg(long)]
    repl: bool,
}

fn main() -> Result<()> {
    // Initialize logger with custom filter
    Builder::from_env(Env::default().default_filter_or("krecviz_rust=info")).init();

    // Show the parsed CLI args
    let args = Args::parse();

    // 1) Start a Rerun recording
    let rec = rerun::RecordingStreamBuilder::new("rust_krecviz_hierarchy_example").spawn()?;

    // 2) If we have a URDF, parse & log it hierarchically
    if let Some(urdf_path) = &args.urdf {
        info!("Loading URDF from {}", urdf_path);
        parse_and_log_urdf_hierarchy(urdf_path, &rec)?;
    } else {
        warn!("No URDF path provided!");
        rec.log(
            "/no_urdf_found",
            &rerun::TextDocument::new("No URDF provided"),
        )?;
    }

    // 3) If we have a KREC, load and parse it
    if let Some(krec_path) = &args.krec {
        info!("Loading KREC from {}", krec_path);
        let loaded_krec = KRec::load(krec_path)
            .map_err(|e| anyhow::anyhow!("Failed to load KREC from {:?}: {:?}", krec_path, e))?;
        info!("Loaded KREC with {} frames", loaded_krec.frames.len());
        parse_and_log_krec(&loaded_krec, args.urdf.as_deref(), &rec)?;
    } else {
        warn!("No KREC path provided, no telemetry will be logged!");
    }

    // REPL to apply transforms to the URDF
    // For debugging
    if args.repl {
        interactive_transform_repl(&rec)?;
    }
    Ok(())
}
