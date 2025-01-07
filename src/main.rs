// src/main.rs

use anyhow::Result;
use clap::Parser;
use env_logger::{Builder, Env};

use krecviz::viz;

#[derive(Parser, Debug)]
#[command(name = "krecviz")]
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
    // Initialize logger
    Builder::from_env(Env::default().default_filter_or("krecviz=info")).init();

    // Parse CLI args
    let args = Args::parse();

    // Call viz from the library
    viz(
        args.urdf.as_deref(),
        args.krec.as_deref(),
        args.output.as_deref(),
    )
}
