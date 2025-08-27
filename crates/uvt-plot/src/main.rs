use clap::Parser;
use std::path::PathBuf;

use uvt_plot;

#[derive(clap::ValueEnum, Parser, Clone, Default, Debug)]
enum Mode {
    // UVT file
    #[default]
    UVT,
    // Rosbag file
    Rosbag,
    // MCAP file
    MCAP,
}

#[derive(Parser, Debug)]
#[clap(author, version, about)]
struct Args {
    /// Input file path
    #[clap(short, long)]
    input_file: PathBuf,

    /// File mode
    #[clap(short, long, default_value_t, value_enum)]
    mode: Mode,

    /// Map topic
    #[clap(long, default_value = "/map")]
    map_topic: String,

    /// Trajectory topic
    #[clap(long, default_value = "/odom")]
    traj_topic: String,
}

fn main() {
    let args = Args::parse();

    if !args.input_file.exists() {
        eprintln!("File does not exist: {}", args.input_file.display());
        std::process::exit(1);
    }

    let uv_traj = match args.mode {
        Mode::UVT => uvt::Uvt::read_file(args.input_file),
        Mode::Rosbag => uvt::Uvt::read_rosbag(args.input_file, &args.map_topic, &args.traj_topic),
        Mode::MCAP => uvt::Uvt::read_mcap(args.input_file, &args.map_topic, &args.traj_topic),
    }
    .unwrap();

    crate::uvt_plot::plot_trajectory(uv_traj);
}
