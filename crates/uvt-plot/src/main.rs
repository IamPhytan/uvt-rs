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
}

#[derive(Parser, Debug)]
#[clap(author, version, about)]
struct Args {
    #[clap(long)]
    input_file: PathBuf,
    #[clap(short, default_value_t, value_enum)]
    mode: Mode,
}

fn main() {
    let args = Args::parse();

    if !args.input_file.exists() {
        eprintln!("File does not exist: {}", args.input_file.display());
        std::process::exit(1);
    }

    let uv_traj = match args.mode {
        Mode::UVT => uvt::Uvt::read_file(args.input_file),
        Mode::Rosbag => uvt::Uvt::read_rosbag(args.input_file, "/map", "/traj"),
    }
    .unwrap();

    crate::uvt_plot::plot_trajectory(uv_traj);
}
