use clap::Parser;
use std::path::PathBuf;

use uvt_viz3d;

#[derive(Parser, Debug)]
#[command(author, version, about)]
struct Args {
    #[arg(long)]
    input_file: PathBuf,
}

fn main() {
    let args = Args::parse();

    if !args.input_file.exists() {
        eprintln!("File does not exist: {}", args.input_file.display());
        std::process::exit(1);
    }

    let uv_traj = uvt::Uvt::read_file(args.input_file).unwrap();

    crate::uvt_viz3d::show_uvt(uv_traj);
}
