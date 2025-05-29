use std::{path::PathBuf, str::FromStr};

use plotters::{
    chart::{self, ChartBuilder},
    prelude::{BitMapBackend, Circle, IntoDrawingArea},
    series::{LineSeries, PointSeries},
    style::{RED, WHITE},
};
use uvt;

pub fn plot_trajectory(uvt_file: uvt::Uvt) {
    let figpath: PathBuf = PathBuf::from_str("traj.png").unwrap();

    let positions: Vec<(f64, f64, f64)> = uvt_file
        .trajectory
        .iter()
        .map(|pose| pose.pose.position)
        .map(|pt| (pt.x, pt.y, pt.z))
        .collect();

    let (xs, ys): (Vec<f64>, Vec<f64>) = positions.iter().map(|pt| (pt.0, pt.1)).unzip();
    let zs: Vec<f64> = positions.iter().map(|pt| pt.2).collect();

    let x_max = xs
        .iter()
        .max_by(|&a, &b| a.partial_cmp(b).unwrap())
        .unwrap()
        .clone();
    let x_min = xs
        .iter()
        .min_by(|&a, &b| a.partial_cmp(b).unwrap())
        .unwrap()
        .clone();
    let y_max = ys
        .iter()
        .max_by(|&a, &b| a.partial_cmp(b).unwrap())
        .unwrap()
        .clone();
    let y_min = ys
        .iter()
        .min_by(|&a, &b| a.partial_cmp(b).unwrap())
        .unwrap()
        .clone();
    let z_max = zs
        .iter()
        .max_by(|&a, &b| a.partial_cmp(b).unwrap())
        .unwrap()
        .clone();
    let z_min = zs
        .iter()
        .min_by(|&a, &b| a.partial_cmp(b).unwrap())
        .unwrap()
        .clone();

    let x_span = (x_min, x_max);
    let y_span = (y_min, y_max);
    let z_span = (z_min, z_max);

    let fig = BitMapBackend::new(figpath.as_os_str(), (800, 600)).into_drawing_area();
    fig.fill(&WHITE).unwrap();

    let mut ctx = ChartBuilder::on(&fig)
        .margin(20)
        .set_label_area_size(chart::LabelAreaPosition::Left, 40)
        .set_label_area_size(chart::LabelAreaPosition::Bottom, 40)
        .build_cartesian_2d(
            (x_span.0 - 10.0)..(x_span.1 + 10.0),
            (y_span.0 - 10.0)..(y_span.1 + 10.0),
        )
        .unwrap();
    ctx.configure_mesh().draw().unwrap();

    ctx.draw_series(LineSeries::new(
        (-100..100)
            .map(|y| y as f64 / 100.0)
            .map(|y| ((y * 10.0).sin(), y)),
        &RED,
    ))
    .unwrap();

    ctx.draw_series(
        positions
            .into_iter()
            .map(|(x, y, _)| (x, y))
            .map(|pt| Circle::new(pt, 1, &RED)),
    )
    .unwrap();

    println!("Saved trajectory plot in {}", figpath.as_os_str().display());
}
