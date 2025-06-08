use rerun::external::glam;
use vtkio::model::{DataSet, Piece};

use uvt;
use vtkio::IOBuffer;

pub fn show_uvt(uvt_file: uvt::Uvt) {
    let map = uvt_file.map;
    let point_cloud = map.data.clone();

    let pieces = match point_cloud {
        DataSet::PolyData { pieces, .. } => pieces,
        _ => {
            panic!("Wrong vtk data type");
        }
    };

    println!("N {} pieces", pieces.len());

    let points: Vec<uvt::Point> = pieces
        .iter()
        .map(|inline_piece| match inline_piece {
            Piece::Inline(piece) => {
                let piece_buf_points = piece.points.clone();
                let buf_values = match piece_buf_points {
                    IOBuffer::F64(buf_points) => buf_points,
                    IOBuffer::F32(buf_points) => buf_points.iter().map(|&v| v as f64).collect(),
                    _ => panic!("Unimplemented IO Buffer"),
                };

                let n_points = piece.num_points();
                let piece_points: Vec<uvt::Point> = (0..n_points)
                    .into_iter()
                    .map(|i| (3 * i + 0, 3 * i + 1, 3 * i + 2))
                    .map(|(idx_x, idx_y, idx_z)| {
                        uvt::Point::new(buf_values[idx_x], buf_values[idx_y], buf_values[idx_z])
                    })
                    .collect();
                piece_points
            }
            _ => panic!("Unknown piece type"),
        })
        .flatten()
        .collect();

    println!("N points {}", points.len());

    // Limits of Z
    let zs: Vec<f64> = points.iter().map(|&pt| pt.z).collect();
    let z_min = zs
        .iter()
        .min_by(|&a, &b| a.partial_cmp(b).unwrap())
        .unwrap()
        .clone();
    let z_max = zs
        .iter()
        .max_by(|&a, &b| a.partial_cmp(b).unwrap())
        .unwrap()
        .clone();

    // Colors
    let colors: Vec<[u8; 4]> = points
        .iter()
        .map(|pt| colormap_turbo_srgb(((pt.z - z_min) / (z_max - z_min)) as f32))
        .collect();

    // Init rerun
    rerun::external::re_log::setup_logging();

    let rec = rerun::RecordingStreamBuilder::new("point-cloud-viewer")
        .spawn()
        .unwrap();
    rec.set_duration_secs("stable-time", 0f64);

    // Log map
    rec.log_static(
        "/map",
        &rerun::Points3D::new(
            points
                .iter()
                .map(|&pt| {
                    let coords: [f32; 3] = pt.into();
                    let vec_pt: glam::Vec3 = coords.into();
                    vec_pt
                })
                .into_iter(),
        )
        .with_colors(colors)
        .with_radii([0.08]),
    )
    .unwrap();

    // Log trajectory
    let n_points = uvt_file.trajectory.len();
    let red: Vec<[u8; 4]> = (0..n_points)
        .into_iter()
        .map(|_| [255, 255, 255, 255])
        .collect();

    rec.log_static(
        "/trajectory",
        &rerun::Points3D::new(
            uvt_file
                .trajectory
                .iter()
                .map(|pt| {
                    let coords: [f32; 3] = pt.pose.position.into();
                    let vec_pt: glam::Vec3 = coords.into();
                    vec_pt
                })
                .into_iter(),
        )
        .with_colors(red)
        .with_radii([0.25]),
    )
    .unwrap();
}

// Returns sRGB polynomial approximation from Turbo color map, assuming `t` is normalized. Copied from rerun DNA demo.
fn colormap_turbo_srgb(t: f32) -> [u8; 4] {
    #![allow(clippy::excessive_precision)]
    use glam::{Vec2, Vec4, Vec4Swizzles as _};

    const R4: Vec4 = Vec4::new(0.13572138, 4.61539260, -42.66032258, 132.13108234);
    const G4: Vec4 = Vec4::new(0.09140261, 2.19418839, 4.84296658, -14.18503333);
    const B4: Vec4 = Vec4::new(0.10667330, 12.64194608, -60.58204836, 110.36276771);

    const R2: Vec2 = Vec2::new(-152.94239396, 59.28637943);
    const G2: Vec2 = Vec2::new(4.27729857, 2.82956604);
    const B2: Vec2 = Vec2::new(-89.90310912, 27.34824973);

    debug_assert!((0.0..=1.0).contains(&t));

    let v4 = glam::vec4(1.0, t, t * t, t * t * t);
    let v2 = v4.zw() * v4.z;

    [
        ((v4.dot(R4) + v2.dot(R2)) * 255.0) as u8,
        ((v4.dot(G4) + v2.dot(G2)) * 255.0) as u8,
        ((v4.dot(B4) + v2.dot(B2)) * 255.0) as u8,
        255,
    ]
}
