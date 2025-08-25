# uvt

This crate provides utilities for reading and writing files in the _Uncrewed Vehicle Trajectory_ (UVT) format.
The UVT format is an extension of the LTR file format introduced in [_Kilometer-Scale Autonomous Navigation in Subarctic Forests: Challenges and Lessons Learned_](https://doi.org/10.55417/fr.2022050).

A UVT file contains:

- a LiDAR map of the environment, stored in the [`VTK` format](https://vtk.org).
- A trajectory recorded by an uncrewed vehicle.

An example UVT file is avilable on [Zenodo](https://doi.org/10.5281/zenodo.16928365).

## Features

This crate:

- Parses `.uvt` files into Rust data structures.
- Exports trajectories and maps back into  `.uvt` files
- Generate UVT files directly from rosbags:
  - ROS (1) (`.bag`)
  - ROS 2 (`.mcap`)

---

## Examples

This crate allows to read, parse, and write uncrewed vehicle trajectories saved in the `UVT` format.

```rs
use std::io;
use uvt::{self, pose};

fn main() -> Result<(), io::Error> {
    // Open a UVT file
    let my_uvt = uvt::Uvt::read_file("example.uvt")?;

    // Retrieve map and trajectory
    let uvt_map = my_uvt.map.clone();
    let uvt_trajectory = my_uvt.trajectory.clone();

    // Retrieve all positions from the trajectory
    let positions: Vec<pose::Point> = uvt_trajectory
        .iter()
        .map(|pose| pose.pose.position)
        .collect();

    // Retrieve all orientations from the trajectory
    let orientations: Vec<pose::Quaternion> = uvt_trajectory
        .iter()
        .map(|pose| pose.pose.orientation)
        .collect();

    // Save my_uvt to a new file
    my_uvt.write_file("my_uvt.uvt")?;

    Ok(())
}
```

You can also generate a UVT file from rosbags, be it in ROS (1) (`.bag`) or in ROS 2 (`.mcap`).
Your rosbag must contain:

- a **map** topic with `sensor_msgs/PointCloud2` messages
- a **trajectory** topic with `nav_msgs/Odometry` messages.

```rust
use std::io;
use uvt;

fn main() -> Result<(), io::Error> {
    let map_topic = "/map";
    let traj_topic = "/odom";

    // Create a UVT file from a ROS 1 bag file
    let bag_uvt = uvt::Uvt::read_rosbag("example.bag", map_topic, traj_topic)?;
    bag_uvt.write_file("bag.uvt")?;

    // Create a UVT file from a ROS 2 MCAP file
    let mcap_uvt = uvt::Uvt::read_mcap("example.mcap", map_topic, traj_topic)?;
    mcap_uvt.write_file("mcap.uvt")?;

    Ok(())
}
```

## Citation

If you use the code or data in an academic context, please cite the following work:

```bibtex
@article{Baril2022,
  title = {Kilometer-Scale Autonomous Navigation in Subarctic Forests: Challenges and Lessons Learned},
  volume = {2},
  ISSN = {2771-3989},
  url = {http://dx.doi.org/10.55417/fr.2022050},
  DOI = {10.55417/fr.2022050},
  journal = {Field Robotics},
  publisher = {Institute of Electrical and Electronics Engineers (IEEE)},
  author = {Baril,  Dominic and Desch\^enes,  Simon-Pierre and Gamache,  Olivier and Vaidis,  Maxime and LaRocque,  Damien and Laconte,  Johann and Kubelka,  Vladimír and Giguère,  Philippe and Pomerleau,  Fran\c{c}ois},
  year = {2022},
  month = jul,
  pages = {1628–1660}
}
```

## License

Licensed under either of

 * Apache License, Version 2.0
   ([LICENSE-APACHE](LICENSE-APACHE) or <http://www.apache.org/licenses/LICENSE-2.0>)
 * MIT license
   ([LICENSE-MIT](LICENSE-MIT) or <http://opensource.org/licenses/MIT>)

at your option.

## Contribution

Unless you explicitly state otherwise, any contribution intentionally submitted
for inclusion in the work by you, as defined in the Apache-2.0 license, shall be
dual licensed as above, without any additional terms or conditions.

