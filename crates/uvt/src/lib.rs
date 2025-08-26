//! # uvt
//!
//! The `uvt` crate provides utilities for working with _Uncrewed Vehicle Trajectory_ (UVT) files.
//!
//! UVT files combine:
//! - A Lidar map of the environment (stored in VTK format).
//! - A trajectory (poses over time), extracted from ROS bag (`.bag`) or MCAP (`.mcap`) recordings.
//!
//! ## Features
//! - Read/write `.uvt` files
//! - Extract map and trajectory data from `.bag` and `.mcap` logs
//!
//! ## Example
//! ```no_run
//! use uvt::Uvt;
//!
//! // Generate a UVt from a bag file
//! let uvt = Uvt::read_rosbag("dataset.bag", "/map", "/trajectory").unwrap();
//!
//! // Write it back to UVT format
//! uvt.write_file("output.uvt").unwrap();
//! ```
use rayon::prelude::*;
use std::io::{Error, ErrorKind};
use std::path;
use std::{fs, time::Duration};

extern crate mcap as mcap_crate;

use rosbag::{ChunkRecord, IndexRecord, MessageRecord, RosBag};
use tqdm::Iter;
use vtkio::Vtk;

mod bag;
mod deserialization;
mod mcap;
mod pointcloud;
pub mod pose;
mod trajectory;
pub use pose::Point;

use memmap2::Mmap;

const TRAJ_DELIM: &str = "#############################";

/// A UVT (_Uncrewed Vehicle Trajectory_)
///
/// Contains:
/// - A map of the environment (`vtkio::Vtk`)
/// - A trajectory (sequence of stamped poses)
pub struct Uvt {
    /// The environment map
    pub map: vtkio::Vtk,
    /// The vehicle's trajectory, saved as a sequence of stamped poses.
    pub trajectory: Vec<pose::PoseStamped>,
}

impl Uvt {
    /// Read a UVT file from disk.
    /// A UVT file contains both a VTK map and a trajectory.
    ///
    ///
    /// # Arguments
    ///
    /// * `path` - A path to the UVT file.
    ///
    /// # Example
    /// ```no_run
    /// use uvt::Uvt;
    ///
    /// let my_uvt = Uvt::read_file("my_uvt.uvt").unwrap();
    /// ```
    ///
    /// # Errors
    /// Returns an error if:
    /// - The file cannot be read
    /// - The VTK or trajectory data is malformed
    /// = The UVT file does not follow the UVT format
    pub fn read_file<P: AsRef<path::Path>>(path: P) -> Result<Self, Error> {
        let fpath = path.as_ref();
        let content = fs::read_to_string(fpath)?;

        println!(
            "Reading uvt file in {}",
            path::absolute(fpath).unwrap().display()
        );

        let delimiter = content.find(TRAJ_DELIM).ok_or(Error::new(
            ErrorKind::InvalidData,
            "Could not find Trajectory delimiter in UVT file",
        ))?;
        let vtk_str = content[..delimiter].trim();
        let traj_str = content[delimiter + TRAJ_DELIM.len()..].trim();

        let vtk_file =
            Vtk::parse_legacy_be(vtk_str.as_bytes()).expect(&format!("Failed to parse vtk"));

        let frame_id = traj_str
            .lines()
            .next()
            .unwrap()
            .split_once(":")
            .expect("Expected frame_id line following 'frame_id : <value>'")
            .1
            .trim();

        let trajectory: Vec<pose::PoseStamped> = traj_str
            .lines()
            .skip(1)
            .enumerate()
            .map(|(i, line)| {
                let values: Vec<f64> = line
                    .split(",")
                    .map(|n| {
                        n.trim().parse::<f64>().unwrap_or_else(|_| {
                            panic!("Failed to parse floats in line {}: '{}'", i + 2, line)
                        })
                    })
                    .collect::<Vec<f64>>();
                if values.len() != 6 {
                    panic!(
                        "Line {}: expected 6 values, got {} - '{}'",
                        i + 2,
                        values.len(),
                        line
                    );
                }

                // TODO: Get more info, with time
                let header = pose::Header {
                    frame_id: frame_id.to_string(),
                    seq: (i + 2) as u32,
                    stamp: Duration::from_secs(0).into(),
                };

                pose::PoseStamped::new(
                    header,
                    pose::Pose::from_6dof((
                        values[0], values[1], values[2], // X, Y, Z
                        values[3], values[4], values[5], // Roll, Pitch, Yaw
                    )),
                )
            })
            .collect();

        Ok(Self {
            map: vtk_file,
            trajectory: trajectory,
        })
    }

    /// Retrieves messages for a given topic from a ROS bag.
    ///
    /// This internal method extracts messages that match a specified topic.
    ///
    /// # Arguments
    ///
    /// * `bag` - A reference to a `RosBag` instance.
    /// * `topic` - The name of the topic for which to retrieve messages.
    ///
    /// # Returns
    ///
    /// A vector of message data as byte vectors.
    fn retrieve_topic_messages<'a>(bag: &'a RosBag, topic: &str) -> Vec<Vec<u8>> {
        let connections: Vec<_> = bag
            .index_records()
            .filter_map(Result::ok)
            .filter_map(|record| match record {
                IndexRecord::Connection(conn) => Some(conn),
                _ => None,
            })
            .collect();

        let topic_conns: Vec<_> = connections
            .iter()
            .filter(|conn| conn.topic == topic)
            .collect();

        let conn_id = topic_conns[0].id;

        let topic_msgs: Vec<Vec<u8>> = bag
            .chunk_records()
            .filter_map(Result::ok)
            .filter_map(|record| match record {
                ChunkRecord::Chunk(chunk) => Some(chunk),
                _ => None,
            })
            .map(|chunk| {
                chunk
                    .messages()
                    .filter_map(Result::ok)
                    .filter_map(|msg| match msg {
                        MessageRecord::MessageData(msg_data) => Some(msg_data.clone()),
                        _ => None,
                    })
                    .filter(|msg| msg.conn_id == conn_id)
                    .map(|msg| msg.data.to_vec())
                    .collect::<Vec<Vec<u8>>>()
            })
            .flatten()
            .collect();
        topic_msgs
    }

    /// Reads a ROS bag file and extracts UVT data.
    ///
    /// The method reads messages for the map and trajectory topics, parses pointcloud data,
    /// and constructs a VTK map using the last pointcloud.
    ///
    /// # Arguments
    ///
    /// * `path` - A path to the ROS bag file.
    /// * `map_topic` - The topic name for map messages.
    /// * `traj_topic` - The topic name for trajectory messages.
    ///
    /// # Returns
    ///
    /// A `Uvt` instance containing the map and trajectory.
    ///
    /// # Errors
    ///
    /// Returns an error if the ROS bag file cannot be read or parsed.
    pub fn read_rosbag<P: AsRef<path::Path>>(
        path: P,
        map_topic: &str,
        traj_topic: &str,
    ) -> Result<Self, Error> {
        let absolute_path = path::absolute(&path).unwrap();

        println!("Reading rosbag file in {}", absolute_path.clone().display());

        let fname = absolute_path.file_name().unwrap().to_str().unwrap();

        let bag = RosBag::new(path)?;

        let map_msgs = Self::retrieve_topic_messages(&bag, map_topic);
        let traj_msgs = Self::retrieve_topic_messages(&bag, traj_topic);

        // Collect maps and trajectory
        let maps: Vec<pointcloud::PointCloud2> = map_msgs
            .iter()
            .tqdm()
            .desc(Some("Reading map msgs"))
            .map(|msg| {
                pointcloud::parse_pointcloud::<bag::BagDeserializer>(bag::BagDeserializer::new(
                    msg.to_vec(),
                ))
                .unwrap()
            })
            .collect();
        let trajectory: Vec<pose::PoseStamped> = traj_msgs
            .iter()
            .tqdm()
            .desc(Some("Reading trajectory msgs"))
            .map(|msg| {
                trajectory::parse_trajectory::<bag::BagDeserializer>(bag::BagDeserializer::new(
                    msg.to_vec(),
                ))
                .unwrap()
            })
            .collect();

        // Retrieve points from pointclouds
        let pointclouds: Vec<Vec<pose::Point>> =
            maps.par_iter().map(|m| m.to_owned().into()).collect();
        println!("Retrieved points from pointclouds");

        // Use last pointcloud as the map
        let last_pcloud = pointclouds[pointclouds.len() - 1].clone();
        let pts: Vec<f32> = last_pcloud
            .par_iter()
            .map(|&pt| Into::<[f32; 3]>::into(pt))
            .flatten()
            .collect::<Vec<f32>>()
            .try_into()
            .unwrap();
        let data = vtkio::model::DataSet::inline(vtkio::model::PolyDataPiece {
            points: vtkio::IOBuffer::F32(pts),
            verts: None,
            lines: None,
            polys: None,
            strips: None,
            data: vtkio::model::Attributes::new(),
        });

        let map_vtk = Vtk {
            version: vtkio::model::Version { major: 3, minor: 0 },
            byte_order: vtkio::model::ByteOrder::BigEndian,
            title: String::from(format!("UVT file generated from {}", fname)),
            file_path: None,
            data: data,
        };

        Ok(Self {
            map: map_vtk,
            trajectory: trajectory,
        })
    }

    /// Retrieves messages for a given topic from an MCAP file.
    ///
    /// This internal method reads an MCAP memory-mapped file and extracts the messages
    /// matching the specified topic.
    ///
    /// # Arguments
    ///
    /// * `mcap_map` - A memory-mapped representation of an MCAP file.
    /// * `topic` - The name of the topic for which to retrieve messages.
    ///
    /// # Returns
    ///
    /// A vector of message data as byte vectors.
    fn retrieve_mcap_topic_messages<'a>(mcap_map: &Mmap, topic: &str) -> Vec<Vec<u8>> {
        let messages = mcap_crate::MessageStream::new(&mcap_map).unwrap();
        let topic_msgs = messages
            .into_iter()
            .filter_map(|stream_msg| {
                let msg = stream_msg.unwrap();
                let msg_topic = msg.channel.topic.as_str();
                if msg_topic == topic {
                    Some(msg.data.to_vec())
                } else {
                    None
                }
            })
            .collect();
        topic_msgs
    }

    /// Reads an MCAP file and extracts UVT data.
    ///
    /// The method reads messages for the map and trajectory topics, parses pointcloud data,
    /// and constructs a VTK map using the last pointcloud.
    ///
    /// # Arguments
    ///
    /// * `path` - A path to the MCAP file.
    /// * `map_topic` - The topic name for map messages.
    /// * `traj_topic` - The topic name for trajectory messages.
    ///
    /// # Returns
    ///
    /// A `Uvt` instance containing the map and trajectory.
    ///
    /// # Errors
    ///
    /// Returns an error if the MCAP file cannot be read or parsed.
    pub fn read_mcap<P: AsRef<path::Path>>(
        path: P,
        map_topic: &str,
        traj_topic: &str,
    ) -> Result<Self, Error> {
        let absolute_path = path::absolute(&path).unwrap();
        println!("Reading MCAP file in {}", absolute_path.clone().display());

        let fname = absolute_path.file_name().unwrap().to_str().unwrap();

        let fd = fs::File::open(path.as_ref()).expect("Couldn't open MCAP file");
        let mapped = unsafe { Mmap::map(&fd) }?;
        println!("MCAP file opened !");

        let map_msgs = Self::retrieve_mcap_topic_messages(&mapped, map_topic);
        let traj_msgs = Self::retrieve_mcap_topic_messages(&mapped, traj_topic);

        // Collect maps and trajectory
        let maps: Vec<pointcloud::PointCloud2> = map_msgs
            .iter()
            .tqdm()
            .desc(Some("Reading map msgs"))
            .map(|msg| {
                pointcloud::parse_pointcloud::<mcap::McapDeserializer>(mcap::McapDeserializer::new(
                    msg.to_vec(),
                ))
                .unwrap()
            })
            .collect();
        let trajectory: Vec<pose::PoseStamped> = traj_msgs
            .iter()
            .tqdm()
            .desc(Some("Reading trajectory msgs"))
            .map(|msg| {
                trajectory::parse_trajectory::<mcap::McapDeserializer>(mcap::McapDeserializer::new(
                    msg.to_vec(),
                ))
                .unwrap()
            })
            .collect();

        // Retrieve points from pointclouds
        let pointclouds: Vec<Vec<pose::Point>> =
            maps.par_iter().map(|m| m.to_owned().into()).collect();
        println!("Retrieved points from pointclouds");

        // Use last pointcloud as the map
        let last_pcloud = pointclouds[pointclouds.len() - 1].clone();
        let pts: Vec<f32> = last_pcloud
            .par_iter()
            .map(|&pt| Into::<[f32; 3]>::into(pt))
            .flatten()
            .collect::<Vec<f32>>()
            .try_into()
            .unwrap();
        let data = vtkio::model::DataSet::inline(vtkio::model::PolyDataPiece {
            points: vtkio::IOBuffer::F32(pts),
            verts: None,
            lines: None,
            polys: None,
            strips: None,
            data: vtkio::model::Attributes::new(),
        });

        let map_vtk = Vtk {
            version: vtkio::model::Version { major: 3, minor: 0 },
            byte_order: vtkio::model::ByteOrder::BigEndian,
            title: String::from(format!("UVT file generated from {}", fname)),
            file_path: None,
            data: data,
        };

        Ok(Self {
            map: map_vtk,
            trajectory: trajectory,
        })
    }

    /// Writes the UVT data (map and trajectory) to a file.
    ///
    /// The output file contains a VTK map encoded in legacy ASCII format,
    /// followed by a delimiter and the trajectory data.
    ///
    /// # Arguments
    ///
    /// * `path` - The destination file path.
    ///
    /// # Returns
    ///
    /// `Ok(())` if the file was written successfully, or an `Error` otherwise.
    pub fn write_file<P: AsRef<path::Path>>(&self, path: P) -> Result<(), std::io::Error> {
        let export_path = path::absolute(path)?.clone();
        println!("Writing file to {}", export_path.display());

        //
        // Map
        //
        let mut map_str = String::new();
        Vtk::write_legacy_ascii(self.map.clone(), &mut map_str)
            .expect(&format!("Failed to write file"));

        //
        // Trajectory
        //
        let uvt_trajectory = self.trajectory.clone();

        // Retrieve frame ID
        let frame_id = uvt_trajectory
            .first()
            .ok_or(Error::new(ErrorKind::InvalidData, "Missing poses"))?
            .header
            .frame_id
            .clone();
        let frame_str = format!("frame_id : {}", frame_id);

        let traj_poses: Vec<String> = uvt_trajectory
            .into_iter()
            .map(|pose| {
                let dofs = pose.pose.to_6dof();
                let x = pose::round(dofs.0, 6);
                let y = pose::round(dofs.1, 6);
                let z = pose::round(dofs.2, 6);
                let roll = pose::round(dofs.3, 6);
                let pitch = pose::round(dofs.4, 6);
                let yaw = pose::round(dofs.5, 6);

                format!("{x},{y},{z},{roll},{pitch},{yaw}")
            })
            .collect();
        let traj_str = traj_poses.join("\n");
        let uvt_str = vec![map_str, TRAJ_DELIM.to_string(), frame_str, traj_str].join("\n");

        fs::write(export_path, uvt_str)?;

        Ok(())
    }
}
