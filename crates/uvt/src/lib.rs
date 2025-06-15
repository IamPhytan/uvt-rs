use std::io::Error;
use std::path;
use std::{fs, time::Duration};

use rosbag::{ChunkRecord, IndexRecord, MessageRecord, RosBag};
use tqdm::Iter;
use vtkio::Vtk;

mod deserialization;
mod pointcloud;
mod pose;
pub use pose::Point;

const TRAJ_DELIM: &str = "#############################";

pub struct Uvt {
    pub map: vtkio::Vtk,
    pub trajectory: Vec<pose::PoseStamped>,
}

impl Uvt {
    pub fn read_file<P: AsRef<path::Path>>(path: P) -> Result<Self, Error> {
        let fpath = path.as_ref();
        let content = fs::read_to_string(fpath)?;

        println!(
            "Reading uvt file in {}",
            path::absolute(fpath).unwrap().display()
        );

        let delimiter = content.find(TRAJ_DELIM).unwrap();
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

    pub fn read_rosbag<P: AsRef<path::Path>>(
        path: P,
        map_topic: &str,
        traj_topic: &str,
    ) -> Result<Self, Error> {
        println!(
            "Reading rosbag file in {}",
            path::absolute(&path).unwrap().display()
        );

        let bag = RosBag::new(path)?;

        let map_msgs = Self::retrieve_topic_messages(&bag, map_topic);
        let traj_msgs = Self::retrieve_topic_messages(&bag, traj_topic);

        let maps: Vec<pointcloud::PointCloud2> = map_msgs
            .iter()
            .tqdm()
            .desc(Some("Reading map msgs"))
            .map(|msg| msg.clone().into())
            .collect();
        let trajectory: Vec<pose::PoseStamped> = traj_msgs
            .iter()
            .tqdm()
            .desc(Some("Reading trajectory msgs"))
            .map(|msg| msg.clone().into())
            .collect();

        let pointclouds: Vec<Vec<pose::Point>> = maps
            .iter()
            .tqdm()
            .desc(Some("Retrieving pointclouds"))
            .map(|m| m.points())
            .collect();

        todo!("Retrieve points from maps data field");

        // Ok(Self {
        //     map: maps,
        //     trajectory: trajectory,
        // })
    }
}
