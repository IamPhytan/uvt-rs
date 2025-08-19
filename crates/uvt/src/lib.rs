use rayon::prelude::*;
use std::io::Error;
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
mod pose;
mod trajectory;
pub use pose::Point;

use memmap2::Mmap;

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
        let absolute_path = path::absolute(&path).unwrap();

        println!("Reading rosbag file in {}", absolute_path.clone().display());

        let fname = absolute_path.file_name().unwrap().to_str().unwrap();

        let bag = RosBag::new(path)?;

        let map_msgs = Self::retrieve_topic_messages(&bag, map_topic);
        let traj_msgs = Self::retrieve_topic_messages(&bag, traj_topic);

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
            .map(|msg| pose::PoseStamped::from_msg_data(msg.to_vec()))
            .collect();

        let pointclouds: Vec<Vec<pose::Point>> =
            maps.par_iter().map(|m| m.to_owned().into()).collect();

        println!("Got pointclouds");

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

    // TODO: Read MCAP
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

        for (i, msg_data) in traj_msgs.iter().enumerate() {
            if i < 2 {
                let mut msg_buf = deserialization::MessageDataBuffer::new(msg_data.clone());

                let _ = msg_buf.dump_to_file(format!("data/traj-{}.hex", fname).as_str());
            }
        }

        // let trajectory: Vec<pose::PoseStamped> = traj_msgs
        //     .iter()
        //     .tqdm()
        //     .desc(Some("Reading trajectory msgs"))
        //     .map(|msg| pose::PoseStamped::from_msg_data(msg.to_vec()))
        //     .collect();

        todo!("Unpack MCAP file");
    }
}
