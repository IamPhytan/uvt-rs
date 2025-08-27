//! This module provides functionality to parse point cloud data from ROS messages.
use std::collections::HashMap;

use crate::deserialization::{BufferReader, MessageDataBuffer};
use crate::pose;
use std::io;

/// Trait for deserializing PointCloud2 messages.
///# Methods
///* `read_point_field` - Reads a single PointField from the buffer.
///* `read_point_fields` - Reads multiple PointFields from the buffer.
///* `read_data` - Reads the raw point cloud data from the buffer.
pub trait PointCloud2Deserializer: BufferReader {
    fn read_point_field(&mut self) -> Result<PointField, io::Error>;
    fn read_point_fields(&mut self) -> Result<Vec<PointField>, io::Error>;
    fn read_data(&mut self) -> Result<Vec<u8>, io::Error>;
}

#[derive(Debug, Clone, PartialEq)]
pub enum DataType {
    INT8 = 1,
    UINT8 = 2,
    INT16 = 3,
    UINT16 = 4,
    INT32 = 5,
    UINT32 = 6,
    FLOAT32 = 7,
    FLOAT64 = 8,
}

//# Analog to sensor_msgs/msg/PointField in ROS
#[derive(Debug, Clone, PartialEq)]
pub struct PointField {
    pub name: String,
    pub offset: u32,
    pub datatype: DataType,
    pub count: u32,
}

//# Analog to sensor_msgs/msg/PointCloud2 in ROS
#[derive(Debug, Clone, PartialEq)]
pub struct PointCloud2 {
    pub header: pose::Header,
    pub height: u32,
    pub width: u32,
    pub fields: Vec<PointField>,
    pub is_bigendian: bool,
    pub point_step: u32,
    pub row_step: u32,
    pub data: Vec<u8>,
    pub is_dense: bool,
}

/// Parses raw point cloud data using the provided deserializer.
///
/// # Arguments
///
/// * `deserializer` - An instance implementing the deserialization trait.
///
/// # Returns
///
/// A PointCloud2 structure parsed from the raw data.
pub fn parse_pointcloud<D: PointCloud2Deserializer>(
    mut d: D,
) -> Result<PointCloud2, std::io::Error> {
    // Message header
    let header = d.read_header()?;

    // 2D structure of the point cloud
    let height = d.read_u32_le()?;
    let width = d.read_u32_le()?;

    // Fields
    let fields = d.read_point_fields()?;

    // Is this data bigendian?
    let is_bigendian = d.read_byte_aligned(4)? == 1;
    // Length of a point in bytes
    let point_step = d.read_u32_le()?;
    // Length of a row in bytes
    let row_step = d.read_u32_le()?;

    // Actual pointcloud data
    let data = d.read_data()?;

    // Is dense
    let is_dense = d.read_byte_aligned(4)? == 1;

    Ok(PointCloud2 {
        header,
        height,
        width,
        fields,
        is_bigendian,
        point_step,
        row_step,
        data,
        is_dense,
    })
}

impl PointCloud2 {
    pub fn len(&self) -> usize {
        self.data.len()
    }

    pub fn n_points(&self) -> usize {
        self.len() / (self.point_step as usize)
    }

    pub fn points(&self) -> Vec<HashMap<String, f64>> {
        let pt_len = self.point_step as usize;

        // Use a MessageDataBuffer to deserialize data
        let mut data_buf = MessageDataBuffer::new(self.data.to_vec());

        let points = (0..self.n_points())
            .into_iter()
            .map(|i| {
                // Create point from fields
                let mut point = HashMap::new();
                for field in &self.fields {
                    let value = match field.datatype {
                        DataType::FLOAT64 => data_buf.read_f64_le().unwrap(),
                        DataType::FLOAT32 => data_buf.read_f32_le().unwrap() as f64,
                        DataType::UINT16 => data_buf.read_u16_le().unwrap() as f64,
                        _ => panic!("Unsupported datatype: {:?}", field.datatype),
                    };
                    point.insert(field.name.clone(), value);
                }
                point
            })
            .collect();

        points
    }
}

impl Into<Vec<pose::Point>> for PointCloud2 {
    fn into(self) -> Vec<pose::Point> {
        let points = self.points();
        points
            .iter()
            .map(|pt_hashmap| pose::Point {
                x: pt_hashmap["x"],
                y: pt_hashmap["y"],
                z: pt_hashmap["z"],
            })
            .collect()
    }
}

impl From<u8> for DataType {
    fn from(byte: u8) -> Self {
        match byte {
            1_u8 => Self::INT8,
            2_u8 => Self::UINT8,
            3_u8 => Self::INT16,
            4_u8 => Self::UINT16,
            5_u8 => Self::INT32,
            6_u8 => Self::UINT32,
            7_u8 => Self::FLOAT32,
            8_u8 => Self::FLOAT64,
            _ => panic!("Unknown byte value"),
        }
    }
}
