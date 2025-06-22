use std::collections::HashMap;

use crate::deserialization::MessageDataBuffer;
use crate::pose;

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

impl PointCloud2 {
    pub fn len(&self) -> usize {
        self.data.len()
    }

    pub fn n_points(&self) -> usize {
        self.len() / (self.point_step as usize)
    }

    pub fn points(&self) -> Vec<HashMap<String, f32>> {
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
                        DataType::FLOAT32 => data_buf.read_f32_le().unwrap(),
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
                x: pt_hashmap["x"] as f64,
                y: pt_hashmap["y"] as f64,
                z: pt_hashmap["z"] as f64,
            })
            .collect()
    }
}

impl From<Vec<u8>> for PointCloud2 {
    fn from(msg_data: Vec<u8>) -> Self {
        let mut msg_buf = MessageDataBuffer::new(msg_data);

        // Message header
        let header = pose::Header {
            seq: msg_buf.read_u32_le().unwrap().clone(),
            stamp: pose::Time {
                sec: msg_buf.read_i32_le().unwrap().clone(),
                nanosec: msg_buf.read_u32_le().unwrap().clone(),
            },
            frame_id: msg_buf.read_lp_string().unwrap(),
        };

        // 2D structure of the point cloud
        let height = msg_buf.read_u32_le().unwrap();
        let width = msg_buf.read_u32_le().unwrap();

        // Fields
        let n_fields = msg_buf.read_u32_le().unwrap();
        let fields: Vec<PointField> = (0..n_fields)
            .into_iter()
            .map(|_| PointField {
                name: msg_buf.read_lp_string().unwrap(),
                offset: msg_buf.read_u32_le().unwrap(),
                datatype: msg_buf.read_byte().unwrap().into(),
                count: msg_buf.read_u32_le().unwrap(),
            })
            .collect();

        // Is this data bigendian?
        let is_bigendian = msg_buf.read_byte().unwrap() == 1;
        // Length of a point in bytes
        let point_step = msg_buf.read_u32_le().unwrap();
        // Length of a row in bytes
        let row_step = msg_buf.read_u32_le().unwrap();

        // Actual point data, size is (row_step*height)
        // TODO: Rely on fields
        let num_data_bytes = msg_buf.read_u32_le().unwrap();
        let data: Vec<u8> = msg_buf.slice(num_data_bytes as usize).unwrap().to_owned();

        // Is dense
        let is_dense = msg_buf.read_byte().unwrap() == 1;

        Self {
            header: header,
            height: height,
            width: width,
            fields: fields,
            is_bigendian: is_bigendian,
            point_step: point_step,
            row_step: row_step,
            data: data,
            is_dense: is_dense,
        }
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
