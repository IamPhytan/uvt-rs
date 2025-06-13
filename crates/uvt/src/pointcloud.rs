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

        let height = msg_buf.read_u32_le().unwrap();
        let width = msg_buf.read_u32_le().unwrap();

        println!("{header:?}");
        println!("{:?}", msg_buf.slice(20).unwrap());

        todo!("Implement this function");
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
