use std::io::{Error, ErrorKind};

use crate::deserialization::MessageDataBuffer;
use crate::pointcloud::{PointCloud2Deserializer, PointField};
use crate::{pointcloud, pose};

pub struct BagDeserializer {
    buf: MessageDataBuffer,
}

impl BagDeserializer {
    pub fn new(data: Vec<u8>) -> Self {
        Self {
            buf: MessageDataBuffer::new(data),
        }
    }
}

impl PointCloud2Deserializer for BagDeserializer {
    fn read_header(&mut self) -> Result<pose::Header, std::io::Error> {
        // TODO: Remove unwraps
        Ok(pose::Header {
            seq: self.buf.read_u32_le().unwrap().clone(),
            stamp: pose::Time {
                sec: self.buf.read_i32_le().unwrap().clone(),
                nanosec: self.buf.read_u32_le().unwrap().clone(),
            },
            frame_id: self.buf.read_lp_string().unwrap(),
        })
    }

    fn read_u32_le(&mut self) -> Result<u32, std::io::Error> {
        self.buf.read_u32_le()
    }

    fn read_byte(&mut self) -> Result<u8, std::io::Error> {
        self.buf.read_byte()
    }

    fn read_lp_string(&mut self) -> Result<String, std::io::Error> {
        self.buf.read_lp_string()
    }

    fn read_null_terminated_string(&mut self) -> Result<String, std::io::Error> {
        self.buf.read_null_terminated_string()
    }

    fn read_point_field(&mut self) -> Result<pointcloud::PointField, std::io::Error> {
        Ok(PointField {
            name: self.buf.read_lp_string().unwrap(),
            offset: self.buf.read_u32_le().unwrap(),
            datatype: self.buf.read_byte().unwrap().into(),
            count: self.buf.read_u32_le().unwrap(),
        })
    }

    fn read_point_fields(&mut self) -> Result<Vec<pointcloud::PointField>, std::io::Error> {
        let n_fields = self.buf.read_u32_le()?;
        let fields: Vec<PointField> = (0..n_fields)
            .into_iter()
            .map(|_| self.read_point_field().unwrap())
            .collect();
        Ok(fields)
    }

    fn read_data(&mut self) -> Result<Vec<u8>, std::io::Error> {
        // TODO: Rely on fields
        // Point cloud data, size is (row_step*height)
        let data_len = self.buf.read_u32_le()?;
        let data: Vec<u8> = self
            .buf
            .slice(data_len as usize)
            .ok_or_else(|| {
                Error::new(
                    ErrorKind::UnexpectedEof,
                    "Not enough bytes to read the pointcloud data with the specified length",
                )
            })?
            .to_vec();
        Ok(data)
    }
}
