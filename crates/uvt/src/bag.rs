use std::io::{Error, ErrorKind};

use crate::deserialization::{BufferReader, MessageDataBuffer};
use crate::pointcloud::{PointCloud2Deserializer, PointField};
use crate::pose;
use crate::trajectory::TrajectoryDeserializer;

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

impl BufferReader for BagDeserializer {
    fn read_u32_le(&mut self) -> Result<u32, std::io::Error> {
        self.buf.read_u32_le()
    }

    fn read_f64_le(&mut self) -> Result<f64, std::io::Error> {
        self.buf.read_f64_le()
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

    fn read_header(&mut self) -> Result<pose::Header, std::io::Error> {
        Ok(pose::Header {
            seq: self.buf.read_u32_le()?.clone(),
            stamp: pose::Time {
                sec: self.buf.read_i32_le()?.clone(),
                nanosec: self.buf.read_u32_le()?.clone(),
            },
            frame_id: self.read_lp_string()?,
        })
    }
}

impl PointCloud2Deserializer for BagDeserializer {
    fn read_point_field(&mut self) -> Result<PointField, std::io::Error> {
        Ok(PointField {
            name: self.read_lp_string()?,
            offset: self.buf.read_u32_le()?,
            datatype: self.buf.read_byte()?.into(),
            count: self.buf.read_u32_le()?,
        })
    }

    fn read_point_fields(&mut self) -> Result<Vec<PointField>, std::io::Error> {
        let n_fields = self.buf.read_u32_le()?;
        let fields = (0..n_fields)
            .into_iter()
            .map(|_| self.read_point_field())
            .into_iter()
            .collect();
        fields
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

impl TrajectoryDeserializer for BagDeserializer {
    fn read_position(&mut self) -> Result<pose::Point, std::io::Error> {
        Ok(pose::Point {
            x: self.buf.read_f64_le()?,
            y: self.buf.read_f64_le()?,
            z: self.buf.read_f64_le()?,
        })
    }
    fn read_orientation(&mut self) -> Result<pose::Quaternion, std::io::Error> {
        Ok(pose::Quaternion {
            x: self.buf.read_f64_le()?,
            y: self.buf.read_f64_le()?,
            z: self.buf.read_f64_le()?,
            w: self.buf.read_f64_le()?,
        })
    }
    fn read_covariance(&mut self) -> Result<Vec<f64>, std::io::Error> {
        (0..36)
            .into_iter()
            .map(|_| self.buf.read_f64_le())
            .into_iter()
            .collect()
    }
    fn read_vector(&mut self) -> Result<pose::Vector3, std::io::Error> {
        Ok(pose::Vector3::new(
            self.buf.read_f64_le()?,
            self.buf.read_f64_le()?,
            self.buf.read_f64_le()?,
        ))
    }
}
