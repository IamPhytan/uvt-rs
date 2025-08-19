use std::io::{Error, ErrorKind};

use crate::deserialization::{BufferReader, MessageDataBuffer};
use crate::pointcloud::{PointCloud2Deserializer, PointField};
use crate::{pointcloud, pose};

pub struct McapDeserializer {
    buf: MessageDataBuffer,
}

impl McapDeserializer {
    pub fn new(data: Vec<u8>) -> Self {
        Self {
            buf: MessageDataBuffer::new(data),
        }
    }
}

impl BufferReader for McapDeserializer {
    fn read_u32_le(&mut self) -> Result<u32, std::io::Error> {
        self.buf.read_u32_le()
    }

    fn read_f64_le(&mut self) -> Result<f64, std::io::Error> {
        self.buf.read_f64_le()
    }

    fn read_byte(&mut self) -> Result<u8, std::io::Error> {
        let b = self.buf.read_byte();
        let _ = self.buf.slice(3);
        b
    }

    fn read_lp_string(&mut self) -> Result<String, std::io::Error> {
        let s = self.buf.read_lp_string()?;
        // Remove null termination byte
        let strdata = s.trim_end_matches("\0").to_string();
        // Respect 4-byte data alignment of CDR
        let strlen = s.len();
        let padding = 4 - (strlen % 4);
        let _ = self.buf.slice(padding);
        Ok(strdata)
    }

    fn read_null_terminated_string(&mut self) -> Result<String, std::io::Error> {
        self.buf.read_null_terminated_string()
    }

    fn read_header(&mut self) -> Result<pose::Header, std::io::Error> {
        // TODO: Remove unwraps

        // Ignore first 4 bytes
        let _ = self.buf.read_u32_le();
        Ok(pose::Header {
            seq: 0,
            stamp: pose::Time {
                sec: self.buf.read_i32_le()?,
                nanosec: self.read_u32_le()?,
            },
            frame_id: self.read_lp_string()?,
        })
    }
}

impl PointCloud2Deserializer for McapDeserializer {
    fn read_point_field(&mut self) -> Result<pointcloud::PointField, std::io::Error> {
        Ok(PointField {
            name: self.read_lp_string()?,
            offset: self.read_u32_le()?,
            datatype: self.read_byte()?.into(),
            count: self.read_u32_le()?,
        })
    }

    fn read_point_fields(&mut self) -> Result<Vec<pointcloud::PointField>, std::io::Error> {
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
