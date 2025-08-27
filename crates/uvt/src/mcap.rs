use std::io::{Error, ErrorKind};

use crate::deserialization::{BufferReader, MessageDataBuffer};
use crate::pointcloud::{PointCloud2Deserializer, PointField};
use crate::trajectory::TrajectoryDeserializer;
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
        self.buf.read_byte()
    }

    fn read_byte_aligned(&mut self, next_alignment: usize) -> Result<u8, std::io::Error> {
        let b = self.read_byte()?;

        // Align to next field
        // Respect 4-byte or 8-byte data alignment of CDR
        let padding = (next_alignment - (1 % next_alignment)) % next_alignment;
        if padding > 0 {
            let _ = self.buf.slice(padding);
        }

        Ok(b)
    }

    fn slice(&mut self, length: usize) -> Option<&[u8]> {
        self.buf.slice(length)
    }

    fn read_lp_string(&mut self) -> Result<String, std::io::Error> {
        let s = self.buf.read_lp_string()?;
        Ok(s)
    }

    fn read_lp_string_aligned(&mut self, next_alignment: usize) -> Result<String, std::io::Error> {
        let s = self.buf.read_lp_string()?;

        let strdata = s.trim_end_matches("\0").to_string();

        // Align to next field
        // Respect 4-byte or 8-byte data alignment of CDR
        let strlen = s.len();
        let padding = (next_alignment - (strlen % next_alignment)) % next_alignment;

        if padding > 0 {
            let _ = self.buf.slice(padding);
        }

        Ok(strdata)
    }

    fn read_null_terminated_string(&mut self) -> Result<String, std::io::Error> {
        self.buf.read_null_terminated_string()
    }

    fn read_header(&mut self) -> Result<pose::Header, std::io::Error> {
        // Ignore first 4 bytes
        // Used in CDR for endianness
        let _ = self.buf.read_u32_le();

        Ok(pose::Header {
            seq: 0,
            stamp: pose::Time {
                sec: self.buf.read_i32_le()?,
                nanosec: self.read_u32_le()?,
            },
            frame_id: self.read_lp_string_aligned(4)?,
        })
    }
}

impl PointCloud2Deserializer for McapDeserializer {
    /// Read a single point field
    fn read_point_field(&mut self) -> Result<pointcloud::PointField, std::io::Error> {
        Ok(PointField {
            name: self.read_lp_string_aligned(4)?,
            offset: self.read_u32_le()?,
            datatype: self.read_byte_aligned(4)?.into(),
            count: self.read_u32_le()?,
        })
    }

    /// Read all point fields
    fn read_point_fields(&mut self) -> Result<Vec<pointcloud::PointField>, std::io::Error> {
        let n_fields = self.buf.read_u32_le()?;
        let fields = (0..n_fields)
            .into_iter()
            .map(|_| self.read_point_field())
            .into_iter()
            .collect();
        fields
    }

    /// Read point cloud data
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

impl TrajectoryDeserializer for McapDeserializer {
    /// Read position point
    fn read_position(&mut self) -> Result<pose::Point, std::io::Error> {
        Ok(pose::Point {
            x: self.buf.read_f64_le()?,
            y: self.buf.read_f64_le()?,
            z: self.buf.read_f64_le()?,
        })
    }

    /// Read pose orientation as a quaternion
    fn read_orientation(&mut self) -> Result<pose::Quaternion, std::io::Error> {
        Ok(pose::Quaternion {
            x: self.buf.read_f64_le()?,
            y: self.buf.read_f64_le()?,
            z: self.buf.read_f64_le()?,
            w: self.buf.read_f64_le()?,
        })
    }

    /// Read covariance values
    /// 6 x 6 covariance matrix = 36 covariance values
    /// https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseWithCovarianceStamped.html
    fn read_covariance(&mut self) -> Result<Vec<f64>, std::io::Error> {
        (0..36)
            .into_iter()
            .map(|_| self.buf.read_f64_le())
            .into_iter()
            .collect()
    }

    /// Read a twist vector
    fn read_vector(&mut self) -> Result<pose::Vector3, std::io::Error> {
        Ok(pose::Vector3::new(
            self.buf.read_f64_le()?,
            self.buf.read_f64_le()?,
            self.buf.read_f64_le()?,
        ))
    }
}
