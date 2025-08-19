use crate::deserialization::BufferReader;
use crate::pose;
use std::io::{self, Error};

pub trait TrajectoryDeserializer: BufferReader {
    fn read_position(&mut self) -> Result<pose::Point, io::Error>;
    fn read_orientation(&mut self) -> Result<pose::Quaternion, io::Error>;
    fn read_covariance(&mut self) -> Result<Vec<f64>, io::Error>;
    fn read_vector(&mut self) -> Result<pose::Vector3, io::Error>;
}

pub fn parse_trajectory<D: TrajectoryDeserializer>(
    mut d: D,
) -> Result<pose::PoseStamped, std::io::Error> {
    // Message header
    let header = d.read_header()?;
    Err(Error::new(io::ErrorKind::InvalidData, "Not implemented"))
}
