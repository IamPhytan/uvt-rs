//! This module provides functionality to parse trajectory data from ROS messages.
use crate::deserialization::BufferReader;
use crate::pose;
use std::io;

pub trait TrajectoryDeserializer: BufferReader {
    fn read_position(&mut self) -> Result<pose::Point, io::Error>;
    fn read_orientation(&mut self) -> Result<pose::Quaternion, io::Error>;
    fn read_covariance(&mut self) -> Result<Vec<f64>, io::Error>;
    fn read_vector(&mut self) -> Result<pose::Vector3, io::Error>;
}

/// Parses a trajectory message from raw data into a sequence of PoseStamped instances.
///
/// # Arguments
///
/// * `deserializer` - An instance that converts the raw message into structured data.
///
/// # Returns
///
/// A vector of PoseStamped elements representing the trajectory.
pub fn parse_trajectory<D: TrajectoryDeserializer>(
    mut d: D,
) -> Result<pose::PoseStamped, std::io::Error> {
    // Message header
    let header = d.read_header()?;

    let _child_frame = d.read_lp_string_aligned(8)?;

    // Message pose
    let position = d.read_position()?;
    let orientation = d.read_orientation()?;

    // TODO: Implement PoseWithCovarianceStamped
    // https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseWithCovarianceStamped.html

    // Pose covariance
    // 6 x 6 covariance matrix = 36 covariance values
    let _pose_covariance = d.read_covariance()?;

    // Twist values
    let _twist_linear = d.read_vector()?;
    let _twist_angular = d.read_vector()?;

    // Twist covariance
    // 6 x 6 covariance matrix = 36 covariance values
    let _twist_covariance = d.read_covariance()?;

    Ok(pose::PoseStamped {
        header: header,
        pose: pose::Pose {
            position: position,
            orientation: orientation,
        },
    })
}
