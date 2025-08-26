//! This module defines types that model ROS messages for Time, Pose, Quaternion, etc.

use std::{
    ops::{Add, Mul, Sub},
    time::Duration,
};

use quaternion_core as quat;
use quaternion_core::RotationSequence::XYZ;
use quaternion_core::RotationType::Extrinsic;

#[cfg(feature = "glam-support")]
use glam;

// HEADER

/// Analog to builtin_interfaces/msg/Time in ROS
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Time {
    pub sec: i32,
    pub nanosec: u32,
}

// Convert a standard Duration into a ROS-like Time message.
impl From<Duration> for Time {
    fn from(duration: Duration) -> Self {
        Time {
            sec: duration.as_secs() as i32,
            nanosec: duration.subsec_nanos(),
        }
    }
}

// Convert a ROS-like Time message back into a standard Duration.
impl From<Time> for Duration {
    fn from(time: Time) -> Self {
        Duration::new(time.sec as u64, time.nanosec)
    }
}

/// A header struct analog to std_msgs/msg/Header in ROS
#[derive(Debug, Clone, PartialEq)]
pub struct Header {
    /// The sequence number is a uint32 that is incremented with each message in ROS stamped messages.
    pub seq: u32,
    /// The stamp is a Time struct representing the time at which the data was recorded.
    pub stamp: Time,
    /// The frame_id is a string that identifies the coordinate frame.
    pub frame_id: String,
}

// POSE

/// Analog to geometry_msgs/msg/Point in ROS
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Point {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Point {
    /// Creates a new point using the given x, y, and z coordinates.
    pub fn new(x: f64, y: f64, z: f64) -> Self {
        Self { x, y, z }
    }
    /// Constructs a point from a tuple of (x, y, z).
    pub fn from_tuple(tup: (f64, f64, f64)) -> Self {
        Self {
            x: tup.0,
            y: tup.1,
            z: tup.2,
        }
    }
    /// Returns the coordinates as a tuple (x, y, z).
    pub fn coords(self) -> (f64, f64, f64) {
        (self.x, self.y, self.z)
    }
}

impl Into<[f32; 3]> for Point {
    fn into(self) -> [f32; 3] {
        [self.x as f32, self.y as f32, self.z as f32]
    }
}

/// Conversion from Point to glam::Vec3 if the "glam-support" feature is enabled.
/// Used to show point clouds in rerun.
#[cfg(feature = "glam-support")]
impl Into<glam::Vec3> for Point {
    fn into(self) -> glam::Vec3 {
        glam::Vec3::new(self.x as f32, self.y as f32, self.z as f32)
    }
}

/// A quaternion struct analog to geometry_msgs/msg/Quaternion in ROS.
/// q = w + xi + yj + zk
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Quaternion {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub w: f64,
}

impl From<quat::Quaternion<f64>> for Quaternion {
    fn from(q: quat::Quaternion<f64>) -> Self {
        Self {
            w: q.0,
            x: q.1[0],
            y: q.1[1],
            z: q.1[2],
        }
    }
}

impl Into<quat::Quaternion<f64>> for Quaternion {
    fn into(self) -> quat::Quaternion<f64> {
        (self.w, [self.x, self.y, self.z])
    }
}

impl Quaternion {
    /// Constructs a new quaternion given x, y, z, and w components.
    pub fn new(x: f64, y: f64, z: f64, w: f64) -> Self {
        Self { x, y, z, w }
    }

    /// Constructs a quaternion from a tuple (x, y, z, w).
    pub fn from_tuple(tup: (f64, f64, f64, f64)) -> Self {
        Self {
            x: tup.0,
            y: tup.1,
            z: tup.2,
            w: tup.3,
        }
    }

    /// Computes the square of the quaternion's length.
    pub fn square_len(&self) -> f64 {
        self.w * self.w + self.x * self.x + self.y * self.y + self.z * self.z
    }

    /// Computes the norm (magnitude) of the quaternion.
    pub fn norm(&self) -> f64 {
        self.square_len().sqrt()
    }

    /// Returns the conjugate of the quaternion.
    /// q' = w - xi - yj - zk
    pub fn conjugate(&self) -> Self {
        Self {
            w: self.w,
            x: -self.x,
            y: -self.y,
            z: -self.z,
        }
    }

    /// Returns a normalized (unit length) version of the quaternion.
    /// Panics if the quaternion has zero length.
    pub fn normalized(&self) -> Self {
        let norm = self.norm();
        if norm == 0.0 {
            panic!("Cannot normalize a quaternion with zero norm");
        }
        self.clone() * (1.0 / norm)
    }
}

impl Add for Quaternion {
    type Output = Self;

    /// Adds two quaternions component-wise.
    /// q1 + q2 = (w1+w2) + (x1+x2)i + (y1+y2)j + (z1+z2)k
    fn add(self, rhs: Self) -> Self {
        Self {
            x: self.x + rhs.x,
            y: self.y + rhs.y,
            z: self.z + rhs.z,
            w: self.w + rhs.w,
        }
    }
}

impl Sub for Quaternion {
    type Output = Self;

    /// Subtracts two quaternions component-wise.
    /// q1 - q2 = (w1-w2) + (x1-x2)i + (y1-y2)j + (z1-z2)k
    fn sub(self, rhs: Self) -> Self {
        Self {
            x: self.x - rhs.x,
            y: self.y - rhs.y,
            z: self.z - rhs.z,
            w: self.w - rhs.w,
        }
    }
}

impl Mul for Quaternion {
    type Output = Self;

    /// Multiplies two quaternions using the Hamilton product.
    /// q1 * q2 = (w1w2 - x1x2 - y1y2 - z1z2) + (w1x2 + x1w2 + y1z2 - z1y2)i
    ///         + (w1y2 - x1z2 + y1w2 + z1x2)j + (w1z2 + x1y2 - y1x2 + z1w2)k
    fn mul(self, rhs: Self) -> Self {
        let mult_w = self.w * rhs.w - self.x * rhs.x - self.y * rhs.y - self.z * rhs.z;
        let mult_x = self.x * rhs.w + self.w * rhs.x - self.z * rhs.y + self.y * rhs.z;
        let mult_y = self.y * rhs.w + self.z * rhs.x + self.w * rhs.y - self.x * rhs.z;
        let mult_z = self.z * rhs.w - self.y * rhs.x + self.x * rhs.y + self.w * rhs.z;

        Self {
            w: mult_w,
            x: mult_x,
            y: mult_y,
            z: mult_z,
        }
    }
}

impl Mul<f64> for Quaternion {
    type Output = Self;

    /// Scales the quaternion by a scalar value.
    /// q * s = (ws) + (xs)i + (ys)j + (zs)k
    fn mul(self, rhs: f64) -> Self {
        Self {
            w: self.w * rhs,
            x: self.x * rhs,
            y: self.y * rhs,
            z: self.z * rhs,
        }
    }
}

/// Analog to geometry_msgs/msg/Pose in ROS
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Pose {
    /// Position represented as a Point (x, y, z)
    pub position: Point,
    /// Orientation represented as a Quaternion (x, y, z, w)
    pub orientation: Quaternion,
}

impl Pose {
    /// Generate a Pose from a tuple of 6 DOFs:
    /// (x, y, z, roll, pitch, yaw).
    /// Angles are in radians
    pub fn from_6dof(dofs: (f64, f64, f64, f64, f64, f64)) -> Self {
        let pt = Point {
            x: dofs.0,
            y: dofs.1,
            z: dofs.2,
        };
        let angles = [dofs.3, dofs.4, dofs.5];
        let q = quat::from_euler_angles(Extrinsic, XYZ, angles);
        Self {
            position: pt,
            orientation: q.into(),
        }
    }

    /// Generate a tuple of 6 DOFs:
    /// (x, y, z, roll, pitch, yaw).
    /// Angles are in radians
    pub fn to_6dof(self) -> (f64, f64, f64, f64, f64, f64) {
        let pt = self.position;
        let q = self.orientation;
        let [roll, pitch, yaw] = quat::to_euler_angles::<f64>(Extrinsic, XYZ, q.into());

        (pt.x, pt.y, pt.z, roll, pitch, yaw)
    }
}

/// Analog to geometry_msgs/msg/PoseStamped in ROS.
/// Combines a Header with a Pose, similar to ROS' geometry_msgs/msg/PoseStamped.
#[derive(Debug, Clone, PartialEq)]
pub struct PoseStamped {
    pub header: Header,
    pub pose: Pose,
}

impl PoseStamped {
    /// Creates a new PoseStamped from a Header and a Pose.
    pub fn new(header: Header, pose: Pose) -> Self {
        Self { header, pose }
    }
    /// Creates a new PoseStamped from a Header, Point, and Quaternion.
    pub fn from_hpo(header: Header, position: Point, orientation: Quaternion) -> Self {
        Self {
            header,
            pose: Pose {
                position,
                orientation,
            },
        }
    }
}

impl Into<Pose> for PoseStamped {
    fn into(self) -> Pose {
        self.pose
    }
}

// ODOMETRY

/// Analog to geometry_msgs/msg/Twist in ROS
#[derive(Debug, Clone, PartialEq)]
pub struct Twist {
    pub linear: Vector3,
    pub angular: Vector3,
}

/// Analog to geometry_msgs/msg/Vector3 in ROS
#[derive(Debug, Clone, PartialEq)]
pub struct Vector3 {
    x: f64,
    y: f64,
    z: f64,
}

impl Vector3 {
    pub fn new(x: f64, y: f64, z: f64) -> Self {
        Self { x, y, z }
    }
}

/// Analog to geometry_msgs/msg/PoseWithCovariance in ROS
#[derive(Debug, Clone, PartialEq)]
pub struct PoseWithCovariance {
    pub pose: Pose,
    /// Row-major representation of the 6x6 covariance matrix
    /// The orientation parameters use a fixed-axis representation.
    /// In order, the parameters are:
    /// (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
    pub covariance: [f64; 36],
}

/// Analog to geometry_msgs/msg/TwistWithCovariance in ROS
#[derive(Debug, Clone, PartialEq)]
pub struct TwistWithCovariance {
    pub twist: Twist,
    /// Row-major representation of the 6x6 covariance matrix
    /// The orientation parameters use a fixed-axis representation.
    /// In order, the parameters are:
    /// (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
    pub covariance: [f64; 36],
}
/// Analog to nav_msgs/msg/Odometry in ROS
#[derive(Debug, Clone, PartialEq)]
pub struct Odometry {
    pub header: Header,
    pub child_frame_id: String,
    pub pose: PoseWithCovariance,
    pub twist: TwistWithCovariance,
}

// PATH
/// Analog to nav_msgs/msg/Path in ROS
#[derive(Debug, Clone, PartialEq)]
pub struct Path {
    pub header: Header,
    pub poses: Vec<Pose>,
}

impl Path {
    pub fn len(&self) -> usize {
        self.poses.len()
    }
}

impl From<Vec<PoseStamped>> for Path {
    fn from(value: Vec<PoseStamped>) -> Self {
        let header = value[0].header.clone();
        let poses: Vec<Pose> = value.iter().map(|p| p.clone().into()).collect();
        Self { header, poses }
    }
}

/// Round a f64 to the nth decimal place
pub fn round(num: f64, n: u32) -> f64 {
    let factor: f64 = 10_f64.powf(n.into());
    (num * factor).round() / factor
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::Duration;

    fn quaternion_data() -> [Quaternion; 4] {
        let q1 = Quaternion {
            w: 0.0,
            x: 0.0,
            y: 0.0,
            z: 1.0,
        };
        let q2: Quaternion = Quaternion {
            x: 3.0,
            y: 4.0,
            z: 5.0,
            w: 2.0,
        };
        let q3 = Quaternion {
            w: f64::sqrt(2.0) / 2.0,
            x: 0.0,
            y: 0.0,
            z: f64::sqrt(2.0) / 2.0,
        };
        let q4 = Quaternion {
            w: f64::sqrt(2.0) / 2.0,
            x: 0.0,
            y: f64::sqrt(2.0) / 2.0,
            z: 0.0,
        };

        [q1, q2, q3, q4]
    }

    #[test]
    /// Ensures that converting between Duration and Time is consistent.
    fn test_duration_time_conversion() {
        let duration = Duration::new(10, 250_000_000);
        let time: Time = duration.into();
        let converted_duration: Duration = time.into();

        assert_eq!(duration, converted_duration);
    }

    #[test]
    fn test_point_creation() {
        let point_constructor = Point {
            x: 1.0,
            y: 2.0,
            z: 4.0,
        };
        let point_tuple = Point::from_tuple((1.0, 2.0, 4.0));

        assert_eq!(point_constructor.x, 1.0);
        assert_eq!(point_constructor.y, 2.0);
        assert_eq!(point_constructor.z, 4.0);

        assert_eq!(point_tuple.x, 1.0);
        assert_eq!(point_tuple.y, 2.0);
        assert_eq!(point_tuple.z, 4.0);

        assert_eq!(point_constructor, point_tuple);
    }

    #[test]
    fn test_quaternion_creation() {
        let q_constructor = Quaternion {
            x: 1.0,
            y: 2.0,
            z: 4.0,
            w: 5.0,
        };
        let q_tuple = Quaternion::from_tuple((1.0, 2.0, 4.0, 5.0));

        assert_eq!(q_constructor.x, 1.0);
        assert_eq!(q_constructor.y, 2.0);
        assert_eq!(q_constructor.z, 4.0);
        assert_eq!(q_constructor.w, 5.0);

        assert_eq!(q_tuple.x, 1.0);
        assert_eq!(q_tuple.y, 2.0);
        assert_eq!(q_tuple.z, 4.0);
        assert_eq!(q_tuple.w, 5.0);

        assert_eq!(q_constructor, q_tuple);
    }

    #[test]
    fn test_square_len_and_norm() {
        let q = Quaternion::new(1.0, 2.0, 2.0, 1.0);
        assert_eq!(q.square_len(), 10.0);
        assert!((q.norm() - 10.0_f64.sqrt()).abs() < f64::EPSILON);
    }

    #[test]
    fn test_conjugate() {
        let q = Quaternion::new(1.0, -2.0, 3.0, -4.0);
        let conj = q.conjugate();
        assert_eq!(conj, Quaternion::new(-1.0, 2.0, -3.0, -4.0));
    }

    #[test]
    fn test_normalized() {
        let q = Quaternion::new(0.0, 3.0, 0.0, 4.0);
        let normed = q.normalized();
        assert!((normed.norm() - 1.0).abs() < f64::EPSILON);
        assert!((normed.x - 0.0).abs() < f64::EPSILON);
        assert!((normed.y - 0.6).abs() < f64::EPSILON);
        assert!((normed.z - 0.0).abs() < f64::EPSILON);
        assert!((normed.w - 0.8).abs() < f64::EPSILON);
    }

    #[test]
    #[should_panic(expected = "Cannot normalize a quaternion with zero norm")]
    fn test_normalized_zero() {
        let q = Quaternion::new(0.0, 0.0, 0.0, 0.0);
        let _ = q.normalized(); // should panic
    }

    #[test]
    fn test_quaternion_conversion() {
        let [q1, q2, q3, q4] = quaternion_data();

        let q1_vec: quat::Quaternion<f64> = q1.into();
        let q1_convert = Quaternion::from(q1_vec);
        assert_eq!(q1, q1_convert);

        let q2_vec: quat::Quaternion<f64> = q2.into();
        let q2_convert = Quaternion::from(q2_vec);
        assert_eq!(q2, q2_convert);

        let q3_vec: quat::Quaternion<f64> = q3.into();
        let q3_convert = Quaternion::from(q3_vec);
        assert_eq!(q3, q3_convert);

        let q4_vec: quat::Quaternion<f64> = q4.into();
        let q4_convert = Quaternion::from(q4_vec);
        assert_eq!(q4, q4_convert);
    }

    #[test]
    fn test_quaternion_multiplication() {
        let [_, q2, _, _] = quaternion_data();

        let q2_scaled = Quaternion {
            w: 4.0,
            x: 6.0,
            y: 8.0,
            z: 10.0,
        };

        assert_eq!(q2 * 2.0, q2_scaled);
    }

    #[test]
    fn test_path_len() {
        let path = Path {
            header: Header {
                frame_id: String::from("Coucou"),
                seq: 0,
                stamp: Duration::from_secs(0).into(),
            },
            poses: [Pose::from_6dof((0.0, 0.0, 0.0, 0.0, 0.0, 0.0)); 6].to_vec(),
        };

        assert_eq!(path.len(), 6);
    }
}
