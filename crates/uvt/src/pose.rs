use std::{
    ops::{Add, Div, Mul, Sub},
    time::Duration,
};

use quaternion;

// HEADER

/// Analog to builtin_interfaces/msg/Time in ROS
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Time {
    sec: i32,
    nanosec: u32,
}

impl From<Duration> for Time {
    fn from(duration: Duration) -> Self {
        Time {
            sec: duration.as_secs() as i32,
            nanosec: duration.subsec_nanos(),
        }
    }
}

impl From<Time> for Duration {
    fn from(time: Time) -> Self {
        Duration::new(time.sec as u64, time.nanosec)
    }
}

/// Analog to std_msgs/msg/Header in ROS
#[derive(Debug, Clone, PartialEq)]
pub struct Header {
    pub seq: u32,
    pub stamp: Time,
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
    pub fn new(x: f64, y: f64, z: f64) -> Self {
        Self { x, y, z }
    }
    fn from_tuple(tup: (f64, f64, f64)) -> Self {
        Self {
            x: tup.0,
            y: tup.1,
            z: tup.2,
        }
    }
}

/// Analog to geometry_msgs/msg/Quaternion in ROS
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Quaternion {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub w: f64,
}

impl From<quaternion::Quaternion<f64>> for Quaternion {
    fn from(q: quaternion::Quaternion<f64>) -> Self {
        Self {
            w: q.0,
            x: q.1[0],
            y: q.1[1],
            z: q.1[2],
        }
    }
}

impl Into<quaternion::Quaternion<f64>> for Quaternion {
    fn into(self) -> quaternion::Quaternion<f64> {
        (self.w, [self.x, self.y, self.z])
    }
}

impl Quaternion {
    pub fn new(x: f64, y: f64, z: f64, w: f64) -> Self {
        Self { x, y, z, w }
    }
    pub fn from_tuple(tup: (f64, f64, f64, f64)) -> Self {
        Self {
            x: tup.0,
            y: tup.1,
            z: tup.2,
            w: tup.3,
        }
    }
    fn square_len(&self) -> f64 {
        self.w * self.w + self.x * self.x + self.y * self.y + self.z * self.z
    }
    fn norm(&self) -> f64 {
        self.square_len().sqrt()
    }
    fn conjugate(&self) -> Self {
        Self {
            w: -self.w,
            x: self.x,
            y: self.y,
            z: self.z,
        }
    }
    fn unit(&self) -> Self {
        self.clone() * (1.0 / self.norm())
    }
}

impl Add for Quaternion {
    type Output = Self;

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
    pub position: Point,
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
        let q = quaternion::euler_angles(dofs.3, dofs.4, dofs.5);
        Self {
            position: pt,
            orientation: q.into(),
        }
    }
}

// Analog to geometry_msgs/msg/PoseStamped in ROS
#[derive(Debug, Clone, PartialEq)]
pub struct PoseStamped {
    pub header: Header,
    pub pose: Pose,
}

impl PoseStamped {
    pub fn new(header: Header, pose: Pose) -> Self {
        Self { header, pose }
    }
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

// Analog to nav_msgs/msg/Path in ROS
#[derive(Debug, Clone, PartialEq)]
pub struct Path {
    pub header: Header,
    pub poses: Vec<Pose>,
}

impl Path {
    fn len(&self) -> usize {
        self.poses.len()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::Duration;

    static EPSILON: f32 = 0.000001;

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
    fn convert_duration_to_time() {
        let duration = Duration::new(10, 250_000_000);
        let time: Time = duration.into();
        let converted_duration: Duration = time.into();

        assert_eq!(duration, converted_duration);
    }

    #[test]
    fn create_point() {
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
    fn create_quaternion() {
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
    fn convert_quaternion() {
        let [q1, q2, q3, q4] = quaternion_data();

        let q1_vec: quaternion::Quaternion<f64> = q1.into();
        let q1_convert = Quaternion::from(q1_vec);
        assert_eq!(q1, q1_convert);

        let q2_vec: quaternion::Quaternion<f64> = q2.into();
        let q2_convert = Quaternion::from(q2_vec);
        assert_eq!(q2, q2_convert);

        let q3_vec: quaternion::Quaternion<f64> = q3.into();
        let q3_convert = Quaternion::from(q3_vec);
        assert_eq!(q3, q3_convert);

        let q4_vec: quaternion::Quaternion<f64> = q4.into();
        let q4_convert = Quaternion::from(q4_vec);
        assert_eq!(q4, q4_convert);
    }

    #[test]
    fn quaternion_multiplication() {
        let [q1, q2, q3, q4] = quaternion_data();

        let q2_scaled = Quaternion {
            w: 4.0,
            x: 6.0,
            y: 8.0,
            z: 10.0,
        };

        assert_eq!(q2 * 2.0, q2_scaled);
    }
}
