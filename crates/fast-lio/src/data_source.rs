//! Offline data sources for driving the FAST-LIO pipeline.
//!
//! Two sources are provided:
//! - `SimSource`: a synthetic LiDAR + IMU generator for a circular trajectory,
//!   used to exercise the whole pipeline end-to-end without sensor hardware.
//! - the `DataSource` trait that any file / bag reader can implement later.

use nalgebra::UnitQuaternion;

use crate::math::so3::{exp, V3D};
use crate::types::{ImuRaw, SensorData, StandardMsg, StdPointMsg};

/// A data source yields time-ordered raw sensor samples.
pub trait DataSource {
    /// Blocking read of the next sample (`None` = end of stream).
    fn next(&mut self) -> Option<SensorData>;

    /// Non-blocking read of the next sample. Returns:
    /// - `Some(sample)` if one is immediately available,
    /// - `None` if the stream has ended,
    /// - `Err(NonBlocking)` if nothing is available right now.
    ///
    /// The default implementation falls back to a blocking `next()`. Live
    /// sources override it with `try_recv` so a driver loop can poll for
    /// Ctrl-C / timeout while streaming.
    fn try_next(&mut self) -> Result<Option<SensorData>, NonBlocking> {
        Ok(self.next())
    }
}

/// Marker for [`DataSource::try_next`] when no sample is available yet.
pub struct NonBlocking;

/// Parameters of the synthetic scenario.
#[derive(Clone, Debug)]
pub struct SimParams {
    /// IMU rate in Hz.
    pub imu_hz: f64,
    /// Lidar rate in Hz.
    pub lidar_hz: f64,
    /// Total duration in seconds.
    pub duration: f64,
    /// Circle radius (m).
    pub radius: f64,
    /// Angular speed (rad/s).
    pub omega: f64,
    /// Height of the robot (m).
    pub height: f64,
    /// Number of lidar points per frame.
    pub points_per_scan: usize,
    /// Duration of the static phase before motion starts (lets the IMU
    /// initializer converge on a near-zero gyro bias, as in practice).
    pub init_static: f64,
    /// Noise on IMU acc (m/s²).
    pub acc_noise: f64,
    /// Noise on IMU gyro (rad/s).
    pub gyr_noise: f64,
}

impl Default for SimParams {
    fn default() -> Self {
        Self {
            imu_hz: 200.0,
            lidar_hz: 10.0,
            duration: 20.0,
            radius: 5.0,
            omega: 0.15,
            height: 1.0,
            points_per_scan: 1200,
            init_static: 1.0,
            acc_noise: 0.02,
            gyr_noise: 0.002,
        }
    }
}

/// Deterministic pseudo-random generator (xorshift), so runs are reproducible.
struct Rng(u64);
impl Rng {
    fn new(seed: u64) -> Self {
        Self(seed)
    }
    fn next_f64(&mut self) -> f64 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 7;
        self.0 ^= self.0 << 17;
        (self.0 as f64 / u64::MAX as f64) * 2.0 - 1.0
    }
}

/// Robot pose (pos + quaternion wxyz).
#[derive(Clone, Copy, Debug)]
pub struct SimPose {
    pub pos: V3D,
    pub rot: UnitQuaternion<f64>,
}

impl SimPose {
    fn at(t: f64, p: &SimParams) -> Self {
        if t < p.init_static {
            // stationary: pose fixed at the circle start, facing +y (tangent)
            return Self {
                pos: V3D::new(p.radius, 0.0, p.height),
                rot: UnitQuaternion::from_axis_angle(
                    &nalgebra::Unit::new_normalize(V3D::new(0.0, 0.0, 1.0)),
                    std::f64::consts::PI / 2.0,
                ),
            };
        }
        let tc = t - p.init_static;
        let yaw = p.omega * tc;
        let pos = V3D::new(p.radius * yaw.cos(), p.radius * yaw.sin(), p.height);
        // body x points along the tangent of the circle
        let rot = UnitQuaternion::from_axis_angle(
            &nalgebra::Unit::new_normalize(V3D::new(0.0, 0.0, 1.0)),
            yaw + std::f64::consts::PI / 2.0,
        );
        Self { pos, rot }
    }
}

/// Synthetic LiDAR + IMU data source.
pub struct SimSource {
    params: SimParams,
    rng: Rng,
    t: f64,
    dt_imu: f64,
    dt_lidar: f64,
    next_imu_t: f64,
    next_lidar_t: f64,
    world_points: Vec<V3D>,
}

impl SimSource {
    pub fn new(params: &SimParams) -> Self {
        let dt_imu = 1.0 / params.imu_hz;
        let dt_lidar = 1.0 / params.lidar_hz;
        Self {
            params: params.clone(),
            rng: Rng::new(0x5EED_CAFE),
            t: 0.0,
            dt_imu,
            dt_lidar,
            next_imu_t: 0.0,
            next_lidar_t: 0.0,
            world_points: build_world_points(),
        }
    }

    fn make_imu(&mut self, t: f64) -> ImuRaw {
        let pose = SimPose::at(t, &self.params);
        let p = &self.params;
        let (acc, gyr) = if t < p.init_static {
            // stationary: specific force exactly counteracts gravity, no rotation
            (
                V3D::new(0.0, 0.0, 9.81),
                V3D::zeros(),
            )
        } else {
            let tc = t - p.init_static;
            let yaw = p.omega * tc;
            // world acceleration of the circular motion (centripetal)
            let a_world = V3D::new(
                -p.omega * p.omega * p.radius * yaw.cos(),
                -p.omega * p.omega * p.radius * yaw.sin(),
                0.0,
            );
            let g = V3D::new(0.0, 0.0, -9.81);
            let acc = pose.rot.conjugate() * (a_world - g);
            (acc, V3D::new(0.0, 0.0, p.omega))
        };
        let acc = acc
            + V3D::new(
                self.rng.next_f64() * p.acc_noise,
                self.rng.next_f64() * p.acc_noise,
                self.rng.next_f64() * p.acc_noise,
            );
        let gyr = gyr
            + V3D::new(
                self.rng.next_f64() * p.gyr_noise,
                self.rng.next_f64() * p.gyr_noise,
                self.rng.next_f64() * p.gyr_noise,
            );
        ImuRaw {
            stamp: t,
            acc: [acc[0], acc[1], acc[2]],
            gyr: [gyr[0], gyr[1], gyr[2]],
        }
    }

    fn make_lidar(&mut self, t: f64) -> StandardMsg {
        let pose = SimPose::at(t, &self.params);
        let p = &self.params;
        let mut points = Vec::with_capacity(p.points_per_scan);
        // scan angle samples over the frame (simple linear sweep)
        let n = p.points_per_scan;
        for i in 0..n {
            let frac = i as f64 / n as f64;
            // pick a world point pseudo-randomly from the walls
            let wp = self.world_points[(i * 37) % self.world_points.len()];
            // body frame
            let pb = pose.rot.conjugate() * (wp - pose.pos);
            let range = pb.norm();
            // keep points within 40 m and forward-ish
            if range > 40.0 || pb[0] < 0.5 {
                continue;
            }
            let ring = (((pb[1].atan2(pb[0]) * 180.0 / std::f64::consts::PI + 90.0) / 6.0) as i32).clamp(0, 15) as u16;
            let time = (frac * 100.0) as f32; // ms within the frame
            points.push(StdPointMsg {
                x: (pb[0] + self.rng.next_f64() * 0.01) as f32,
                y: (pb[1] + self.rng.next_f64() * 0.01) as f32,
                z: (pb[2] + self.rng.next_f64() * 0.01) as f32,
                intensity: 100.0,
                time,
                ring,
            });
        }
        StandardMsg { stamp: t, points }
    }
}

impl DataSource for SimSource {
    fn next(&mut self) -> Option<SensorData> {
        if self.next_imu_t >= self.params.duration && self.next_lidar_t >= self.params.duration {
            return None;
        }
        // emit whichever event comes first (IMU at higher rate)
        let (t, tag) = if self.next_imu_t <= self.next_lidar_t {
            let t = self.next_imu_t;
            self.next_imu_t += self.dt_imu;
            (t, 0u8)
        } else {
            let t = self.next_lidar_t;
            self.next_lidar_t += self.dt_lidar;
            (t, 1u8)
        };
        let _ = self.t;
        if tag == 0 {
            Some(SensorData::Imu(self.make_imu(t)))
        } else {
            Some(SensorData::LidarStandard(self.make_lidar(t)))
        }
    }
}

/// Build a set of world points on a few flat walls and a ground plane.
fn build_world_points() -> Vec<V3D> {
    let mut pts = Vec::new();
    let step = 0.4;
    let half = 20.0;
    // ground plane z = 0
    let mut x = -half;
    while x <= half {
        let mut y = -half;
        while y <= half {
            pts.push(V3D::new(x, y, 0.0));
            y += step;
        }
        x += step;
    }
    // walls x = ±half and y = ±half, z in [0, 6]
    let mut x = -half;
    while x <= half {
        let mut z = 0.0;
        while z <= 6.0 {
            pts.push(V3D::new(x, -half, z));
            pts.push(V3D::new(x, half, z));
            pts.push(V3D::new(-half, x, z));
            pts.push(V3D::new(half, x, z));
            z += step;
        }
        x += step;
    }
    pts
}

/// Re-export for convenience.
pub fn exp_rot(v: &V3D) -> nalgebra::Matrix3<f64> {
    exp(v)
}
