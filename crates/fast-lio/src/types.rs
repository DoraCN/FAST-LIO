//! Core data types: point cloud, lidar type enums, raw sensor messages and
//! the measurement group used by the pipeline. Ported from
//! `fast_lio/include/common_lib.h`, `preprocess.h` and ROS message layouts.

/// Point cloud point type (pcl::PointXYZINormal equivalent).
///
/// `curvature` is reused as the per-point time offset in milliseconds, exactly
/// as in the C++ implementation.
#[derive(Clone, Copy, Debug)]
pub struct PointType {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub intensity: f32,
    pub curvature: f32,
    pub normal_x: f32,
    pub normal_y: f32,
    pub normal_z: f32,
}

impl Default for PointType {
    fn default() -> Self {
        Self {
            x: 0.0,
            y: 0.0,
            z: 0.0,
            intensity: 0.0,
            curvature: 0.0,
            normal_x: 0.0,
            normal_y: 0.0,
            normal_z: 0.0,
        }
    }
}

impl PointType {
    pub fn new(x: f32, y: f32, z: f32) -> Self {
        Self {
            x,
            y,
            z,
            ..Default::default()
        }
    }
}

/// Point cloud (= `pcl::PointCloud<PointType>`).
pub type PointCloud = Vec<PointType>;

/// Lidar type enum (`LID_TYPE` in preprocess.h).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LidarType {
    Avia = 1,
    Velo16 = 2,
    Oust64 = 3,
    Marsim = 4,
}

impl LidarType {
    pub fn from_i32(v: i32) -> Self {
        match v {
            2 => LidarType::Velo16,
            3 => LidarType::Oust64,
            4 => LidarType::Marsim,
            _ => LidarType::Avia,
        }
    }
}

/// Timestamp unit enum (`TIME_UNIT` in preprocess.h).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TimeUnit {
    Sec = 0,
    Ms = 1,
    Us = 2,
    Ns = 3,
}

impl TimeUnit {
    /// Scale that converts a raw time field into milliseconds.
    pub fn to_ms_scale(&self) -> f32 {
        match self {
            TimeUnit::Sec => 1e3,
            TimeUnit::Ms => 1.0,
            TimeUnit::Us => 1e-3,
            TimeUnit::Ns => 1e-6,
        }
    }
}

/// A single IMU measurement.
#[derive(Clone, Copy, Debug)]
pub struct ImuRaw {
    pub stamp: f64,
    pub acc: [f64; 3],
    pub gyr: [f64; 3],
}

/// Livox Avia custom message point (`livox_ros_driver::CustomMsg`).
#[derive(Clone, Copy, Debug)]
pub struct AviaPointMsg {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub reflectivity: u8,
    pub tag: u16,
    pub line: u8,
    pub offset_time: u32,
}

/// Livox Avia message.
#[derive(Clone, Debug)]
pub struct AviaMsg {
    pub stamp: f64,
    pub points: Vec<AviaPointMsg>,
}

/// Spinning-lidar point (velodyne / ouster / marsim): the fields actually used
/// by the preprocessor are `x/y/z`, `intensity`, `time` and `ring`.
#[derive(Clone, Copy, Debug)]
pub struct StdPointMsg {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub intensity: f32,
    pub time: f32,
    pub ring: u16,
}

/// PointCloud2-like message for spinning lidars.
#[derive(Clone, Debug)]
pub struct StandardMsg {
    pub stamp: f64,
    pub points: Vec<StdPointMsg>,
}

/// One raw sensor sample from any source.
#[derive(Clone, Debug)]
pub enum SensorData {
    Imu(ImuRaw),
    LidarAvia(AviaMsg),
    LidarStandard(StandardMsg),
}

/// Feature type enum (`Feature` in preprocess.h).
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum Feature {
    Nor = 0,
    PossPlane = 1,
    RealPlane = 2,
    EdgeJump = 3,
    EdgePlane = 4,
    Wire = 5,
    ZeroPoint = 6,
}

/// Edge jump enum (`E_jump` in preprocess.h).
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum Ejump {
    NrNor = 0,
    NrZero = 1,
    Nr180 = 2,
    NrInf = 3,
    NrBlind = 4,
}

/// Per-point feature extraction bookkeeping (`orgtype` in preprocess.h).
#[derive(Clone, Debug)]
pub struct OrgType {
    pub range: f64,
    pub dista: f64,
    pub angle: [f64; 2],
    pub intersect: f64,
    pub edj: [Ejump; 2],
    pub ftype: Feature,
}

impl Default for OrgType {
    fn default() -> Self {
        Self {
            range: 0.0,
            dista: 0.0,
            angle: [0.0; 2],
            intersect: 2.0,
            edj: [Ejump::NrNor, Ejump::NrNor],
            ftype: Feature::Nor,
        }
    }
}

/// Surround direction enum (`Surround` in preprocess.h).
pub const SURROUND_PREV: usize = 0;
pub const SURROUND_NEXT: usize = 1;

/// A synchronized group of lidar scan + the IMU measurements covering it.
#[derive(Clone, Debug)]
pub struct MeasureGroup {
    pub lidar_beg_time: f64,
    pub lidar_end_time: f64,
    pub lidar: PointCloud,
    pub imu: Vec<ImuRaw>,
}

impl Default for MeasureGroup {
    fn default() -> Self {
        Self {
            lidar_beg_time: 0.0,
            lidar_end_time: 0.0,
            lidar: Vec::new(),
            imu: Vec::new(),
        }
    }
}

/// Global constants (common_lib.h / laserMapping.cpp).
pub mod consts {
    /// Gravity magnitude used for IMU init (m/s²).
    pub const G_M_S2: f64 = 9.81;
    /// Minimum points for plane matching.
    pub const NUM_MATCH_POINTS: usize = 5;
    /// Laser point covariance used as the measurement noise R.
    pub const LASER_POINT_COV: f64 = 0.001;
    /// EKF initial-time threshold (seconds).
    pub const INIT_TIME: f64 = 0.1;
    /// Lidar-to-IMU lever arm magnitude used for the FOV axis point.
    pub const LIDAR_SP_LEN: f32 = 2.0;
}
