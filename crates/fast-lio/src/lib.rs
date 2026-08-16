//! FAST-LIO core algorithm library.
//!
//! Rust port of the C++ `fast_lio` (FAST-LIO2) package. This crate contains the
//! pure algorithm front-end: preprocessing, IMU propagation / undistortion, the
//! iterated error-state Kalman filter (esekfom), the incremental ikd-Tree map and
//! the laser-mapping main loop. It is deliberately free of any IO / ROS dependency
//! so it can be reused by downstream crates (`lidar-map`, `lidar-nav`) and driven
//! by any data source.

pub mod data_source;
pub mod esekf;
pub mod ikdtree;
pub mod imu_processing;
#[cfg(feature = "live")]
pub mod livox;
pub mod laser_mapping;
pub mod math;
pub mod model;
pub mod preprocess;
pub mod types;

pub use types::consts;
