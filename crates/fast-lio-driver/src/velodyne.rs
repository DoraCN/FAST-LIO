//! Velodyne driver adapter (spinning LiDAR).
//!
//! # Contract
//!
//! Implement [`DataSource`] for a `VelodyneSource` that:
//! 1. Binds a UDP socket to the Velodyne data port (default `2368`) and
//!    parses the VLP-16/32E packet layout (1248-byte packets, 12 data blocks,
//!    32 points per block, 100 µs per block).
//! 2. Accumulates points within one scan period into a
//!    [`StandardMsg`](fast_lio::types::StandardMsg):
//!    `x/y/z` (m), `intensity`, `time` (per-point offset in the unit declared
//!    by `timestamp_unit`), `ring` (the laser id 0..N_SCANS).
//! 3. Feeds the host IMU as
//!    [`ImuRaw`](fast_lio::types::ImuRaw) samples on the same clock.
//!
//! TODO: implement UDP parsing + frame accumulation.

#![allow(dead_code)]

use fast_lio::data_source::DataSource;
use fast_lio::types::SensorData;

/// Velodyne UDP data source (placeholder until implemented).
pub struct VelodyneSource {
    // udp: UdpSocket,
    // scan_period: Duration,
    // frame: FrameAcc,
}

impl DataSource for VelodyneSource {
    fn next(&mut self) -> Option<SensorData> {
        None
    }
}
