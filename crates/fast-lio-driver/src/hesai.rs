//! Hesai (Pandar / QT / AT) driver adapter (spinning LiDAR).
//!
//! # Contract
//!
//! Implement [`DataSource`] for a `HesaiSource` that:
//! 1. Binds a UDP socket to the Hesai data port (e.g. `2369`) and parses the
//!    Pandar packet layout (per-packet lidar + IMU blocks).
//! 2. Accumulates points within one scan period into a
//!    [`StandardMsg`](fast_lio::types::StandardMsg): `x/y/z` (m), `intensity`,
//!    `time`, `ring` (0..N_SCANS).
//! 3. Hesai packets carry **onboard IMU** samples inside the same datagram —
//!    feed them as [`ImuRaw`](fast_lio::types::ImuRaw) with the gyro in rad/s
//!    and accel in m/s².
//!
//! TODO: implement packet parsing.

#![allow(dead_code)]

use fast_lio::data_source::DataSource;
use fast_lio::types::SensorData;

/// Hesai UDP data source (placeholder until implemented).
pub struct HesaiSource {
    // udp: UdpSocket,
    // scan_period: Duration,
    // frame: FrameAcc,
}

impl DataSource for HesaiSource {
    fn next(&mut self) -> Option<SensorData> {
        None
    }
}
