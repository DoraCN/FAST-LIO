//! Ouster driver adapter (spinning LiDAR).
//!
//! # Contract
//!
//! Implement [`DataSource`] for an `OusterSource` that:
//! 1. Connects to the Ouster sensor (default UDP data port `7502`), or reads
//!    from the TCP config port, and parses the Ouster LIDAR_DATA packets.
//! 2. Accumulates points within one scan period into a
//!    [`StandardMsg`](fast_lio::types::StandardMsg): `x/y/z` (m), `intensity`,
//!    `time` (per-point offset, scaled by `timestamp_unit`), `ring` (0..N_SCANS).
//!    Note Ouster points carry a `range` that must be converted to Cartesian
//!    using the per-column beam-to-azimuth/elevation tables from the metadata.
//! 3. Feeds the host IMU as
//!    [`ImuRaw`](fast_lio::types::ImuRaw) samples.
//!
//! TODO: implement metadata fetch + packet parsing.

#![allow(dead_code)]

use fast_lio::data_source::DataSource;
use fast_lio::types::SensorData;

/// Ouster UDP data source (placeholder until implemented).
pub struct OusterSource {
    // udp: UdpSocket,
    // scan_period: Duration,
    // frame: FrameAcc,
}

impl DataSource for OusterSource {
    fn next(&mut self) -> Option<SensorData> {
        None
    }
}
