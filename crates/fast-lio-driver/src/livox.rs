//! Real-time Livox data source for use **without ROS**, connecting directly to
//! the LiDAR through the Livox SDK2 via the [`livox-sdk2`] crate (HAP / Mid-360
//! devices). Point clouds and the built-in IMU are received through SDK
//! callbacks, grouped into scan frames and pushed into the pipeline as
//! [`SensorData`] samples, exactly like the offline [`DataSource`]s.
//!
//! # Units
//!
//! - SDK2 points are already in Cartesian meters.
//! - SDK2 IMU gyro is in rad/s; the accel is delivered in **g** (per the
//!   `livox-sdk2` documentation) and converted to m/s² using
//!   [`ACC_G_TO_MPS2`]. If a future device/firmware reports m/s² directly,
//!   set that constant to `1.0`.
//!
//! # Device requirements
//!
//! - A Livox config file (e.g. `mid360_config.json`, the same file the official
//!   Livox Viewer / driver2 uses) that lists the device IP / subnet.
//! - The target machine must be able to build the vendored SDK2 C++ sources
//!   (cmake + a C++ compiler), and be on the same network as the LiDAR.

#![cfg(feature = "livox-sdk2")]

use std::sync::mpsc;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use livox_sdk2::{Packet, Sdk};

use fast_lio::data_source::DataSource;
use fast_lio::types::{AviaMsg, AviaPointMsg, ImuRaw, SensorData};

/// Convert the SDK2 IMU accel (g) into m/s². Set to `1.0` if the device
/// reports m/s² directly.
pub const ACC_G_TO_MPS2: f64 = 9.80665;

/// Shared clock + per-frame point accumulation used by the SDK callbacks.
struct FrameAcc {
    scan_period: Duration,
    frame_start: Instant,
    /// Local epoch used to convert `Instant` to f64 seconds on one clock.
    t0: Instant,
    unix0: f64,
    points: Vec<AviaPointMsg>,
}

impl FrameAcc {
    fn new(scan_period: Duration) -> Self {
        let t0 = Instant::now();
        let unix0 = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .map(|d| d.as_secs_f64())
            .unwrap_or(0.0);
        Self {
            scan_period,
            frame_start: t0,
            t0,
            unix0,
            points: Vec::new(),
        }
    }

    /// Convert an instant to f64 seconds on the shared clock.
    fn epoch(&self, t: Instant) -> f64 {
        self.unix0 + t.duration_since(self.t0).as_secs_f64()
    }

    fn ready(&self, now: Instant) -> bool {
        now.duration_since(self.frame_start) >= self.scan_period
    }

    fn take(&mut self, now: Instant) -> AviaMsg {
        let stamp = self.epoch(self.frame_start);
        let msg = AviaMsg {
            stamp,
            points: std::mem::take(&mut self.points),
        };
        self.frame_start = now;
        msg
    }
}

/// A live `DataSource` that streams data from a Livox LiDAR over the network.
pub struct LivoxSource {
    rx: mpsc::Receiver<SensorData>,
}

impl LivoxSource {
    /// Connect to the device(s) described by `config_path` and start streaming.
    ///
    /// `scan_period` groups the raw point packets into one lidar frame per
    /// period (e.g. `100 ms` for a 10 Hz scan).
    pub fn connect(config_path: &str, scan_period: Duration) -> Result<Self, String> {
        let mut sdk = Sdk::new(config_path)?;
        let (tx, rx) = mpsc::channel();
        let frame = Arc::new(Mutex::new(FrameAcc::new(scan_period)));

        // Log device discovery for diagnostics.
        sdk.set_device_change_callback(move |dev| {
            println!(
                "Livox device: {} @ {} (SN {})",
                dev.type_name(),
                dev.lidar_ip,
                dev.sn
            );
        });

        // Point-cloud callback: accumulate packets into scan frames.
        let pc_frame = frame.clone();
        let pc_tx = tx.clone();
        sdk.set_point_cloud_callback(move |_handle, _dev_type, packet: Packet<'_>| {
            let pts = packet.points();
            if pts.is_empty() {
                return;
            }
            let now = Instant::now();
            let mut acc = pc_frame.lock().unwrap();
            let base_us = now.duration_since(acc.frame_start).as_micros() as u32;
            // intra-packet spacing in µs (time_interval is in 0.1 µs units)
            let interval_us = (packet.time_interval() as u64) / 10;
            for (k, p) in pts.iter().enumerate() {
                // Livox marks invalid / blind-spot returns with reflectivity 0xFF
                if p.reflectivity == 0xFF {
                    continue;
                }
                let off = base_us.saturating_add(interval_us as u32 * k as u32);
                acc.points.push(AviaPointMsg {
                    x: p.x as f32,
                    y: p.y as f32,
                    z: p.z as f32,
                    reflectivity: p.reflectivity,
                    tag: p.tag as u16,
                    // direct odometry mode only: no per-point ring in SDK2,
                    // so route all points through scan line 0
                    line: 0,
                    offset_time: off,
                });
            }
            if acc.ready(now) {
                let msg = acc.take(now);
                let _ = pc_tx.send(SensorData::LidarAvia(msg));
            }
        });

        // IMU callback: convert g -> m/s² and push samples.
        let imu_frame = frame.clone();
        let imu_tx = tx;
        sdk.set_imu_callback(move |_handle, _dev_type, packet: Packet<'_>| {
            let now = Instant::now();
            let stamp = imu_frame.lock().unwrap().epoch(now);
            for imu in packet.imu_points() {
                let _ = imu_tx.send(SensorData::Imu(ImuRaw {
                    stamp,
                    acc: [
                        imu.acc_x as f64 * ACC_G_TO_MPS2,
                        imu.acc_y as f64 * ACC_G_TO_MPS2,
                        imu.acc_z as f64 * ACC_G_TO_MPS2,
                    ],
                    gyr: [imu.gyro_x as f64, imu.gyro_y as f64, imu.gyro_z as f64],
                }));
            }
        });

        // The SDK runs its own receive loop; it must live in a thread because
        // `Sdk::run` blocks forever.
        std::thread::Builder::new()
            .name("livox-sdk".to_string())
            .spawn(move || sdk.run())
            .map_err(|e| format!("failed to spawn SDK thread: {e}"))?;

        Ok(Self { rx })
    }
}

impl DataSource for LivoxSource {
    fn next(&mut self) -> Option<SensorData> {
        self.rx.recv().ok()
    }
}
