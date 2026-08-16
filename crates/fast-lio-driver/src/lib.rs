//! LiDAR + IMU **driver adapters**.
//!
//! The algorithm core (`fast-lio`) never talks to a vendor SDK: it consumes
//! normalized [`SensorData`]. This crate implements the per-brand adapters that
//! connect to real hardware and translate raw data into that normalized format.
//!
//! # Layout rule
//!
//! - Lightweight adapters (pure-Rust UDP parsing of spinning LiDARs, e.g.
//!   Velodyne / Ouster / Hesai) live as **one module per brand**.
//! - An adapter that must compile a heavy vendor C/C++ SDK (currently Livox
//!   SDK2) is **feature-gated** ([`livox`] behind `feature = "livox-sdk2"`).
//!
//! # How to add a new brand
//!
//! 1. Create `src/<brand>.rs` implementing [`DataSource`]: convert the vendor's
//!    point packets into a [`StandardMsg`] (spinning) or [`AviaMsg`]
//!    (non-repetitive scanning) plus [`ImuRaw`] samples.
//! 2. Add a match arm in [`open`].
//! 3. The algorithm core needs no changes.

pub mod hesai;
#[cfg(feature = "livox-sdk2")]
pub mod livox;
pub mod ouster;
pub mod velodyne;

use std::time::Duration;

pub use fast_lio::data_source::DataSource;
pub use fast_lio::types::{LidarType, SensorData};

/// Parameters used by [`open`] to construct a driver.
///
/// Per-driver fields are optional; each adapter reads only what it needs:
/// - [`LidarType::Avia`] → Livox (needs `config_path` = Livox JSON config)
/// - spinning LiDARs (Velo16 / Oust64 / Marsim) → need `udp_ip` / port (TBD)
#[derive(Clone, Debug)]
pub struct DriverParams {
    /// Which driver to construct.
    pub lidar_type: LidarType,
    /// Livox SDK2 config file (for Livox devices).
    pub config_path: Option<String>,
    /// UDP address of a spinning LiDAR (for future drivers).
    pub udp_ip: Option<String>,
    /// Grouping period for one lidar scan frame.
    pub scan_period: Duration,
}

impl DriverParams {
    pub fn livox(config_path: impl Into<String>, scan_period: Duration) -> Self {
        Self {
            lidar_type: LidarType::Avia,
            config_path: Some(config_path.into()),
            udp_ip: None,
            scan_period,
        }
    }
}

/// Construct a [`DataSource`] for the requested LiDAR.
///
/// Returns an error for drivers that are not yet implemented, so the contract
/// stays explicit until each adapter lands.
pub fn open(params: &DriverParams) -> Result<Box<dyn DataSource>, String> {
    match params.lidar_type {
        LidarType::Avia => {
            #[cfg(feature = "livox-sdk2")]
            {
                let config = params
                    .config_path
                    .as_deref()
                    .ok_or("Livox driver requires a config file path")?;
                livox::LivoxSource::connect(config, params.scan_period)
                    .map(|s| Box::new(s) as Box<dyn DataSource>)
            }
            #[cfg(not(feature = "livox-sdk2"))]
            {
                let _ = params;
                Err("built without the `livox-sdk2` feature".to_string())
            }
        }
        LidarType::Velo16 => Err("velodyne driver not yet implemented".to_string()),
        LidarType::Oust64 => Err("ouster driver not yet implemented".to_string()),
        LidarType::Marsim => Err("marsim driver not yet implemented".to_string()),
    }
}
