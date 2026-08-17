# fast-lio

Pure-Rust, dependency-light port of [FAST-LIO2](https://github.com/hku-mars/FAST_LIO):
a tightly-coupled LiDAR-inertial odometry (LIO) system using an iterated
error-state Kalman filter (IEKF) on a manifold and an incremental k-d tree
(ikd-Tree) map.

This core crate contains **no I/O or ROS dependencies**: it consumes timestamped
IMU samples and LiDAR scans (see [`SensorData`](src/types.rs)) and produces
poses, velocity, biases and the local map. Drive it from any data source.

## Usage

```toml
[dependencies]
fast-lio = "0.1"
```

```rust
use fast_lio::laser_mapping::{LaserMapping, LioConfig};
use fast_lio::types::{LidarType, SensorData};

let cfg = LioConfig {
    lidar_type: LidarType::Velo16,
    ..Default::default()
};
let mut mapping = LaserMapping::new(&cfg);

// feed samples; run one LIO iteration when a synchronized frame is ready
while let Some(sample) = source.next() {
    match sample {
        SensorData::Imu(imu) => mapping.add_imu(&imu),
        SensorData::LidarAvia(msg) => mapping.add_lidar_avia(&msg),
        SensorData::LidarStandard(msg) => mapping.add_lidar_standard(&msg),
    }
    if mapping.has_data() {
        if let Some(res) = mapping.run_once() {
            println!("pose @ {:.3}s: {:?}", res.time, res.pos);
        }
    }
}
```

## Modules

- `math` — SO(3) / S² / manifold primitives
- `model` — process model and process-noise covariance
- `esekf` — iterated error-state Kalman filter
- `preprocess` — LiDAR handlers & feature extraction
- `imu_processing` — IMU init, propagation and undistortion
- `ikdtree` — incremental k-d tree map
- `laser_mapping` — main pipeline front-end (`LaserMapping`)
- `data_source` — `DataSource` trait + synthetic simulator
- `types` — normalized sensor messages

## License

MIT OR Apache-2.0. See the [repository](https://github.com/DoraCN/FAST-LIO)
for the full project documentation, configuration reference and real-device
usage.
