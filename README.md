# fast-lio

[![license](https://img.shields.io/badge/license-MIT%20OR%20Apache--2.0-blue)](#license)
[![edition](https://img.shields.io/badge/edition-2024-orange)](Cargo.toml)

**fast-lio** is a pure-Rust, dependency-light port of [FAST-LIO2](https://github.com/hku-mars/FAST_LIO) — a computationally efficient, robust tightly-coupled LiDAR-inertial odometry (LIO) system. It fuses raw LiDAR points with IMU data using an iterated error-state Kalman filter (IEKF) on a manifold and maintains an incremental k-d tree (ikd-Tree) map, enabling accurate, drift-bounded odometry and mapping at high rates.

The core crate contains **no I/O or ROS dependencies**: the whole front-end is a plain library that consumes timestamped IMU samples and LiDAR scans and produces poses, velocity, biases and the local map. This makes it easy to embed, unit-test, and drive from any data source (rosbag, custom file format, live sensors).

## Table of contents

- [Features](#features)
- [Status](#status)
- [Workspace layout](#workspace-layout)
- [Quick start](#quick-start)
- [Library usage](#library-usage)
- [Configuration](#configuration)
- [Data sources](#data-sources)
- [Outputs](#outputs)
- [Testing](#testing)
- [Performance notes](#performance-notes)
- [Roadmap](#roadmap)
- [Attribution & license](#attribution--license)
- [References](#references)

## Features

- Full FAST-LIO2 pipeline in Rust:
  - **Preprocessing** — Livox Avia (`CustomMsg`-like), Velodyne, Ouster and MARSIM handlers, optional scan feature extraction (plane / edge classification).
  - **IMU processing** — automatic initialization (gravity via S² manifold, gyro/acc bias, covariance), forward propagation and per-point backward undistortion.
  - **IEKF (`esekfom`)** — iterated error-state Kalman filter on the 23-DOF manifold state `{pos, rot, offset_R_L_I, offset_T_L_I, vel, bg, ba, grav(S²)}`, including the FAST-LIO2 `update_iterated_dyn_share_modified` formulation with online extrinsic estimation.
  - **ikd-Tree** — incremental k-d tree with lazy deletion, box deletion, downsampled insertion, subtree rebalancing and O(log n) k-NN search.
  - **Mapping** — FOV-based sliding local map, voxel-grid downsampling and incremental map update.
- Zero IO in the core crate — `no_std`-friendly architecture (currently `std` only), no ROS, no PCL.
- **Decoupled driver layer** — all hardware access lives in the `fast-lio-driver` crate (one module per brand, heavy vendor SDKs feature-gated); the algorithm core never sees a vendor SDK.
- **Live Livox support (no ROS)** — `fast-lio-driver` connects directly to HAP / Mid-360 LiDARs through the official Livox SDK2 (via the `livox-sdk2` crate) and streams point clouds + the built-in IMU into the pipeline.
- Numerically faithful port, verified against the C++ implementation module by module (see [Testing](#testing)).
- Workspace-ready for the full autonomy stack: `lidar-map` (occupancy/voxel map) and `lidar-nav` (planning/navigation) crates are reserved.

## Status

| Module | File | Status |
|---|---|---|
| SO(3) / S² / manifold math | `src/math/{so3,s2,manifold}.rs` | ✅ tested |
| Process model (`f`, `f_x`, `f_w`, Q) | `src/model.rs` | ✅ numeric-diff verified |
| Iterated ESKF | `src/esekf.rs` | ✅ tested |
| Preprocessing (4 LiDAR types) | `src/preprocess.rs` | ✅ |
| IMU init / propagation / undistortion | `src/imu_processing.rs` | ✅ |
| ikd-Tree | `src/ikdtree.rs` | ✅ brute-force cross-checked |
| Laser-mapping main loop | `src/laser_mapping.rs` | ✅ end-to-end |
| Offline driver + synthetic data source | `crates/fast-lio-app` | ✅ end-to-end |
| Live Livox SDK2 source (`livox-sdk2` feature) | `crates/fast-lio-driver/src/livox.rs` | ✅ builds (needs hardware to verify) |
| Real-dataset validation (C++ golden comparison) | — | 🔜 pending dataset |
| `lidar-map` / `lidar-nav` | — | 🔜 planned |

## Workspace layout

```
fast-lio/
├── Cargo.toml                 # virtual workspace
└── crates/
    ├── fast-lio/              # core algorithm library (published as `fast-lio`)
    │   └── src/
    │       ├── math/          #   so3.rs · s2.rs · manifold.rs
    │       ├── model.rs       #   process model & process noise
    │       ├── esekf.rs       #   iterated error-state Kalman filter
    │       ├── preprocess.rs  #   LiDAR drivers & feature extraction
    │       ├── imu_processing.rs
    │       ├── ikdtree.rs     #   incremental k-d tree
    │       ├── laser_mapping.rs # main pipeline
    │       ├── data_source.rs #   DataSource trait + synthetic simulator
    │       └── types.rs       #   normalized SensorData messages
    ├── fast-lio-driver/       # device adapters (one module per brand)
    │   └── src/
    │       ├── lib.rs         #   DriverParams + open() factory
    │       ├── livox.rs       #   Livox SDK2 (feature: livox-sdk2)
    │       ├── velodyne.rs    #   spinning LiDAR (WIP)
    │       ├── ouster.rs      #   spinning LiDAR (WIP)
    │       └── hesai.rs       #   spinning LiDAR (WIP)
    ├── lidar-map/             # (placeholder) occupancy / voxel map — future
    ├── lidar-nav/             # (placeholder) planning & navigation — future
    └── fast-lio-app/          # offline driver binary (not published)
```

The algorithm core (`fast-lio`) is **vendor-SDK-free**: it only consumes the
normalized [`SensorData`](crates/fast-lio/src/types.rs). All hardware access
lives in `fast-lio-driver`, whose adapters translate each brand's raw output
into that format. Dependency direction: `fast-lio-app → fast-lio-driver →
fast-lio`.

## Quick start

Requirements: stable Rust ≥ 1.85 (edition 2024).

```bash
# run the offline demo (synthetic circular trajectory, IMU 200 Hz + LiDAR 10 Hz, 20 s)
cargo run -p fast-lio-app --release -- --sim

# run on a real Livox LiDAR (no ROS) — direct SDK2 connection
cargo run -p fast-lio-app --release -- --live /path/to/mid360_config.json

# results are written to ./out by default (or pass --out <dir>)
cargo run -p fast-lio-app --release -- --sim --out my_output
```

The demo drives the whole pipeline with the built-in `SimSource` and produces:

- `pos_log.txt` — per-frame pose (time, euler angles, position, velocity, gyro bias), the same format as the C++ node;
- `map.xyz` — the world-frame map points stored in the ikd-Tree.

## Library usage

Add the core crate as a dependency and feed it sensor data:

```rust
use fast_lio::laser_mapping::{LaserMapping, LioConfig, LioResult};
use fast_lio::types::{LidarType, SensorData, TimeUnit};

let cfg = LioConfig {
    lidar_type: LidarType::Velo16,
    filter_size_surf: 0.5,
    filter_size_map: 0.5,
    ..Default::default()
};

let mut mapping = LaserMapping::new(&cfg);

for sample in data_source {
    match sample {
        SensorData::Imu(imu) => mapping.add_imu(&imu),
        SensorData::LidarAvia(msg) => mapping.add_lidar_avia(&msg),
        SensorData::LidarStandard(msg) => mapping.add_lidar_standard(&msg),
    }
    // a synchronized frame is ready -> run one LIO iteration
    if mapping.has_data() {
        if let Some(res) = mapping.run_once() {
            // res: LioResult { time, pos, quat, vel, bg, ba, map_points, .. }
            println!("pose @ {:.3}s: {:?}", res.time, res.pos);
        }
    }
}
```

The same pattern works for any sensor input — implement the [`DataSource`](crates/fast-lio/src/data_source.rs) trait for your bag reader, socket, or device driver.

## Configuration

The pipeline is configured through [`LioConfig`](crates/fast-lio/src/laser_mapping.rs), mirroring the ROS parameters / yaml files of the C++ node:

| Parameter | Default | Meaning |
|---|---|---|
| `lidar_type` | `Avia` | `Avia` / `Velo16` / `Oust64` / `Marsim` |
| `feature_extract_enable` | `false` | enable plane/edge feature extraction |
| `point_filter_num` | `2` | keep every Nth point in direct mode |
| `blind` | `0.01` | minimum range (m² threshold) |
| `n_scans` / `scan_rate` | `16` / `10` | LiDAR lines & scan rate (Velodyne time computation) |
| `timestamp_unit` | `Us` | unit of the raw point timestamp field |
| `filter_size_surf` / `filter_size_map` | `0.5` / `0.5` | voxel sizes (scan / map, m) |
| `cube_len` | `1000` | local-map box side length (m) — launch files use `1000` |
| `det_range` | `300` | detection range used by the FOV sliding logic (m) |
| `gyr_cov` / `acc_cov` | `0.1` | IMU measurement covariances |
| `b_gyr_cov` / `b_acc_cov` | `1e-4` | bias random-walk covariances |
| `extrinsic_est_en` | `true` | online estimation of the LiDAR↔IMU extrinsic |
| `extrinsic_t` / `extrinsic_r` | identity | initial extrinsic (translation + 3×3 rotation) |
| `max_iteration` | `4` | IEKF iterations per frame |

## Data sources

`fast_lio::data_source` provides:

- **`DataSource`** — the trait implemented by any input (rosbag reader, file, socket…). Samples must be time-ordered.
- **`SimSource`** — a deterministic synthetic generator (static initialization phase followed by circular motion over wall/ground planes). Used by the demo and for CI-style smoke tests; not a replacement for real-data validation.

### Real device — Livox via SDK2 (no ROS)

With the `livox-sdk2` feature (enabled by default in `fast-lio-app`), the
`fast-lio-driver` crate connects **directly to the LiDAR over the network**, no ROS involved:

```bash
cargo run -p fast-lio-app --release -- --live mid360_config.json [--scan-ms 100]
```

Requirements and notes:

- A **valid Livox config file** (`mid360_config.json`, the same file used by Livox Viewer / driver2)
  listing the device IP / subnet. The SDK2 `Sdk::new` aborts on a missing/malformed file.
- The target machine needs `cmake` and a C++ compiler (the crate vendors and builds the official SDK2).
- Supported devices: **HAP / Mid-360** (SDK2). The older Avia SDK1 line is not covered.
- The pipeline runs in **direct odometry mode** (`feature_extract_enable = false`): the SDK2 stream is
  routed through a single scan line because per-point ring indices are not exposed by the SDK.
- Units: points are in meters; IMU gyro in rad/s; IMU accel is converted from **g** to m/s²
  (`ACC_G_TO_MPS2` in `crates/fast-lio-driver/src/livox.rs` — set to `1.0` if your firmware reports m/s² directly).
- Frame timestamps use a local monotonic clock (arrival time). If you need the device PTP/UTC
  timestamps for exact synchronization, `Packet::timestamp()` is exposed for that.

## Outputs

The offline app writes, per run:

- `pos_log.txt` — trajectory in the C++ `dump_lio_state_to_log` format (time, RPY, position, velocity, gyro bias);
- `map.xyz` — the accumulated world-frame map (`x y z` lines).

Both are directly comparable with logs produced by the C++ implementation for validation.

## Testing

```bash
cargo test --workspace       # 42 unit tests
cargo clippy --workspace --all-targets
```

Test coverage includes:

- SO(3)/S² manifold round-trips (`boxplus ∘ boxminus ≈ id`) and geometric invariants;
- process model Jacobians checked against finite differences (`df_dx`, `df_dw`);
- IEKF update behavior (position/rotation observability, invalid-measurement semantics);
- ikd-Tree k-NN results cross-checked against brute force, box deletion and downsampled insertion;
- plane fitting and voxel downsampling.

End-to-end: the `fast-lio-app` demo processes ~200 frames of synthetic data and converges point-to-plane residuals to the centimetre range.

## Performance notes

- Build with `--release`; the workspace enables `lto = "thin"` and `codegen-units = 1`.
- Consider `RUSTFLAGS="-C target-cpu=native"` for auto-vectorization of the linear algebra.
- The port keeps the C++ hot paths allocation-lean (reused buffers, manual binary heap for k-NN).
- Known simplification: ikd-Tree subtree rebuilds run inline on the calling thread (the C++ version uses a background thread); semantics are identical, worst-case latency differs. `rayon` is available for parallelizing the per-point matching loop.

## Roadmap

1. Real-dataset validation against the C++ implementation (trajectory ATE/RPE, per-stage golden comparison).
2. Verify `LivoxSource` on hardware; switch to PTP/UTC timestamps for tighter sync.
3. `DataSource` implementations for rosbag / custom files.
4. `lidar-map`: incremental occupancy / voxel map for planning.
5. `lidar-nav`: path planning and obstacle avoidance on top of the map.
6. Publish `fast-lio` (and later `lidar-map`, `lidar-nav`) to crates.io.

## Attribution & license

This project is a port of the following open-source works; the original algorithms, structure and variable naming are preserved wherever possible for numerical fidelity:

- [FAST-LIO2](https://github.com/hku-mars/FAST_LIO) — Xu et al., HKU Mars Lab
- [ikd-Tree](https://github.com/hku-mars/ikd-Tree) — Yixi Cai
- [IKFoM](https://github.com/hku-mars/IKFoM) / MTK — HKU / University of Bremen (C. Hertzberg et al.)

New Rust code in this repository is licensed under **MIT OR Apache-2.0** (see [`Cargo.toml`](Cargo.toml)). Ported code retains the copyright terms of the original projects (BSD-style notices); please review the upstream repositories before redistribution.

## References

- W. Xu, Y. Cai, D. He, J. Lin, F. Zhang, *FAST-LIO2: Fast Direct LiDAR-inertial Odometry*, IEEE Transactions on Robotics, 2022.
- Y. Cai, W. Xu, F. Zhang, *ikd-Tree: An Incremental K-D Tree for Robotic Applications*, arXiv:2102.10808, 2021.
- D. He, W. Xu, F. Zhang, *Kalman Filters on Differentiable Manifolds*, arXiv:2102.03804, 2021.
