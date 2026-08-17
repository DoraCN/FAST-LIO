# fast-lio

[![license](https://img.shields.io/badge/license-MIT%20OR%20Apache--2.0-blue)](#license)
[![edition](https://img.shields.io/badge/edition-2024-orange)](Cargo.toml)

**Languages:** English · [简体中文](README_zh.md)

**fast-lio** is a pure-Rust, dependency-light port of [FAST-LIO2](https://github.com/hku-mars/FAST_LIO) — a computationally efficient, robust tightly-coupled LiDAR-inertial odometry (LIO) system. It fuses raw LiDAR points with IMU data using an iterated error-state Kalman filter (IEKF) on a manifold and maintains an incremental k-d tree (ikd-Tree) map, enabling accurate, drift-bounded odometry and mapping at high rates.

The core crate contains **no I/O or ROS dependencies**: the whole front-end is a plain library that consumes timestamped IMU samples and LiDAR scans and produces poses, velocity, biases and the local map. This makes it easy to embed, unit-test, and drive from any data source (rosbag, custom file format, live sensors).

## Table of contents

- [Features](#features)
- [Status](#status)
- [Workspace layout](#workspace-layout)
- [Requirements](#requirements)
- [Quick start](#quick-start)
- [Mapping results](#mapping-results)
- [Command line reference](#command-line-reference)
- [Configuration](#configuration)
- [Library usage](#library-usage)
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
- Zero IO in the core crate — no ROS, no PCL.
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
    └── fast-lio-app/          # offline/live driver binary (not published)
```

The algorithm core (`fast-lio`) is **vendor-SDK-free**: it only consumes the
normalized [`SensorData`](crates/fast-lio/src/types.rs). All hardware access
lives in `fast-lio-driver`, whose adapters translate each brand's raw output
into that format. Dependency direction: `fast-lio-app → fast-lio-driver →
fast-lio`.

## Requirements

- **Rust toolchain ≥ 1.85** (edition 2024). Install via [rustup](https://rustup.rs).
- For live Livox devices: `cmake` and a C/C++ compiler on the target machine
  (the `livox-sdk2` crate vendors and builds the official C++ SDK2), plus
  network access to the LiDAR.

## Quick start

```bash
# 1) run the offline demo (synthetic circular trajectory, IMU 200 Hz + LiDAR 10 Hz, 20 s)
cargo run -p fast-lio-app --release -- --sim

# 2) run on a real LiDAR (no ROS) — driver-agnostic, pick the brand by name
cargo run -p fast-lio-app --release -- --driver livox --config mid360_config.json

# 3) results are written to ./out by default (or pass --out <dir>)
cargo run -p fast-lio-app --release -- --sim --out my_output
```

The demo drives the whole pipeline with the built-in `SimSource` and produces:

- `pos_log.txt` — per-frame pose (time, euler angles, position, velocity, gyro bias), the same format as the C++ node;
- `map.pcd` (default; or `.xyz` / `.ply`, see [`--out-format`](#command-line-reference)) — the world-frame map points stored in the ikd-Tree.

## Mapping results

The screenshots below were produced with the current Rust port running live on a
Mid-360 in a real environment:

| | |
|---|---|
| <img src="assets/map01.png" width="480"/> | <img src="assets/map02.png" width="480"/> |
| The complete 3D point-cloud map built by the pipeline, showing the full environment (walls, structures and terrain) without any filtering. | The same map with a slice of the Z (height) axis removed, so the drivable paths / road level become clearly visible in 3D. |

## Command line reference

`fast-lio-app` (the binary in `crates/fast-lio-app`) is **driver-agnostic**: you
select the LiDAR brand with `--driver <name>` and the CLI is never bound to a
specific sensor model.

```
usage: fast-lio-app [common opts] --sim | --driver <name> [driver opts]

common opts:
  --out <dir>              output directory (default "out")
  --out-format <fmt>       map file format: xyz | pcd | ply (default pcd)
  --scan-ms <ms>           scan frame period in ms (default 100)
  --duration <secs>        auto-stop after N seconds and save (default: run until Ctrl-C)
  --map-voxel <m>           global map voxel size (default 0.5; smaller = denser)
  --surf-voxel <m>          per-frame scan voxel size (default 0.5)
  --point-filter-num <n>    keep every Nth point (default 2; 1 = keep all)

modes:
  --sim                    synthetic demo data (default)
  --driver <name>          connect to a real LiDAR. Supported names:
    livox                  Livox (HAP / Mid-360) via SDK2, needs --config
    velodyne | ouster | hesai | marsim   spinning LiDAR (adapter may be WIP)

driver opts:
  --config <file>          vendor config file (Livox SDK2 JSON)
  --ip <addr>              LiDAR network address (spinning LiDARs)
  --port <port>            UDP data port (spinning LiDARs)

examples:
  fast-lio-app --sim
  fast-lio-app --driver livox --config mid360_config.json
  fast-lio-app --driver velodyne --ip 192.168.1.100 --port 2368
```

| Option | Default | Meaning |
|---|---|---|
| `--sim` | — | Run on the synthetic `SimSource` demo (mutually exclusive with `--driver`). |
| `--driver <name>` | — | Select the LiDAR driver by name (`livox`, `velodyne`, `ouster`, `hesai`, `marsim`). Unknown names are rejected with an actionable error; adapters not yet implemented (e.g. `hesai`) are reported as such. |
| `--config <file>` | — | Vendor config file (Livox SDK2 JSON, the same one used by Livox Viewer / driver2). |
| `--ip <addr>` | — | LiDAR network address (spinning LiDARs). |
| `--port <port>` | — | UDP data port for the spinning-LiDAR packet stream. |
| `--scan-ms <ms>` | `100` | LiDAR scan frame period in milliseconds (10 Hz → 100). Lower = higher scan rate. |
| `--duration <secs>` | — | Auto-stop after N seconds and save the map. Default runs until Ctrl-C. |
| `--map-voxel <m>` | `0.5` | Global-map voxel size (m). **Smaller = denser saved map** (e.g. `0.1`). |
| `--surf-voxel <m>` | `0.5` | Per-frame scan voxel size (m); keep consistent with `--map-voxel`. |
| `--point-filter-num <n>` | `2` | Keep every Nth point; `1` keeps all points (denser but slower). |
| `--out <dir>` | `out` | Output directory for the trajectory and map files (created if missing). |
| `--out-format <fmt>` | `pcd` | Map file format: `xyz`, `pcd`, or `ply`. See [Outputs](#outputs). |

Notes:

- `--live <config>` is kept as a **backwards-compatible alias** for `--driver livox --config <config>`.
- Adding a new LiDAR brand only requires implementing its adapter in
  `fast-lio-driver` and registering it in `open()`; the CLI needs **no change**
  (see [`fast-lio-driver`](crates/fast-lio-driver)).
- The `livox` driver runs in **direct odometry mode** (`feature_extract_enable = false`): the SDK2 stream is routed through a single scan line because per-point ring indices are not exposed by the SDK. Spinning LiDARs use the per-point `ring`/`time` fields instead.
- The **IMU accel is converted from g to m/s²** (see `ACC_G_TO_MPS2` in `crates/fast-lio-driver/src/livox.rs`; set it to `1.0` if your firmware reports m/s² directly).
- Frame timestamps use a local monotonic clock (arrival time). If you need exact PTP/UTC synchronization, `Packet::timestamp()` is exposed for that.

## Configuration

The pipeline is configured through [`LioConfig`](crates/fast-lio/src/laser_mapping.rs), which mirrors the ROS parameters / yaml files of the C++ node. Build a `LioConfig` and pass it to `LaserMapping::new(&cfg)`; all fields have sensible defaults, so `..Default::default()` gives a working configuration.

| Parameter | Type | Default | Meaning |
|---|---|---|---|
| `lidar_type` | `LidarType` | `Avia` | `Avia` / `Velo16` / `Oust64` / `Marsim` |
| `feature_extract_enable` | `bool` | `false` | enable plane/edge feature extraction |
| `point_filter_num` | `i32` | `2` | keep every Nth point in direct mode |
| `blind` | `f64` | `0.01` | minimum range (m² threshold) |
| `n_scans` / `scan_rate` | `usize` / `i32` | `16` / `10` | LiDAR lines & scan rate (Velodyne time computation) |
| `timestamp_unit` | `TimeUnit` | `Us` | unit of the raw point timestamp field |
| `filter_size_surf` / `filter_size_map` | `f32` | `0.5` / `0.5` | voxel sizes (scan / map, m) |
| `cube_len` | `f64` | `1000` | local-map box side length (m) — launch files use `1000` |
| `det_range` | `f32` | `300` | detection range used by the FOV sliding logic (m) |
| `fov_deg` | `f64` | `180.0` | reserved field (not yet wired into the FOV logic; the sliding map currently uses `det_range` only) |
| `gyr_cov` / `acc_cov` | `f64` | `0.1` | IMU measurement covariances |
| `b_gyr_cov` / `b_acc_cov` | `f64` | `1e-4` | bias random-walk covariances |
| `extrinsic_est_en` | `bool` | `true` | online estimation of the LiDAR↔IMU extrinsic |
| `time_sync_en` | `bool` | `false` | enable the LiDAR↔IMU time-offset estimator |
| `time_offset_lidar_to_imu` | `f64` | `0.0` | initial LiDAR-to-IMU time offset (s) |
| `extrinsic_t` | `[f64; 3]` | `[0,0,0]` | initial extrinsic translation |
| `extrinsic_r` | `[f64; 9]` | identity | initial extrinsic rotation (row-major 3×3) |
| `max_iteration` | `usize` | `4` | IEKF iterations per frame |

## Library usage

Add the core crate as a dependency and feed it sensor data. The core is **purely a library**: no I/O, no ROS — you supply the samples.

### 1. Add the dependency

```toml
[dependencies]
fast-lio = { path = "../fast-lio" }   # or a version tag once published
```

### 2. Implement the data source

Any time-ordered stream of [`SensorData`](crates/fast-lio/src/types.rs) works. Implement the [`DataSource`](crates/fast-lio/src/data_source.rs) trait (a bag reader, socket handler, or your own simulator):

```rust
use fast_lio::types::SensorData;

struct MySource;

impl fast_lio::data_source::DataSource for MySource {
    fn next(&mut self) -> Option<SensorData> {
        // read one sample from your bag / socket / device and return it
        todo!()
    }
}
```

### 3. Configure and run the pipeline

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

// `data_source` is any `DataSource` (see step 2)
while let Some(sample) = data_source.next() {
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

### Public API summary

| Item | Path | Purpose |
|---|---|---|
| `LaserMapping` | `fast_lio::laser_mapping::LaserMapping` | the main front-end: `new(&LioConfig)`, `add_imu`, `add_lidar_avia`, `add_lidar_standard`, `has_data`, `run_once` |
| `LioConfig` | `fast_lio::laser_mapping::LioConfig` | pipeline configuration (see [Configuration](#configuration)) |
| `LioResult` | `fast_lio::laser_mapping::LioResult` | per-frame output: `time`, `pos`, `quat [w,x,y,z]`, `vel`, `bg`, `ba`, `map_points`, `effct_feat_num`, `res_mean` |
| `SensorData` | `fast_lio::types::SensorData` | normalized input enum: `Imu` / `LidarAvia` / `LidarStandard` |
| `LidarType` | `fast_lio::types::LidarType` | `Avia` / `Velo16` / `Oust64` / `Marsim` |
| `TimeUnit` | `fast_lio::types::TimeUnit` | `Sec` / `Ms` / `Us` / `Ns` — unit of the per-point timestamp field |
| `DataSource` | `fast_lio::data_source::DataSource` | trait for any time-ordered sensor source |
| `KdTree` | `fast_lio::ikdtree::KdTree` | the incremental k-d tree (map): `build`, `nearest_search`, `add_points`, `delete_point_boxes`, `validnum` |

The measurement model is exposed for advanced users: `LaserMapping` keeps its
`kf` (the `EseKf`) and `ikdtree` (the `KdTree`) as public fields, matching the
C++ node's structure, so you can drive the IEKF update yourself with a custom
`h_share_model`.

## Data sources

`fast_lio::data_source` provides:

- **`DataSource`** — the trait implemented by any input (rosbag reader, file, socket…). Samples must be time-ordered.
- **`SimSource`** — a deterministic synthetic generator (static initialization phase followed by circular motion over wall/ground planes). Used by the demo and for CI-style smoke tests; not a replacement for real-data validation.

### Real device — Livox via SDK2 (no ROS)

With the `livox-sdk2` feature (enabled by default in `fast-lio-app`), the
`fast-lio-driver` crate connects **directly to the LiDAR over the network**, no ROS involved:

```bash
cargo run -p fast-lio-app --release -- --driver livox --config mid360_config.json [--scan-ms 100] [--duration 120]
```

The program streams until **Ctrl-C** (graceful exit: trajectory + map are saved) or
`--duration <secs>` elapses.

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

The app writes, per run:

| File | Format | Description |
|---|---|---|
| `pos_log.txt` | text | trajectory in the C++ `dump_lio_state_to_log` format: `time RPY(deg) pos vel bg` |
| `map.pcd` | ASCII PCD | `x y z intensity` — readable by PCL / rviz tools (default) |
| `map.xyz` | text (`x y z` per line) | the accumulated world-frame map (via `--out-format xyz`) |
| `map.ply` | ASCII PLY | `x y z intensity` — opens directly in CloudCompare / MeshLab (via `--out-format ply`) |

> `intensity` is the raw LiDAR reflectivity passed through from the sensor
> (for Livox it is the SDK2 `reflectivity` 0–255; the synthetic demo fills a
> constant `100.0`). It is not used by the algorithm.

All formats are directly comparable with logs produced by the C++ implementation for validation.

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
