# fast-lio-driver

Device adapters that feed the [`fast-lio`](https://crates.io/crates/fast-lio)
core with normalized [`SensorData`]. The algorithm core never touches a vendor
SDK — this crate translates each brand's raw output into the normalized format.

- **Livox (HAP / Mid-360)** — via the official Livox SDK2 (`livox-sdk2` crate,
  feature `livox-sdk2`). Requires `cmake` + a C++ compiler at build time.
- **Velodyne / Ouster / Hesai / Marsim** — spinning-LiDAR adapters (WIP).

## Usage

```toml
[dependencies]
fast-lio-driver = "0.1"
```

```rust
use std::time::Duration;
use fast_lio_driver::{open, DriverParams};

let params = DriverParams::new(
    fast_lio::types::LidarType::Avia,
    Some("mid360_config.json".into()),
    None,
    None,
    Duration::from_millis(100),
);
let mut source = open(&params)?; // Box<dyn DataSource>
```

Or use the ready-made constructors:

```rust
let params = DriverParams::livox("mid360_config.json", Duration::from_millis(100));
```

## License

MIT OR Apache-2.0. See the [repository](https://github.com/DoraCN/FAST-LIO)
for the full project documentation.
