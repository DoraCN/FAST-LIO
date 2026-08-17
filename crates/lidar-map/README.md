# lidar-map

Occupancy / voxel map built incrementally from
[FAST-LIO](https://github.com/DoraCN/FAST-LIO) point clouds, ready for planning
& navigation.

## Features

- **`GridMap`** — 2D occupancy grid with:
  - incremental **ray-casting updates** from a sensor pose + a scan
    (`update_from_scan` / `update_from_cloud`),
  - direct `mark_occupied` for static global maps,
  - occupancy queries (`occupancy`, `occupancy_at`, `is_occupied`),
  - PGM / PNG / YAML export and PGM loading (ROS `map_server` compatible).
- **`VoxelMap`** — sparse 3D occupancy voxel map with incremental updates and
  occupancy queries.

## Usage

```toml
[dependencies]
lidar-map = "0.1"
```

```rust
use lidar_map::{GridMap, GridMapParams};

let mut grid = GridMap::new(GridMapParams {
    resolution: 0.05,
    min_x: -50.0,
    min_y: -50.0,
    max_x: 50.0,
    max_y: 50.0,
    ..Default::default()
});

// incrementally feed world-frame scans with the LiDAR origin
grid.update_from_scan([0.0, 0.0], &[[2.0, 1.0], [2.5, 1.0], [1.8, -0.5]]);

// query
if grid.is_occupied(2.0, 1.0, 0.6) {
    println!("cell occupied, p = {:.3}", grid.occupancy_at(2.0, 1.0));
}

// export for a nav stack
grid.save_pgm("map.pgm")?;
grid.save_yaml("map.yaml", "map.pgm")?;
```

## CLI example

Build a 2D occupancy grid from a 3D point cloud (PCD):

```sh
cargo run --release -p lidar-map --example to_2d_grid -- \
    --input map.pcd --height 0.0 --band 0.3 --resolution 0.05 --output map
# writes map.pgm, map.png, map.yaml
```

## License

MIT OR Apache-2.0. See the [repository](https://github.com/DoraCN/FAST-LIO).
