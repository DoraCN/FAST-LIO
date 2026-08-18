# lidar-nav

Path planning and navigation on top of the map produced by
[`lidar-map`](https://crates.io/crates/lidar-map) / FAST-LIO.

## Features

- **`astar`** — grid-based global path planning (A*) on a
  [`GridMap`](lidar-map) with obstacle inflation, octile heuristic and a
  configurable expansion budget.
- **`dwa`** — local obstacle avoidance (Dynamic Window Approach) that samples
  feasible velocities inside the dynamic window, simulates short trajectories,
  and returns a `(linear, angular)` control command scored by goal progress,
  clearance and speed.
- **`task`** — multi-waypoint navigation: A* plan → Pure Pursuit follow → face
  the goal heading → dwell → next waypoint, driven by a text file.

## Usage

```toml
[dependencies]
lidar-map = "0.1"
lidar-nav = "0.1"
```

### Global plan (A*)

```rust
use lidar_map::{GridMap, GridMapParams};
use lidar_nav::{AStarOptions, Waypoint, astar};

let grid = GridMap::new(GridMapParams::default());
let path = astar(
    &grid,
    Waypoint { x: 0.0, y: 0.0 },
    Waypoint { x: 10.0, y: 5.0 },
    &AStarOptions { inflation: 0.3, ..Default::default() },
).expect("path");

for wp in &path {
    println!("({:.2}, {:.2})", wp.x, wp.y);
}
```

### Local obstacle avoidance (DWA)

```rust
use lidar_nav::{DwaGoal, DwaParams, DwaState, Obstacle, Pose, dwa_step};

let params = DwaParams::default();
let state = DwaState { pose: Pose { x: 0.0, y: 0.0, theta: 0.0 }, ..Default::default() };
let goal = DwaGoal { x: 5.0, y: 0.0, tolerance: 0.4 };
let obstacles = [Obstacle::new(2.0, 0.5, 0.2)];

if let Some(cmd) = dwa_step(&params, &state, goal, &obstacles) {
    println!("cmd: linear={:.2} m/s, angular={:.2} rad/s", cmd.linear, cmd.angular);
}
```

## CLI example

Plan a global path on a grid produced by `lidar-map`:

```sh
cargo run --release -p lidar-nav --example plan -- \
    --pgm map.pgm --yaml map.yaml --start 0 0 --goal 10 5
```

### Multi-waypoint task

Task file (`task.txt`) — one waypoint per line:

```text
# x y [yaw_deg] [dwell_sec]
2.0 1.5 90 5
8.0 -2.0 0
```

Simulate the sequence (drive → face heading → dwell → next) on a map:

```sh
cargo run --release -p lidar-nav --example navigate -- \
    --pgm map.pgm --yaml map.yaml --task task.txt --start 0 0
```

## License

MIT OR Apache-2.0. See the [repository](https://github.com/DoraCN/FAST-LIO).
