# nav-app

Full robot navigation application: **fast-lio odometry + livox sensor +
lidar-map grid + lidar-nav planning**, driving a differential chassis via
[`chassis-driver`](https://github.com/DoraCN/chassis-driver).

One command drives the robot through a multi-waypoint task:

```sh
cargo run --release -p nav-app -- \
    --port /dev/chassis --baud 115200 \
    --map map.yaml --task task.txt \
    [--config mid360_config.json] [--radius 0.4] [--lidar-fwd 0.30] [--rot-fwd 0.15]
```

## Task file

`task.txt` — one waypoint per line, `x y [yaw_deg] [dwell_sec]`:

```text
# go to the door, face it, wait 5 s, then the loading bay
2.0 1.5 90 5
8.0 -2.0 0
```

The robot drives to each waypoint (A* + Pure Pursuit), rotates to face
`yaw_deg` if given, holds for `dwell_sec` if given, then moves to the next.

## Parameters

| Flag | Default | Meaning |
|---|---|---|
| `--port` | **required** | serial device for the chassis (e.g. `/dev/chassis`) |
| `--baud` | `115200` | serial baud rate |
| `--map` | `map.yaml` | navigation map YAML (from `lidar-map to_2d_grid`) |
| `--task` | `task.txt` | waypoint task file |
| `--config` | `mid360_config.json` | livox config JSON |
| `--radius` | `0.40` | robot radius (m) for obstacle inflation |
| `--lidar-fwd` | `0.10` | lidar offset forward of the rotation centre (m) |
| `--rot-fwd` | `0.0` | rotation centre forward of the geometric centre (m) |
| `--lidar-height` | `0.5` | lidar height above floor (m), for stop-obstacle filtering |
| `--stop-dist` | `0.5` | stop when a front obstacle is closer than this (m) |
| `--slow-dist` | `1.5` | slow down when a front obstacle is closer than this (m) |
| `--obst-width` | `0.6` | forward wedge half-width for obstacle detection (m) |
| `--max-vx` | `0.4` | max linear speed (m/s) |
| `--max-wz` | `1.0` | max angular speed (rad/s) |
| `--lookahead` | `0.6` | pure pursuit lookahead distance (m) |
| `--arrive` | `0.15` | arrival tolerance (m) |
| `--duration` | — | stop after N seconds |

## Pipeline

```
livox → fast-lio (odometry) → lidar-map grid + A* → lidar-nav task executor
      → pure pursuit (vx, wz) → chassis-driver → differential chassis
```

- 20 ms control tick; the chassis speed keep-alive is re-sent every tick.
- Live front-obstacle stop / slow using the latest lidar frame.
- Ctrl-C / `--duration` → safe stop (the chassis driver also sends zero on drop).
