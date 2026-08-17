//! Plan a global path (A*) on a 2D occupancy grid built by `lidar-map`.
//!
//! Usage:
//! ```sh
//! cargo run --release -p lidar-nav --example plan -- \
//!     --pgm map.pgm --yaml map.yaml \
//!     --start 0.0 0.0 --goal 10.0 5.0
//! ```
//!
//! Reads the grid from the PGM + YAML pair written by
//! `lidar-map --example to_2d_grid`, runs A* from `--start x y` to
//! `--goal x y`, and prints the waypoints plus an ASCII visualization.

use lidar_map::GridMap;
use lidar_nav::{AStarOptions, Waypoint, astar};
use std::fs;

fn parse_yaml(path: &str) -> Result<(f64, f64, f64), String> {
    let text = fs::read_to_string(path).map_err(|e| e.to_string())?;
    let mut res = 0.05;
    let mut ox = 0.0;
    let mut oy = 0.0;
    for line in text.lines() {
        let line = line.trim();
        if let Some(v) = line.strip_prefix("resolution:") {
            res = v.trim().parse().map_err(|_| "bad resolution")?;
        } else if let Some(v) = line.strip_prefix("origin:") {
            let inner = v.trim().trim_start_matches('[').trim_end_matches(']');
            let mut it = inner.split(',').filter_map(|s| s.trim().parse::<f64>().ok());
            ox = it.next().unwrap_or(0.0);
            oy = it.next().unwrap_or(0.0);
        }
    }
    Ok((res, ox, oy))
}

fn main() -> Result<(), String> {
    let mut pgm = "map.pgm".to_string();
    let mut yaml = "map.yaml".to_string();
    let mut start = [0.0f64, 0.0f64];
    let mut goal = [10.0f64, 5.0f64];
    let mut inflation = 0.3f64;

    let mut args = std::env::args().skip(1);
    while let Some(a) = args.next() {
        match a.as_str() {
            "--pgm" => pgm = args.next().ok_or("--pgm needs a value")?,
            "--yaml" => yaml = args.next().ok_or("--yaml needs a value")?,
            "--start" => {
                start[0] = args.next().and_then(|v| v.parse().ok()).ok_or("bad --start x")?;
                start[1] = args.next().and_then(|v| v.parse().ok()).ok_or("bad --start y")?;
            }
            "--goal" => {
                goal[0] = args.next().and_then(|v| v.parse().ok()).ok_or("bad --goal x")?;
                goal[1] = args.next().and_then(|v| v.parse().ok()).ok_or("bad --goal y")?;
            }
            "--inflation" => {
                inflation = args.next().and_then(|v| v.parse().ok()).ok_or("bad --inflation")?
            }
            s if s.starts_with('-') => return Err(format!("unknown flag: {s}")),
            s => return Err(format!("unexpected positional argument: {s}")),
        }
    }

    let (res, ox, oy) = parse_yaml(&yaml)?;
    println!("loading grid from {pgm} (res {res} m, origin [{ox}, {oy}]) ...");
    let grid = GridMap::load_pgm(&pgm, res, ox, oy).map_err(|e| e.to_string())?;
    let (w, h) = grid.dims();
    println!("grid {w}x{h} cells, {res} m");

    let start_wp = Waypoint { x: start[0], y: start[1] };
    let goal_wp = Waypoint { x: goal[0], y: goal[1] };
    println!("planning A* from {start_wp:?} to {goal_wp:?} (inflation {inflation} m) ...");

    let path = astar(
        &grid,
        start_wp,
        goal_wp,
        &AStarOptions { inflation, ..Default::default() },
    )
    .ok_or("no path found (goal unreachable or start/goal blocked)")?;

    println!("path: {} waypoints", path.len());
    for (i, wp) in path.iter().enumerate() {
        println!("  {i:3}: ({:.3}, {:.3})", wp.x, wp.y);
    }

    // ASCII visualization: downsample the grid to ~80 columns.
    let (ncols, nrows) = grid.dims();
    let gw = 80usize;
    let gh = ((nrows as f64 / ncols as f64) * gw as f64).max(10.0) as usize;
    let mut canvas = vec![b' '; gw * gh];
    for ((col, row), _) in grid.iter_cells() {
        if col < 0 || row < 0 || (col as usize) >= ncols || (row as usize) >= nrows {
            continue;
        }
        let cx = (col as f64 / ncols as f64 * gw as f64) as usize;
        let cy = gh - 1 - (row as f64 / nrows as f64 * gh as f64) as usize;
        if grid.occupancy(col, row) > 0.6 {
            canvas[cy * gw + cx] = b'#';
        }
    }
    for wp in &path {
        let cx = ((wp.x - ox) / (ncols as f64 * res) * gw as f64) as isize;
        let cy = gh as isize - 1 - (((wp.y - oy) / (nrows as f64 * res)) * gh as f64) as isize;
        if cx >= 0 && cx < gw as isize && cy >= 0 && cy < gh as isize {
            canvas[cy as usize * gw + cx as usize] = b'o';
        }
    }
    let (sc, sr) = grid.world_to_cell(start[0], start[1]);
    let (gc, gr) = grid.world_to_cell(goal[0], goal[1]);
    let scx = (sc as f64 / ncols as f64 * gw as f64) as usize;
    let scy = gh - 1 - (sr as f64 / nrows as f64 * gh as f64) as usize;
    let gcx = (gc as f64 / ncols as f64 * gw as f64) as usize;
    let gcy = gh - 1 - (gr as f64 / nrows as f64 * gh as f64) as usize;
    if scx < gw && scy < gh {
        canvas[scy * gw + scx] = b'S';
    }
    if gcx < gw && gcy < gh {
        canvas[gcy * gw + gcx] = b'G';
    }
    println!("legend: # occupied, o path, S start, G goal");
    for r in 0..gh {
        println!("{}", String::from_utf8_lossy(&canvas[r * gw..(r + 1) * gw]));
    }
    Ok(())
}
