//! Multi-waypoint navigation on a 2D grid map, driven by a task file.
//!
//! Task file format (one waypoint per line):
//! ```text
//! x y [yaw_deg] [dwell_sec]
//! ```
//! e.g.
//! ```text
//! # go to the door, face it, wait 5 s, then the loading bay
//! 2.0 1.5 90 5
//! 8.0 -2.0 0
//! ```
//!
//! Usage:
//! ```sh
//! cargo run --release -p lidar-nav --example navigate -- \
//!     --pgm map.pgm --yaml map.yaml --task task.txt \
//!     [--start 0 0] [--max-vx 0.4] [--max-wz 1.0] [--inflation 0.3]
//! ```
//!
//! Simulates the robot with a simple kinematic model and prints each control
//! tick, so you can verify the sequence (drive → face → dwell → next) without
//! a real chassis. The [`TaskExecutor`] API is the same one you'd call from a
//! live loop (feeding the real odom pose).

use lidar_map::GridMap;
use lidar_nav::task::{Phase, TaskExecutor, TaskParams, load_task, norm_angle};
use std::fs;
use std::time::{Duration, Instant};

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
    let mut task_file = "task.txt".to_string();
    let mut start = [0.0f64, 0.0, 0.0];
    let mut max_vx = 0.4f64;
    let mut max_wz = 1.0f64;
    let mut inflation = 0.3f64;

    let mut args = std::env::args().skip(1);
    while let Some(a) = args.next() {
        match a.as_str() {
            "--pgm" => pgm = args.next().ok_or("--pgm needs a value")?,
            "--yaml" => yaml = args.next().ok_or("--yaml needs a value")?,
            "--task" => task_file = args.next().ok_or("--task needs a value")?,
            "--start" => {
                start[0] = args.next().and_then(|v| v.parse().ok()).ok_or("bad --start x")?;
                start[1] = args.next().and_then(|v| v.parse().ok()).ok_or("bad --start y")?;
            }
            "--max-vx" => max_vx = args.next().and_then(|v| v.parse().ok()).ok_or("bad --max-vx")?,
            "--max-wz" => max_wz = args.next().and_then(|v| v.parse().ok()).ok_or("bad --max-wz")?,
            "--inflation" => inflation = args.next().and_then(|v| v.parse().ok()).ok_or("bad --inflation")?,
            s if s.starts_with('-') => return Err(format!("unknown flag: {s}")),
            s => return Err(format!("unexpected positional argument: {s}")),
        }
    }

    let task = load_task(&task_file)?;
    println!("task: {} waypoints from {task_file}", task.len());
    for (i, wp) in task.iter().enumerate() {
        println!(
            "  {i}: ({:.2}, {:.2}) yaw={} dwell={}s",
            wp.x,
            wp.y,
            wp.yaw_deg.map_or("--".into(), |y| format!("{y:.0}°")),
            wp.dwell_s
        );
    }

    let (res, ox, oy) = parse_yaml(&yaml)?;
    println!("loading grid from {pgm} (res {res} m, origin [{ox}, {oy}]) ...");
    let grid = GridMap::load_pgm(&pgm, res, ox, oy).map_err(|e| e.to_string())?;

    let params = TaskParams {
        max_vx,
        max_wz,
        inflation,
        ..Default::default()
    };
    let mut exec = TaskExecutor::new(&grid, task, params);

    println!("simulating from start ({:.2}, {:.2}) ...", start[0], start[1]);
    let mut pose = start;
    let t0 = Instant::now();
    let dt = 0.1f64;
    let mut last_print = t0;
    let mut n = 0usize;
    loop {
        let now = t0 + Duration::from_secs_f64(n as f64 * dt);
        let c = exec.step(pose, now);
        if now.duration_since(last_print) >= Duration::from_millis(200) || c.phase != Phase::Navigating {
            last_print = now;
            println!(
                "[{:6.1}s] pos=({:7.3},{:7.3}) yaw={:6.1}° phase={:?} cmd=(vx {:.2}, wz {:.2})",
                n as f64 * dt,
                pose[0],
                pose[1],
                pose[2].to_degrees(),
                c.phase,
                c.vx,
                c.wz
            );
        }
        if c.phase == Phase::Done {
            println!("task complete.");
            return Ok(());
        }
        pose[2] = norm_angle(pose[2] + c.wz * dt);
        pose[0] += c.vx * pose[2].cos() * dt;
        pose[1] += c.vx * pose[2].sin() * dt;
        n += 1;
        if n > 20_000 {
            return Err("simulation did not finish in 2000 s".into());
        }
    }
}
