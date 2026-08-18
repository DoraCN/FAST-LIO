//! Full robot navigation: fast-lio odometry + lidar-map grid + lidar-nav task
//! planning, driving the chassis via `chassis-driver`.
//!
//! One command drives the robot through a task file:
//! ```sh
//! cargo run --release -p nav-app -- \
//!     --port /dev/chassis --baud 115200 \
//!     --map map.yaml --task task.txt \
//!     [--config mid360_config.json] [--radius 0.4] [--lidar-fwd 0.30]
//! ```
//!
//! Task file format: one waypoint per line, `x y [yaw_deg] [dwell_sec]`.
//!
//! Assumptions (same as the reference nav):
//! - the robot boots at the pose where the map was built, so fast-lio odom
//!   (world frame = map origin) is directly the robot pose in the map;
//! - the lidar is mounted `--lidar-fwd` ahead of the chassis rotation centre,
//!   aligned with the chassis heading.

use fast_lio::data_source::NonBlocking;
use fast_lio::laser_mapping::{LaserMapping, LioConfig, LioResult};
use fast_lio::types::{AviaMsg, LidarType, SensorData, TimeUnit};
use fast_lio_driver::{DriverParams, open};
use lidar_map::GridMap;
use lidar_nav::task::{Control, Phase, TaskExecutor, TaskParams, TaskWaypoint, load_task};
use std::fs;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use chassis_driver::{Chassis, SerialTransport};

const TICK: Duration = Duration::from_millis(20);
const INIT_DELAY: Duration = Duration::from_secs(2);

/// Yaw (rad) from a [w, x, y, z] quaternion.
fn yaw_from_quat(q: [f64; 4]) -> f64 {
    let [w, x, y, z] = q;
    (2.0 * (w * z + x * y)).atan2(1.0 - 2.0 * (y * y + z * z))
}

/// Odom pose (lidar/IMU frame) -> rotation-centre pose in the map frame.
/// The rotation centre is `rot_fwd` ahead of the geometric centre and the
/// lidar is `lidar_fwd` ahead of it, so relative to the lidar the rotation
/// centre sits at `(rot_fwd - lidar_fwd)` along the heading.
fn chassis_pose(res: &LioResult, lidar_fwd: f64, rot_fwd: f64) -> [f64; 3] {
    let yaw = yaw_from_quat(res.quat);
    let rel = rot_fwd - lidar_fwd;
    [
        res.pos[0] + rel * yaw.cos(),
        res.pos[1] + rel * yaw.sin(),
        yaw,
    ]
}

/// Nearest obstacle in a forward wedge of the latest lidar frame, in the
/// lidar frame (lidar forward == chassis forward). Only points that could hit
/// the body are considered: above the ground plane (lidar is `lidar_height`
/// above the floor) and below ~0.6 m.
fn front_obstacle(msg: &AviaMsg, lidar_height: f64, width: f64) -> Option<f64> {
    let mut dmin = f64::MAX;
    let z_min = (-lidar_height + 0.05) as f32;
    let z_max = 0.6f32;
    let half_width = (width / 2.0) as f32;
    for p in &msg.points {
        if p.z < z_min || p.z > z_max {
            continue;
        }
        if p.x < 0.05f32 {
            continue;
        }
        if p.y.abs() > half_width {
            continue;
        }
        let d = (p.x as f64).hypot(p.y as f64);
        if d < dmin {
            dmin = d;
        }
    }
    (dmin < f64::MAX).then_some(dmin)
}

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

struct Args {
    port: String,
    baud: u32,
    map_yaml: String,
    task: String,
    config: String,
    radius: f64,
    lidar_fwd: f64,
    rot_fwd: f64,
    lidar_height: f64,
    stop_dist: f64,
    slow_dist: f64,
    obst_width: f64,
    max_vx: f64,
    max_wz: f64,
    lookahead: f64,
    arrive: f64,
    duration: Option<f64>,
}

impl Default for Args {
    fn default() -> Self {
        Self {
            port: String::new(),
            baud: 115200,
            map_yaml: "map.yaml".to_string(),
            task: "task.txt".to_string(),
            config: String::new(),
            radius: 0.40,
            lidar_fwd: 0.10,
            rot_fwd: 0.0,
            lidar_height: 0.5,
            stop_dist: 0.5,
            slow_dist: 1.5,
            obst_width: 0.6,
            max_vx: 0.4,
            max_wz: 1.0,
            lookahead: 0.6,
            arrive: 0.15,
            duration: None,
        }
    }
}

fn parse_args() -> Args {
    let mut a = Args::default();
    let expect = |flag: &str, v: Option<String>| match v {
        Some(v) => v,
        None => {
            eprintln!("{flag} expects a value");
            std::process::exit(2);
        }
    };
    let mut args = std::env::args().skip(1);
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--port" => a.port = expect("--port", args.next()),
            "--baud" => a.baud = expect("--baud", args.next()).parse().unwrap_or(115200),
            "--map" => a.map_yaml = expect("--map", args.next()),
            "--task" => a.task = expect("--task", args.next()),
            "--config" => a.config = expect("--config", args.next()),
            "--radius" => a.radius = expect("--radius", args.next()).parse().unwrap_or(0.40),
            "--lidar-fwd" => a.lidar_fwd = expect("--lidar-fwd", args.next()).parse().unwrap_or(0.10),
            "--rot-fwd" => a.rot_fwd = expect("--rot-fwd", args.next()).parse().unwrap_or(0.0),
            "--lidar-height" => {
                a.lidar_height = expect("--lidar-height", args.next()).parse().unwrap_or(0.5)
            }
            "--stop-dist" => a.stop_dist = expect("--stop-dist", args.next()).parse().unwrap_or(0.5),
            "--slow-dist" => a.slow_dist = expect("--slow-dist", args.next()).parse().unwrap_or(1.5),
            "--obst-width" => a.obst_width = expect("--obst-width", args.next()).parse().unwrap_or(0.6),
            "--max-vx" => a.max_vx = expect("--max-vx", args.next()).parse().unwrap_or(0.4),
            "--max-wz" => a.max_wz = expect("--max-wz", args.next()).parse().unwrap_or(1.0),
            "--lookahead" => a.lookahead = expect("--lookahead", args.next()).parse().unwrap_or(0.6),
            "--arrive" => a.arrive = expect("--arrive", args.next()).parse().unwrap_or(0.15),
            "--duration" => {
                a.duration = expect("--duration", args.next()).parse().ok().filter(|d| *d > 0.0)
            }
            s if s.starts_with('-') => {
                eprintln!("unknown flag: {s}");
                std::process::exit(2);
            }
            s => {
                eprintln!("unexpected positional argument: {s}");
                std::process::exit(2);
            }
        }
    }
    if a.port.is_empty() {
        eprintln!("--port /dev/ttyXXX is required (not hardcoded)");
        std::process::exit(2);
    }
    a
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args = parse_args();
    let config = if args.config.is_empty() {
        "mid360_config.json".to_string()
    } else {
        args.config
    };

    // ---- map + task --------------------------------------------------------
    let (res, ox, oy) = parse_yaml(&args.map_yaml)?;
    let pgm = args.map_yaml.replace(".yaml", ".pgm");
    let mut grid = GridMap::load_pgm(&pgm, res, ox, oy).map_err(|e| e.to_string())?;
    // inflate obstacles for the A* costmap (avoid walls); match the reference
    // nav exactly: inflate by the robot radius with no extra margin
    grid.inflate(args.radius);
    let (w, h) = grid.dims();
    let p = grid.params();
    println!(
        "map: {pgm} res={res} dims={w}x{h} world x:[{:.2},{:.2}] y:[{:.2},{:.2}] inflated {:.2} m",
        p.min_x,
        p.max_x,
        p.min_y,
        p.max_y,
        args.radius
    );

    let task: Vec<TaskWaypoint> = load_task(&args.task)?;
    println!("task: {} waypoints from {}", task.len(), args.task);
    for (i, wp) in task.iter().enumerate() {
        let (c, r) = grid.world_to_cell(wp.x, wp.y);
        let inside = grid.in_bounds(c, r);
        println!(
            "  {}: ({:.2}, {:.2}) yaw={} dwell={}s {}",
            i,
            wp.x,
            wp.y,
            wp.yaw_deg.map_or("--".into(), |y| format!("{y:.0}°")),
            wp.dwell_s,
            if inside { "" } else { "  <-- OUTSIDE MAP" }
        );
    }

    let task_params = TaskParams {
        lookahead: args.lookahead,
        max_vx: args.max_vx,
        max_wz: args.max_wz,
        arrive: args.arrive,
        inflation: 0.0, // grid already inflated
        ..Default::default()
    };

    // ---- odometry (fast-lio) ----------------------------------------------
    let cfg = LioConfig {
        lidar_type: LidarType::Avia,
        feature_extract_enable: false,
        point_filter_num: 2,
        n_scans: 6,
        scan_rate: 10,
        timestamp_unit: TimeUnit::Us,
        filter_size_surf: 0.5,
        filter_size_map: 0.5,
        ..Default::default()
    };
    let mut mapping = LaserMapping::new(&cfg);
    let mut latest_odom: Option<LioResult> = None;

    // ---- livox source ------------------------------------------------------
    let params = DriverParams::livox(&config, Duration::from_millis(100));
    let mut source = open(&params).map_err(|e| format!("livox driver: {e}"))?;
    println!("livox connected ({config})");

    // ---- chassis -----------------------------------------------------------
    let mut chassis = Chassis::new(SerialTransport::open(&args.port, args.baud)?);
    chassis.init()?;
    println!("chassis connected on {} @ {} baud", args.port, args.baud);

    let done = Arc::new(AtomicBool::new(false));
    ctrlc::set_handler({
        let done = done.clone();
        move || done.store(true, Ordering::SeqCst)
    })?;

    let t0 = Instant::now();
    let mut executor = TaskExecutor::new(&grid, task, task_params);
    let mut last_frame: Option<AviaMsg> = None;
    let mut last_frame_time = Instant::now();
    let mut cmd_vx = 0.0f64;
    let mut cmd_wz = 0.0f64;
    let mut last_report = Instant::now();
    let mut n_frames = 0u64;

    println!("navigating ... Ctrl-C to stop");
    loop {
        let timed_out = args.duration.is_some_and(|d| t0.elapsed().as_secs_f64() >= d);
        if done.load(Ordering::SeqCst) || timed_out {
            break;
        }

        // consume sensor data, feed odometry
        loop {
            match source.try_next() {
                Ok(Some(SensorData::Imu(imu))) => mapping.add_imu(&imu),
                Ok(Some(SensorData::LidarAvia(msg))) => {
                    last_frame = Some(msg.clone());
                    last_frame_time = Instant::now();
                    mapping.add_lidar_avia(&msg);
                    n_frames += 1;
                }
                Ok(Some(SensorData::LidarStandard(msg))) => mapping.add_lidar_standard(&msg),
                Ok(None) => break,
                Err(NonBlocking) => break,
            }
        }
        if t0.elapsed() >= INIT_DELAY && mapping.has_data() && let Some(r) = mapping.run_once() {
            latest_odom = Some(r);
        }
        // plan + drive
        if let Some(res) = latest_odom.as_ref() {
            let pose = chassis_pose(res, args.lidar_fwd, args.rot_fwd);
            let ctrl: Control = executor.step(pose, Instant::now());

            let mut vx = ctrl.vx;
            let mut wz = ctrl.wz;

            // front-obstacle stop / slow (live lidar frame)
            let obstacle = if last_frame_time.elapsed() < Duration::from_millis(500) {
                last_frame
                    .as_ref()
                    .and_then(|f| front_obstacle(f, args.lidar_height, args.obst_width))
            } else {
                None
            };
            if let Some(d) = obstacle {
                if vx > 0.0 && d < args.stop_dist {
                    vx = 0.0;
                    wz = 0.0;
                    println!("OBSTACLE {:.2}m < {:.2}m -> STOP", d, args.stop_dist);
                } else if vx > 0.0 && d < args.slow_dist {
                    vx *= (d - args.stop_dist) / (args.slow_dist - args.stop_dist);
                    println!("OBSTACLE {:.2}m -> slow vx={:.2}", d, vx);
                }
            }

            cmd_vx = vx;
            cmd_wz = wz;
            if ctrl.phase == Phase::Done {
                chassis.stop()?;
                println!("task complete — stopped");
                break;
            }
            chassis.set_velocity(vx, wz)?;
            chassis.keep_alive()?;
        } else {
            chassis.stop()?;
        }

        if last_report.elapsed() >= Duration::from_millis(500) {
            last_report = Instant::now();
            if let Some(res) = latest_odom.as_ref() {
                let p = chassis_pose(res, args.lidar_fwd, args.rot_fwd);
                println!(
                    "pos=({:7.3},{:7.3}) yaw={:6.1}° cmd(vx={:.2},wz={:.2}) phase={:?} frames={n_frames}",
                    p[0], p[1], p[2].to_degrees(), cmd_vx, cmd_wz, executor.phase()
                );
            }
        }

        std::thread::sleep(TICK);
    }

    chassis.stop()?;
    println!("stopped (Ctrl-C / duration / task done)");
    Ok(())
}
