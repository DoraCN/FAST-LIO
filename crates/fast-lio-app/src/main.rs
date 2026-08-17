use std::fs::File;
use std::io::{BufWriter, Write};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use fast_lio::data_source::{DataSource, SimParams, SimSource};
use fast_lio::laser_mapping::{LaserMapping, LioConfig, LioResult};
use fast_lio::types::{LidarType, PointType, SensorData, TimeUnit};
use fast_lio_driver::{open, DriverParams};

#[derive(Clone, Copy, PartialEq)]
enum MapFormat {
    Xyz,
    Pcd,
    Ply,
}

impl MapFormat {
    fn parse(s: &str) -> Option<Self> {
        match s {
            "xyz" => Some(Self::Xyz),
            "pcd" => Some(Self::Pcd),
            "ply" => Some(Self::Ply),
            _ => None,
        }
    }

    fn extension(self) -> &'static str {
        match self {
            Self::Xyz => "xyz",
            Self::Pcd => "pcd",
            Self::Ply => "ply",
        }
    }
}

fn usage() -> ! {
    eprintln!(
        "usage: fast-lio-app [common opts] --sim | --driver <name> [driver opts]\n\
         \n\
         common opts:\n\
         \x20 --out <dir>              output directory (default \"out\")\n\
         \x20 --out-format <fmt>       map file format: xyz | pcd | ply (default pcd)\n\
         \x20 --scan-ms <ms>           scan frame period in ms (default 100)\n\
         \x20 --duration <secs>        auto-stop after N seconds and save (default: run until Ctrl-C)\n\
         \x20 --map-voxel <m>           global map voxel size (default 0.5; smaller = denser)\n\
         \x20 --surf-voxel <m>          per-frame scan voxel size (default 0.5)\n\
         \x20 --point-filter-num <n>    keep every Nth point (default 2; 1 = keep all)\n\
         \n\
         modes:\n\
         \x20 --sim                    synthetic demo data (default)\n\
         \x20 --driver <name>          connect to a real LiDAR. Supported names:\n\
         \x20   livox                  Livox (HAP / Mid-360) via SDK2, needs --config\n\
         \x20   velodyne | ouster | hesai | marsim   spinning LiDAR (adapter may be WIP)\n\
         \n\
         driver opts:\n\
         \x20 --config <file>          vendor config file (Livox SDK2 JSON)\n\
         \x20 --ip <addr>              LiDAR network address (spinning LiDARs)\n\
         \x20 --port <port>            UDP data port (spinning LiDARs)\n\
         \n\
         examples:\n\
         \x20 fast-lio-app --sim\n\
         \x20 fast-lio-app --driver livox --config mid360_config.json --duration 120\n\
         \x20 fast-lio-app --driver velodyne --ip 192.168.1.100 --port 2368"
    );
    std::process::exit(2);
}

fn main() {
    let mut out_dir = "out".to_string();
    let mut out_format = MapFormat::Pcd;
    let mut driver_name: Option<String> = None;
    let mut config_path: Option<String> = None;
    let mut udp_ip: Option<String> = None;
    let mut udp_port: Option<u16> = None;
    let mut scan_ms: f64 = 100.0;
    let mut duration: Option<f64> = None;
    let mut map_voxel: Option<f32> = None;
    let mut surf_voxel: Option<f32> = None;
    let mut point_filter_num: Option<i32> = None;
    let mut sim = false;

    let mut args = std::env::args().skip(1);
    while let Some(a) = args.next() {
        match a.as_str() {
            "--out" => out_dir = args.next().unwrap_or_else(|| usage()),
            "--out-format" => {
                out_format = args
                    .next()
                    .and_then(|s| MapFormat::parse(&s))
                    .unwrap_or_else(|| usage())
            }
            "--duration" => {
                duration = args
                    .next()
                    .and_then(|v| v.parse().ok())
                    .filter(|d| *d > 0.0);
                if duration.is_none() {
                    eprintln!("--duration expects seconds (e.g. --duration 120)");
                    usage();
                }
            }
            "--sim" => sim = true,
            // backwards-compatible alias: --live <config> == --driver livox --config <config>
            "--live" => {
                driver_name = Some("livox".to_string());
                config_path = Some(args.next().unwrap_or_else(|| usage()));
            }
            "--driver" => driver_name = Some(args.next().unwrap_or_else(|| usage())),
            "--config" => config_path = Some(args.next().unwrap_or_else(|| usage())),
            "--ip" => udp_ip = Some(args.next().unwrap_or_else(|| usage())),
            "--port" => {
                udp_port = Some(
                    args.next()
                        .unwrap_or_else(|| usage())
                        .parse()
                        .unwrap_or_else(|_| usage()),
                )
            }
            "--scan-ms" => scan_ms = args.next().unwrap_or_else(|| usage()).parse().unwrap_or_else(|_| usage()),
            "--map-voxel" => {
                map_voxel = args
                    .next()
                    .and_then(|v| v.parse().ok())
                    .filter(|v| *v > 0.0);
                if map_voxel.is_none() {
                    eprintln!("--map-voxel expects meters (e.g. --map-voxel 0.1)");
                    usage();
                }
            }
            "--surf-voxel" => {
                surf_voxel = args
                    .next()
                    .and_then(|v| v.parse().ok())
                    .filter(|v| *v > 0.0);
                if surf_voxel.is_none() {
                    eprintln!("--surf-voxel expects meters (e.g. --surf-voxel 0.1)");
                    usage();
                }
            }
            "--point-filter-num" => {
                point_filter_num = args
                    .next()
                    .and_then(|v| v.parse().ok())
                    .filter(|v| *v > 0);
                if point_filter_num.is_none() {
                    eprintln!("--point-filter-num expects an integer >= 1 (1 = keep all)");
                    usage();
                }
            }
            _ => usage(),
        }
    }
    if sim && driver_name.is_some() {
        usage();
    }
    std::fs::create_dir_all(&out_dir).expect("create output dir");

    // ---- pipeline configuration -----------------------------------------
    let lidar_type = match driver_name.as_deref() {
        None => LidarType::Velo16,
        Some(name) => lidar_type_from_driver(name).unwrap_or_else(|e| {
            eprintln!("error: {e}");
            std::process::exit(2);
        }),
    };
    // density overrides: smaller map/surf voxels or a smaller point-filter
    // interval produce a denser saved map
    let filter_size_map = map_voxel.unwrap_or(0.5);
    let filter_size_surf = surf_voxel.unwrap_or(0.5);
    let point_filter_num = point_filter_num.unwrap_or(2);
    let cfg = match lidar_type {
        LidarType::Avia => {
            // Direct odometry mode (no feature extraction): robust for Livox and
            // required because the SDK2 stream is routed through a single scan
            // line (no per-point ring information).
            LioConfig {
                lidar_type: LidarType::Avia,
                feature_extract_enable: false,
                point_filter_num,
                n_scans: 6,
                scan_rate: 10,
                timestamp_unit: TimeUnit::Us,
                filter_size_surf,
                filter_size_map,
                gyr_cov: 0.1,
                acc_cov: 0.1,
                b_gyr_cov: 0.0001,
                b_acc_cov: 0.0001,
                ..Default::default()
            }
        }
        _ => {
            // spinning LiDAR: per-point ring available, feature extraction
            // can be enabled; timestamps in milliseconds
            LioConfig {
                lidar_type,
                feature_extract_enable: false,
                point_filter_num,
                n_scans: 16,
                scan_rate: 10,
                timestamp_unit: TimeUnit::Ms,
                filter_size_surf,
                filter_size_map,
                gyr_cov: 0.1,
                acc_cov: 0.1,
                b_gyr_cov: 0.0001,
                b_acc_cov: 0.0001,
                ..Default::default()
            }
        }
    };
    println!(
        "density: filter_size_map={filter_size_map}, filter_size_surf={filter_size_surf}, point_filter_num={point_filter_num}"
    );

    // ---- data source ----------------------------------------------------
    let mut source: Box<dyn DataSource> = if let Some(name) = driver_name {
        let lidar_type = lidar_type_from_driver(&name).unwrap_or_else(|e| {
            eprintln!("error: {e}");
            std::process::exit(2);
        });
        let params = DriverParams::new(
            lidar_type,
            config_path,
            udp_ip,
            udp_port,
            Duration::from_secs_f64(scan_ms / 1000.0),
        );
        println!(
            "opening {name} driver: config={:?}, ip={:?}, port={:?}, scan={} ms ...",
            params.config_path,
            params.udp_ip,
            params.udp_port,
            scan_ms as i64
        );
        open(&params)
            .unwrap_or_else(|e| panic!("failed to open the {name} driver: {e}"))
    } else {
        let _ = scan_ms;
        let sim = SimParams {
            imu_hz: 200.0,
            lidar_hz: 10.0,
            duration: 20.0,
            radius: 5.0,
            omega: 0.15,
            height: 1.0,
            points_per_scan: 1500,
            ..Default::default()
        };
        Box::new(SimSource::new(&sim))
    };

    let mut mapping = LaserMapping::new(&cfg);

    let mut results: Vec<LioResult> = Vec::new();
    let mut n_sensor = 0u64;
    let mut n_frames = 0u64;
    let mut n_skipped = 0u64;

    // Ctrl-C -> graceful stop & save (like the C++ node).
    let done = Arc::new(AtomicBool::new(false));
    ctrlc::set_handler({
        let done = done.clone();
        move || done.store(true, Ordering::SeqCst)
    })
    .expect("install Ctrl-C handler");
    if let Some(d) = duration {
        println!("auto-stop: saving after {d} s (or Ctrl-C to stop now)");
    } else {
        println!("running ... press Ctrl-C to stop and save the map");
    }

    let t0 = Instant::now();
    let mut last_progress = Instant::now();
    loop {
        let timed_out = duration.is_some_and(|d| t0.elapsed().as_secs_f64() >= d);
        if done.load(Ordering::SeqCst) || timed_out {
            break;
        }
        match source.try_next() {
            Ok(Some(data)) => {
                n_sensor += 1;
                match data {
                    SensorData::Imu(imu) => mapping.add_imu(&imu),
                    SensorData::LidarAvia(msg) => mapping.add_lidar_avia(&msg),
                    SensorData::LidarStandard(msg) => mapping.add_lidar_standard(&msg),
                }
                if mapping.has_data() {
                    if let Some(res) = mapping.run_once() {
                        results.push(res);
                        n_frames += 1;
                    } else {
                        n_skipped += 1;
                    }
                }
            }
            Ok(None) => break,
            Err(_) => {
                // nothing available right now: keep polling for Ctrl-C / timeout
                if last_progress.elapsed() >= Duration::from_secs(2) {
                    last_progress = Instant::now();
                    println!(
                        "received {n_sensor} samples, processed {n_frames} frames, skipped {n_skipped}"
                    );
                }
                std::thread::sleep(Duration::from_millis(20));
            }
        }
    }

    println!(
        "sensor samples: {n_sensor}, processed frames: {n_frames}, skipped: {n_skipped}"
    );

    // write trajectory (pos_log format, similar to the C++ node)
    let traj_path = format!("{out_dir}/pos_log.txt");
    {
        let f = File::create(&traj_path).expect("open trajectory");
        let mut w = BufWriter::new(f);
        for r in &results {
            let rot = quat_to_euler(&r.quat);
            writeln!(
                w,
                "{:.6} {:.6} {:.6} {:.6} {:.6} {:.6} {:.6} {:.6} {:.6} {:.6} {:.6} {:.6} {:.6} {:.6} {:.6} {:.6} {:.6} {:.6} {:.6}",
                r.time - results[0].time,
                rot[0], rot[1], rot[2],
                r.pos[0], r.pos[1], r.pos[2],
                0.0, 0.0, 0.0,
                r.vel[0], r.vel[1], r.vel[2],
                0.0, 0.0, 0.0,
                r.bg[0], r.bg[1], r.bg[2],
            )
            .expect("write traj");
        }
    }
    println!("trajectory -> {traj_path} ({} poses)", results.len());

    // write the map (world frame points from the kd-tree)
    mapping.ikdtree.flatten_to_storage();
    let map_points = mapping.ikdtree.pcl_storage.clone();
    let map_path = format!("{out_dir}/map.{}", out_format.extension());
    write_map(&map_path, &map_points, out_format);
    println!("map -> {map_path} ({} points)", map_points.len());

    // print a few sanity numbers
    if let Some(last) = results.last() {
        println!(
            "final pos=({:.2},{:.2},{:.2}) |pos|={:.2}, map points={}, res_mean={:.4}",
            last.pos[0],
            last.pos[1],
            last.pos[2],
            last.pos.norm(),
            last.map_points,
            last.res_mean
        );
    }
}

/// Convert a CLI driver name into a [`LidarType`]. Unknown names are rejected
/// with an actionable error instead of being silently mapped to a wrong driver.
fn lidar_type_from_driver(name: &str) -> Result<LidarType, String> {
    match name.to_ascii_lowercase().as_str() {
        "livox" | "avia" | "hap" | "mid360" | "mid-360" => Ok(LidarType::Avia),
        "velodyne" | "velo16" => Ok(LidarType::Velo16),
        "ouster" | "oust64" => Ok(LidarType::Oust64),
        "marsim" => Ok(LidarType::Marsim),
        "hesai" => Err("the hesai driver is not implemented yet (add an adapter in fast-lio-driver)".to_string()),
        other => Err(format!(
            "unknown driver '{other}' — supported: livox, velodyne, ouster, hesai, marsim"
        )),
    }
}

/// Convert a (w,x,y,z) quaternion to Euler angles in degrees (roll/pitch/yaw).
fn quat_to_euler(q: &[f64; 4]) -> [f64; 3] {
    let (w, x, y, z) = (q[0], q[1], q[2], q[3]);
    let sqw = w * w;
    let sqx = x * x;
    let sqy = y * y;
    let sqz = z * z;
    let unit = sqx + sqy + sqz + sqw;
    let test = w * y - z * x;
    if test > 0.49999 * unit {
        return [2.0 * x.atan2(w) * 57.3, 90.0, 0.0];
    }
    if test < -0.49999 * unit {
        return [-2.0 * x.atan2(w) * 57.3, -90.0, 0.0];
    }
    [
        (2.0 * x * w + 2.0 * y * z).atan2(-sqx - sqy + sqz + sqw) * 57.3,
        (2.0 * test / unit).asin() * 57.3,
        (2.0 * z * w + 2.0 * y * x).atan2(sqx - sqy - sqz + sqw) * 57.3,
    ]
}

/// Write the map point cloud in the requested format.
fn write_map(path: &str, points: &[PointType], fmt: MapFormat) {
    let f = File::create(path).expect("open map");
    let mut w = BufWriter::new(f);
    match fmt {
        MapFormat::Xyz => {
            for p in points {
                writeln!(w, "{} {} {}", p.x, p.y, p.z).expect("write map");
            }
        }
        MapFormat::Pcd => {
            writeln!(w, "# .PCD v0.7 - Point Cloud Data file format").expect("write map");
            writeln!(w, "VERSION 0.7").expect("write map");
            writeln!(w, "FIELDS x y z intensity").expect("write map");
            writeln!(w, "SIZE 4 4 4 4").expect("write map");
            writeln!(w, "TYPE F F F F").expect("write map");
            writeln!(w, "COUNT 1 1 1 1").expect("write map");
            writeln!(w, "WIDTH {}", points.len()).expect("write map");
            writeln!(w, "HEIGHT 1").expect("write map");
            writeln!(w, "VIEWPOINT 0 0 0 1 0 0 0").expect("write map");
            writeln!(w, "POINTS {}", points.len()).expect("write map");
            writeln!(w, "DATA ascii").expect("write map");
            for p in points {
                writeln!(w, "{} {} {} {}", p.x, p.y, p.z, p.intensity).expect("write map");
            }
        }
        MapFormat::Ply => {
            writeln!(w, "ply").expect("write map");
            writeln!(w, "format ascii 1.0").expect("write map");
            writeln!(w, "element vertex {}", points.len()).expect("write map");
            writeln!(w, "property float x").expect("write map");
            writeln!(w, "property float y").expect("write map");
            writeln!(w, "property float z").expect("write map");
            writeln!(w, "property float intensity").expect("write map");
            writeln!(w, "end_header").expect("write map");
            for p in points {
                writeln!(w, "{} {} {} {}", p.x, p.y, p.z, p.intensity).expect("write map");
            }
        }
    }
}
