use std::fs::File;
use std::io::{BufWriter, Write};
use std::time::Duration;

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
        "usage: fast-lio-app [--out <dir>] [--out-format <xyz|pcd|ply>] [--sim] | [--live <config.json> [--scan-ms <ms>]]\n\
         \n\
         modes:\n\
         \x20 --sim                    synthetic demo data (default)\n\
         \x20 --live <config.json>     connect to a Livox LiDAR via the SDK2 (no ROS)\n\
         \x20   --scan-ms <ms>         scan frame period in ms (default 100)\n\
         \x20 --out <dir>              output directory (default \"out\")\n\
         \x20 --out-format <fmt>       map file format: xyz | pcd | ply (default xyz)"
    );
    std::process::exit(2);
}

fn main() {
    let mut out_dir = "out".to_string();
    let mut out_format = MapFormat::Xyz;
    let mut live_config: Option<String> = None;
    let mut scan_ms: f64 = 100.0;
    let mut sim = false;

    let mut args = std::env::args().skip(1);
    while let Some(a) = args.next() {
        match a.as_str() {
            "--out" => out_dir = args.next().unwrap_or_else(|| usage()),
            "--out-format" => {
                out_format = args
                    .next()
                    .map(|s| MapFormat::parse(&s))
                    .flatten()
                    .unwrap_or_else(|| usage())
            }
            "--sim" => sim = true,
            "--live" => live_config = Some(args.next().unwrap_or_else(|| usage())),
            "--scan-ms" => scan_ms = args.next().unwrap_or_else(|| usage()).parse().unwrap_or_else(|_| usage()),
            _ => usage(),
        }
    }
    if live_config.is_some() && sim {
        usage();
    }
    std::fs::create_dir_all(&out_dir).expect("create output dir");

    // ---- pipeline configuration -----------------------------------------
    let cfg = if live_config.is_some() {
        // Direct odometry mode (no feature extraction): robust for Livox and
        // required here because the SDK2 stream is routed through a single scan
        // line (no per-point ring information).
        LioConfig {
            lidar_type: LidarType::Avia,
            feature_extract_enable: false,
            point_filter_num: 2,
            n_scans: 6,
            scan_rate: 10,
            timestamp_unit: TimeUnit::Us,
            filter_size_surf: 0.5,
            filter_size_map: 0.5,
            gyr_cov: 0.1,
            acc_cov: 0.1,
            b_gyr_cov: 0.0001,
            b_acc_cov: 0.0001,
            ..Default::default()
        }
    } else {
        // synthetic demo: spinning-lidar style data
        LioConfig {
            lidar_type: LidarType::Velo16,
            feature_extract_enable: false,
            point_filter_num: 2,
            n_scans: 16,
            timestamp_unit: TimeUnit::Ms,
            filter_size_surf: 0.5,
            filter_size_map: 0.5,
            gyr_cov: 0.1,
            acc_cov: 0.1,
            b_gyr_cov: 0.0001,
            b_acc_cov: 0.0001,
            ..Default::default()
        }
    };

    // ---- data source ----------------------------------------------------
    let mut source: Box<dyn DataSource> = if let Some(config) = live_config {
        println!("connecting to Livox device via SDK2 (config: {config}) ...");
        let params = DriverParams::livox(config, Duration::from_secs_f64(scan_ms / 1000.0));
        open(&params)
            .expect("failed to open the lidar driver — check the config file and the network")
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

    while let Some(data) = source.next() {
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
