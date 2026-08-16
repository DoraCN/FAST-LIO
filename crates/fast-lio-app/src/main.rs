use std::fs::File;
use std::io::{BufWriter, Write};

use fast_lio::consts::G_M_S2;
use fast_lio::data_source::{DataSource, SimParams, SimSource};
use fast_lio::laser_mapping::{LaserMapping, LioConfig, LioResult};
use fast_lio::types::{LidarType, SensorData, TimeUnit};

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let out_dir = args
        .get(1)
        .cloned()
        .unwrap_or_else(|| "out".to_string());
    std::fs::create_dir_all(&out_dir).expect("create output dir");

    let cfg = LioConfig {
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
    };

    let mut mapping = LaserMapping::new(&cfg);
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
    let mut source = SimSource::new(&sim);

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

    println!("sensor samples: {n_sensor}, processed frames: {n_frames}, skipped: {n_skipped}");

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
    let map_path = format!("{out_dir}/map.xyz");
    {
        mapping.ikdtree.flatten_to_storage();
        let f = File::create(&map_path).expect("open map");
        let mut w = BufWriter::new(f);
        for p in &mapping.ikdtree.pcl_storage {
            writeln!(w, "{} {} {}", p.x, p.y, p.z).expect("write map");
        }
    }
    println!("map -> {map_path} ({} points)", mapping.ikdtree.pcl_storage.len());

    // print a few sanity numbers
    if let Some(last) = results.last() {
        let r = last.pos.norm();
        println!(
            "final pos=({:.2},{:.2},{:.2}) |pos|={:.2}, map points={}, res_mean={:.4}",
            last.pos[0], last.pos[1], last.pos[2], r, last.map_points, last.res_mean
        );
        println!("gravity = 9.81 (ref)");
        let _ = G_M_S2;
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
