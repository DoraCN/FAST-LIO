//! Build a 2D occupancy grid (PGM + PNG + YAML) from a 3D point cloud / PCD.
//!
//! Usage:
//! ```sh
//! cargo run --release -p lidar-map --example to_2d_grid -- \
//!     --input map.pcd --height 0.0 --band 0.3 --resolution 0.05 --output map
//! ```
//!
//! Options:
//! - `--height` / `--band` : horizontal slice around the lidar mounting height.
//! - `--z-min` / `--z-max` : explicit slice bounds (override height/band).
//! - `--resolution`        : grid cell size in meters.
//! - `--max-range`         : drop points farther than N m from the origin.
//! - `--min-range`         : drop points closer than N m (robot's own structure).
//! - `--output`            : output basename (writes `map.pgm`, `map.png`, `map.yaml`).
//!
//! Output conventions match ROS `map_server`: PGM row 0 = north, `0` occupied,
//! `255` free, `127` unknown.

use lidar_map::{GridMap, GridMapParams};
use std::fs::File;
use std::io::{BufRead, BufReader, Read};
use std::path::Path;

fn read_scalar(b: &[u8], ty: &str) -> f64 {
    match ty {
        "F" => f32::from_le_bytes(b.try_into().unwrap()) as f64,
        "D" => f64::from_le_bytes(b.try_into().unwrap()),
        "U" if b.len() == 1 => b[0] as f64,
        "U" if b.len() == 2 => u16::from_le_bytes(b.try_into().unwrap()) as f64,
        "U" if b.len() == 4 => u32::from_le_bytes(b.try_into().unwrap()) as f64,
        "I" if b.len() == 1 => b[0] as i8 as f64,
        "I" if b.len() == 2 => i16::from_le_bytes(b.try_into().unwrap()) as f64,
        "I" if b.len() == 4 => i32::from_le_bytes(b.try_into().unwrap()) as f64,
        _ => 0.0,
    }
}

fn parse_pcd(path: &Path) -> Result<Vec<[f64; 3]>, String> {
    let mut f = BufReader::new(File::open(path).map_err(|e| e.to_string())?);
    let mut header: Vec<String> = Vec::new();
    let mut line = String::new();
    loop {
        line.clear();
        let n = f.read_line(&mut line).map_err(|e| e.to_string())?;
        if n == 0 {
            return Err("unexpected EOF in PCD header".into());
        }
        let t = line.trim().to_string();
        let is_data = t.starts_with("DATA");
        header.push(t);
        if is_data {
            break;
        }
    }
    let fields: Vec<String> = header
        .iter()
        .find_map(|l| l.strip_prefix("FIELDS").map(|r| {
            r.split_whitespace().map(|s| s.to_string()).collect()
        }))
        .ok_or("no FIELDS line")?;
    let size: Vec<usize> = header
        .iter()
        .find_map(|l| l.strip_prefix("SIZE").map(|r| {
            r.split_whitespace().filter_map(|s| s.parse().ok()).collect()
        }))
        .ok_or("no SIZE line")?;
    let ty: Vec<String> = header
        .iter()
        .find_map(|l| l.strip_prefix("TYPE").map(|r| {
            r.split_whitespace().map(|s| s.to_string()).collect()
        }))
        .ok_or("no TYPE line")?;
    let points: usize = header
        .iter()
        .find_map(|l| l.strip_prefix("POINTS").and_then(|r| r.trim().parse().ok()))
        .ok_or("no POINTS line")?;
    let data = header.iter().rev().find_map(|l| l.strip_prefix("DATA").map(str::trim)).unwrap_or("");
    if fields.len() != size.len() || fields.len() != ty.len() {
        return Err("PCD FIELDS/SIZE/TYPE mismatch".into());
    }
    let ix = fields.iter().position(|f| f == "x").ok_or("no x field")?;
    let iy = fields.iter().position(|f| f == "y").ok_or("no y field")?;
    let iz = fields.iter().position(|f| f == "z").ok_or("no z field")?;
    let stride: usize = size.iter().sum();

    let mut xyz = Vec::with_capacity(points);
    match data {
        "binary" => {
            let mut buf = vec![0u8; stride * points];
            f.read_exact(&mut buf).map_err(|e| e.to_string())?;
            for p in 0..points {
                let off = p * stride;
                let mut acc = 0usize;
                let mut px = 0.0;
                let mut py = 0.0;
                let mut pz = 0.0;
                for (fi, sz) in size.iter().enumerate() {
                    let v = read_scalar(&buf[off + acc..off + acc + sz], &ty[fi]);
                    acc += sz;
                    if fi == ix {
                        px = v;
                    } else if fi == iy {
                        py = v;
                    } else if fi == iz {
                        pz = v;
                    }
                }
                xyz.push([px, py, pz]);
            }
        }
        "ascii" => {
            for _ in 0..points {
                line.clear();
                f.read_line(&mut line).map_err(|e| e.to_string())?;
                let vals: Vec<f64> = line.split_whitespace().filter_map(|s| s.parse().ok()).collect();
                if vals.len() > iz {
                    xyz.push([vals[ix], vals[iy], vals[iz]]);
                }
            }
        }
        other => return Err(format!("unsupported DATA mode: {other}")),
    }
    Ok(xyz)
}

fn main() -> Result<(), String> {
    let mut input = "map.pcd".to_string();
    let mut height = 0.0f64;
    let mut band = 0.3f64;
    let mut z_min: Option<f64> = None;
    let mut z_max: Option<f64> = None;
    let mut res = 0.05f64;
    let mut output = "map".to_string();
    let mut max_range: Option<f64> = None;
    let mut min_range: Option<f64> = None;
    let mut pad = 2.0f64;

    let mut args = std::env::args().skip(1);
    while let Some(a) = args.next() {
        match a.as_str() {
            "--input" => input = args.next().ok_or("--input needs a value")?,
            "--height" => height = args.next().and_then(|v| v.parse().ok()).ok_or("bad --height")?,
            "--band" => band = args.next().and_then(|v| v.parse().ok()).ok_or("bad --band")?,
            "--z-min" => z_min = Some(args.next().and_then(|v| v.parse().ok()).ok_or("bad --z-min")?),
            "--z-max" => z_max = Some(args.next().and_then(|v| v.parse().ok()).ok_or("bad --z-max")?),
            "--resolution" => res = args.next().and_then(|v| v.parse().ok()).ok_or("bad --resolution")?,
            "--output" => output = args.next().ok_or("--output needs a value")?,
            "--max-range" => max_range = Some(args.next().and_then(|v| v.parse().ok()).ok_or("bad --max-range")?),
            "--min-range" => min_range = Some(args.next().and_then(|v| v.parse().ok()).ok_or("bad --min-range")?),
            "--pad" => pad = args.next().and_then(|v| v.parse().ok()).ok_or("bad --pad")?,
            s if s.starts_with('-') => return Err(format!("unknown flag: {s}")),
            s => return Err(format!("unexpected positional argument: {s}")),
        }
    }

    println!("reading {input} ...");
    let cloud = parse_pcd(Path::new(&input))?;
    println!("total points: {}", cloud.len());

    let (lo, hi) = match (z_min, z_max) {
        (Some(lo), Some(hi)) => (lo, hi),
        (Some(lo), None) => (lo, lo + band),
        (None, Some(hi)) => (hi - band, hi),
        (None, None) => (height - band / 2.0, height + band / 2.0),
    };
    println!("slice z in [{lo:.3}, {hi:.3}]");

    let mut xs: Vec<f64> = Vec::new();
    let mut ys: Vec<f64> = Vec::new();
    for p in &cloud {
        if p[2] < lo || p[2] > hi {
            continue;
        }
        let r = p[0].hypot(p[1]);
        if max_range.is_some_and(|mr| r > mr) || min_range.is_some_and(|mr| r < mr) {
            continue;
        }        xs.push(p[0]);
        ys.push(p[1]);
    }
    println!("points in band: {}", xs.len());
    if xs.is_empty() {
        return Err("no points in the height band; check --height / --band / --z-min / --z-max".into());
    }

    let min_x = xs.iter().cloned().fold(f64::INFINITY, f64::min) - pad;
    let max_x = xs.iter().cloned().fold(f64::NEG_INFINITY, f64::max) + pad;
    let min_y = ys.iter().cloned().fold(f64::INFINITY, f64::min) - pad;
    let max_y = ys.iter().cloned().fold(f64::NEG_INFINITY, f64::max) + pad;

    let params = GridMapParams {
        resolution: res,
        min_x,
        min_y,
        max_x,
        max_y,
        ..Default::default()
    };
    let mut grid = GridMap::new(params);
    // treat the projected hit points as occupied (no sensor origin for a static
    // map, so we use the voxel-style additive update instead of ray casting)
    let pts: Vec<lidar_map::Point3> = xs.iter().zip(ys.iter()).map(|(&x, &y)| [x, y, lo]).collect();
    grid.update_from_cloud([0.0, 0.0], &pts);

    let (ncols, nrows) = grid.dims();
    let max_x2 = min_x + ncols as f64 * res;
    let max_y2 = min_y + nrows as f64 * res;
    println!(
        "map bbox x:[{min_x:.2},{max_x2:.2}] y:[{min_y:.2},{max_y2:.2}] grid {ncols}x{nrows} @ {res} m"
    );

    let pgm_path = format!("{output}.pgm");
    grid.save_pgm(&pgm_path).map_err(|e| e.to_string())?;
    println!("wrote {pgm_path}");

    let png_path = format!("{output}.png");
    grid.save_png(&png_path)?;
    println!("wrote {png_path}");

    let yaml_path = format!("{output}.yaml");
    grid.save_yaml(&yaml_path, &pgm_path).map_err(|e| e.to_string())?;
    println!("wrote {yaml_path}");

    println!("occupied cells: {}", grid.cell_count());
    Ok(())
}
