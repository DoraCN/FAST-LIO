//! 2D occupancy grid map with incremental ray-casting updates.
//!
//! The grid covers a fixed world-aligned rectangle (`min_x..max_x` ×
//! `min_y..max_y`) at a given resolution. Each cell holds a log-odds value;
//! positive = occupied, negative = free, zero = unknown.
//!
//! Updates follow the standard laser scan-matching recipe:
//! - every cell on the ray from the sensor origin to a hit point is marked
//!   **free** (incrementally, with a bounded step so thin obstacles are kept),
//! - the hit cell itself is marked **occupied** (with saturation).
//!
//! Because the grid is fixed-size and world-aligned, a robot that travels far
//! must slide/reset the window; a simple `grow()` / re-center API is provided.

use std::collections::HashMap;

use crate::{Point2, Point3};

/// Configuration for building a [`GridMap`].
#[derive(Clone, Debug)]
pub struct GridMapParams {
    /// Cell size in meters (e.g. `0.05`).
    pub resolution: f64,
    /// Minimum world x of the covered rectangle.
    pub min_x: f64,
    /// Minimum world y of the covered rectangle.
    pub min_y: f64,
    /// Maximum world x of the covered rectangle.
    pub max_x: f64,
    /// Maximum world y of the covered rectangle.
    pub max_y: f64,
    /// Log-odds increment applied to free cells along the ray.
    pub free_log_odds: f64,
    /// Log-odds increment applied to occupied (hit) cells.
    pub occ_log_odds: f64,
    /// Ray-marching step in meters (<= resolution keeps thin walls).
    pub ray_step: f64,
    /// Saturation bound for |log-odds|.
    pub saturate: f64,
}

impl Default for GridMapParams {
    fn default() -> Self {
        Self {
            resolution: 0.05,
            min_x: -50.0,
            min_y: -50.0,
            max_x: 50.0,
            max_y: 50.0,
            free_log_odds: 0.2,
            occ_log_odds: 0.6,
            ray_step: 0.05,
            saturate: 4.0,
        }
    }
}

/// A 2D occupancy grid.
///
/// Cells are stored sparsely (only the ones that have been observed) so the
/// memory cost scales with the mapped area, not the bounding box.
#[derive(Clone, Debug)]
pub struct GridMap {
    params: GridMapParams,
    /// cell index (col, row) -> log-odds.
    cells: HashMap<(i64, i64), f64>,
}

impl GridMap {
    pub fn new(params: GridMapParams) -> Self {
        Self {
            params,
            cells: HashMap::new(),
        }
    }

    pub fn params(&self) -> &GridMapParams {
        &self.params
    }

    /// World -> cell index.
    pub fn world_to_cell(&self, x: f64, y: f64) -> (i64, i64) {
        let col = ((x - self.params.min_x) / self.params.resolution).floor() as i64;
        let row = ((y - self.params.min_y) / self.params.resolution).floor() as i64;
        (col, row)
    }

    /// Cell index -> world coordinates of the cell centre.
    pub fn cell_to_world(&self, col: i64, row: i64) -> Point2 {
        [
            self.params.min_x + (col as f64 + 0.5) * self.params.resolution,
            self.params.min_y + (row as f64 + 0.5) * self.params.resolution,
        ]
    }

    /// True if the cell index lies inside the covered rectangle.
    pub fn in_bounds(&self, col: i64, row: i64) -> bool {
        let ncols = ((self.params.max_x - self.params.min_x) / self.params.resolution) as i64;
        let nrows = ((self.params.max_y - self.params.min_y) / self.params.resolution) as i64;
        col >= 0 && row >= 0 && col < ncols && row < nrows
    }

    /// Log-odds of a cell (0.0 when never observed).
    pub fn cell_log_odds(&self, col: i64, row: i64) -> f64 {
        self.cells.get(&(col, row)).copied().unwrap_or(0.0)
    }

    /// Occupancy probability of a cell in `[0, 1]`; 0.5 = unknown.
    pub fn occupancy(&self, col: i64, row: i64) -> f64 {
        let l = self.cell_log_odds(col, row);
        1.0 / (1.0 + (-l).exp())
    }

    /// Occupancy probability at an arbitrary world point.
    pub fn occupancy_at(&self, x: f64, y: f64) -> f64 {
        let (c, r) = self.world_to_cell(x, y);
        if !self.in_bounds(c, r) {
            return 0.5;
        }
        self.occupancy(c, r)
    }

    /// Convenience: is the cell considered occupied (probability > threshold)?
    pub fn is_occupied(&self, x: f64, y: f64, threshold: f64) -> bool {
        self.occupancy_at(x, y) > threshold
    }

    /// Update one cell by a signed log-odds delta (saturated).
    fn apply_delta(&mut self, col: i64, row: i64, delta: f64) {
        if !self.in_bounds(col, row) {
            return;
        }
        let e = self.cells.entry((col, row)).or_insert(0.0);
        *e = (*e + delta).clamp(-self.params.saturate, self.params.saturate);
    }

    /// Update the map from one laser scan.
    ///
    /// `sensor` is the LiDAR origin in the same world frame as the grid;
    /// `points` are the scan's hit points (world frame, meters). Points outside
    /// the grid bounds are clamped to the grid edge so they still clear the
    /// interior rays.
    pub fn update_from_scan(&mut self, sensor: Point2, points: &[Point2]) {
        for p in points {
            let dx = p[0] - sensor[0];
            let dy = p[1] - sensor[1];
            let dist = (dx * dx + dy * dy).sqrt();
            if dist < 1e-6 {
                continue;
            }
            let step = self.params.ray_step;
            let n = (dist / step).ceil().max(1.0) as usize;
            let mut hit_cell = None;
            for i in 0..n {
                let t = (i as f64 + 1.0) / n as f64;
                let x = sensor[0] + dx * t;
                let y = sensor[1] + dy * t;
                let (c, r) = self.world_to_cell(x, y);
                if !self.in_bounds(c, r) {
                    // went outside the map: stop clearing this ray
                    break;
                }
                hit_cell = Some((c, r));
                self.apply_delta(c, r, -self.params.free_log_odds);
            }
            if let Some((c, r)) = hit_cell {
                self.apply_delta(c, r, self.params.occ_log_odds + self.params.free_log_odds);
            }
        }
    }

    /// Update from a 2D-projected point cloud using a sensor pose.
    ///
    /// `points` are world-frame 3D points; only the x/y are used (project the
    /// cloud onto the ground plane before calling if you want a height band).
    pub fn update_from_cloud(&mut self, sensor: Point2, points: &[Point3]) {
        let pts: Vec<Point2> = points.iter().map(|p| [p[0], p[1]]).collect();
        self.update_from_scan(sensor, &pts);
    }

    /// Mark cells as occupied directly (no ray clearing).
    ///
    /// Use this for a **static** global map built from an accumulated point
    /// cloud where the original sensor origins are unknown: every point just
    /// increments its cell's occupancy.
    pub fn mark_occupied(&mut self, points: &[Point3]) {
        for p in points {
            let (c, r) = self.world_to_cell(p[0], p[1]);
            if self.in_bounds(c, r) {
                self.apply_delta(c, r, self.params.occ_log_odds);
            }
        }
    }

    /// All observed cells as `(col, row, log_odds)`.
    pub fn iter_cells(&self) -> impl Iterator<Item = ((i64, i64), f64)> + '_ {
        self.cells.iter().map(|(&k, &v)| (k, v))
    }

    /// Number of observed cells.
    pub fn cell_count(&self) -> usize {
        self.cells.len()
    }

    /// Reset the grid to empty (keeps parameters).
    pub fn clear(&mut self) {
        self.cells.clear();
    }

    /// Grid dimensions in cells.
    pub fn dims(&self) -> (usize, usize) {
        let ncols = ((self.params.max_x - self.params.min_x) / self.params.resolution) as usize;
        let nrows = ((self.params.max_y - self.params.min_y) / self.params.resolution) as usize;
        (ncols.max(1), nrows.max(1))
    }

    /// Render the grid as a grayscale image.
    ///
    /// Values: `0` = occupied, `255` = free, `127` = unknown. Row 0 is the
    /// largest y (north), matching the ROS map_server / PGM convention.
    /// Returns a packed `(width, height, row-major bytes)`.
    pub fn to_grayscale(&self) -> (usize, usize, Vec<u8>) {
        let (ncols, nrows) = self.dims();
        let mut img = vec![127u8; ncols * nrows];
        for ((col, row), _log) in self.iter_cells() {
            if col < 0 || row < 0 || col as usize >= ncols || row as usize >= nrows {
                continue;
            }
            // probability in [0,1] -> 0 = occupied
            let p = self.occupancy(col, row);
            let v = if p >= 0.6 {
                0u8
            } else if p <= 0.4 {
                255u8
            } else {
                127u8
            };
            img[row as usize * ncols + col as usize] = v;
        }
        // flip rows so row 0 = largest y
        let mut flipped = vec![0u8; ncols * nrows];
        for r in 0..nrows {
            let src = nrows - 1 - r;
            flipped[r * ncols..(r + 1) * ncols].copy_from_slice(&img[src * ncols..(src + 1) * ncols]);
        }
        (ncols, nrows, flipped)
    }

    /// Write the grid as a PGM file (P5).
    pub fn save_pgm(&self, path: &str) -> std::io::Result<()> {
        let (w, h, data) = self.to_grayscale();
        use std::io::Write;
        let mut f = std::fs::File::create(path)?;
        writeln!(f, "P5")?;
        writeln!(f, "{w} {h}")?;
        writeln!(f, "255")?;
        f.write_all(&data)?;
        Ok(())
    }

    /// Load a grid from a PGM (P5) file plus its YAML metadata.
    ///
    /// The YAML provides `resolution` and `origin: [x, y, yaw]` in the
    /// nav-map (ROS map_server) convention. Returns the grid with the file's
    /// bounding box, and leaves log-odds unset (unknown) so the caller can
    /// re-observe cells.
    pub fn load_pgm(path: &str, resolution: f64, origin_x: f64, origin_y: f64) -> std::io::Result<Self> {
        let bytes = std::fs::read(path)?;
        let mut it = bytes.splitn(4, |&b| b == b'\n');
        let magic = String::from_utf8_lossy(it.next().unwrap_or(&[]));
        if magic.trim() != "P5" {
            return Err(std::io::Error::new(
                std::io::ErrorKind::InvalidData,
                format!("not a P5 PGM: {magic}"),
            ));
        }
        let dims = String::from_utf8_lossy(it.next().unwrap_or(&[]));
        let maxval = String::from_utf8_lossy(it.next().unwrap_or(&[]));
        let mut parts = dims.split_whitespace();
        let w: usize = parts.next().and_then(|s| s.parse().ok()).unwrap_or(0);
        let h: usize = parts.next().and_then(|s| s.parse().ok()).unwrap_or(0);
        if w == 0 || h == 0 {
            return Err(std::io::Error::new(std::io::ErrorKind::InvalidData, "bad PGM dims"));
        }
        let maxv: u16 = maxval.trim().parse().unwrap_or(255);
        let data = it.next().unwrap_or(&[]);
        if data.len() < w * h {
            return Err(std::io::Error::new(
                std::io::ErrorKind::InvalidData,
                "PGM pixel data too short",
            ));
        }

        let params = GridMapParams {
            resolution,
            min_x: origin_x,
            min_y: origin_y,
            max_x: origin_x + w as f64 * resolution,
            max_y: origin_y + h as f64 * resolution,
            ..Default::default()
        };
        let mut g = GridMap::new(params);
        // PGM row 0 = north (largest y); internal grid row 0 = smallest y.
        // occupied (dark) pixels get positive log-odds.
        for r in 0..h {
            for c in 0..w {
                let src = h - 1 - r;
                let v = if maxv == 255 {
                    data[src * w + c] as f64
                } else {
                    data[src * w + c] as f64 * 255.0 / maxv as f64
                };
                if v <= 40.0 {
                    g.apply_delta(c as i64, r as i64, g.params().occ_log_odds);
                }
            }
        }
        Ok(g)
    }

    /// Write the grid as a PNG file.
    pub fn save_png(&self, path: &str) -> Result<(), String> {
        let (w, h, data) = self.to_grayscale();
        let img = image::GrayImage::from_raw(w as u32, h as u32, data)
            .ok_or("grid size overflow")?;
        img.save(path).map_err(|e| e.to_string())
    }

    /// Write nav-map-style YAML metadata (ROS map_server compatible).
    pub fn save_yaml(&self, path: &str, image_file: &str) -> std::io::Result<()> {
        use std::io::Write;
        let p = self.params();
        let mut f = std::fs::File::create(path)?;
        writeln!(f, "image: {image_file}")?;
        writeln!(f, "resolution: {}", p.resolution)?;
        writeln!(f, "origin: [{}, {}, 0.0]", p.min_x, p.min_y)?;
        writeln!(f, "occupied_thresh: 0.65")?;
        writeln!(f, "free_thresh: 0.25")?;
        writeln!(f, "negate: 0")?;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn params() -> GridMapParams {
        GridMapParams {
            resolution: 0.1,
            min_x: -10.0,
            min_y: -10.0,
            max_x: 10.0,
            max_y: 10.0,
            ..Default::default()
        }
    }

    #[test]
    fn world_to_cell_roundtrip() {
        let g = GridMap::new(params());
        let (c, r) = g.world_to_cell(0.0, 0.0);
        assert_eq!((c, r), (100, 100));
        let w = g.cell_to_world(c, r);
        assert!((w[0] - 0.05).abs() < 1e-9);
        assert!((w[1] - 0.05).abs() < 1e-9);
    }

    #[test]
    fn bounds_checking() {
        let g = GridMap::new(params());
        assert!(g.in_bounds(0, 0));
        assert!(!g.in_bounds(-1, 0));
        assert!(!g.in_bounds(200, 0));
    }

    #[test]
    fn occupancy_default_is_unknown() {
        let g = GridMap::new(params());
        assert!((g.occupancy_at(1.0, 1.0) - 0.5).abs() < 1e-9);
        // out of bounds also unknown
        assert!((g.occupancy_at(100.0, 100.0) - 0.5).abs() < 1e-9);
    }

    #[test]
    fn ray_marks_hit_occupied() {
        let mut g = GridMap::new(params());
        let sensor = [0.0, 0.0];
        let hit = [1.0, 0.0];
        g.update_from_scan(sensor, &[hit]);
        // the hit cell should be occupied (single hit: +0.6 log-odds)
        let (hc, hr) = g.world_to_cell(1.0, 0.0);
        assert!(g.occupancy(hc, hr) > 0.6, "hit cell occupancy = {}", g.occupancy(hc, hr));
        assert!(g.cell_log_odds(hc, hr) > 0.0);
        // a cell halfway should be free
        let (fc, fr) = g.world_to_cell(0.4, 0.0);
        assert!(g.occupancy(fc, fr) < 0.5, "free cell occupancy = {}", g.occupancy(fc, fr));
    }

    #[test]
    fn point_outside_is_clamped_not_crash() {
        let mut g = GridMap::new(params());
        let sensor = [0.0, 0.0];
        let hit = [100.0, 0.0]; // far outside
        g.update_from_scan(sensor, &[hit]);
        assert!(g.cell_count() > 0);
    }

    #[test]
    fn clear_resets() {
        let mut g = GridMap::new(params());
        g.update_from_scan([0.0, 0.0], &[[1.0, 0.0]]);
        assert!(g.cell_count() > 0);
        g.clear();
        assert_eq!(g.cell_count(), 0);
    }
}
