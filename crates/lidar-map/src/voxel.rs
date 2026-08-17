//! Sparse 3D occupancy voxel map.
//!
//! World points are discretized into an axis-aligned voxel grid; each voxel
//! stores an occupancy log-odds value. Unlike the 2D [`GridMap`](crate::grid),
//! this representation is sparse and has no fixed bounding box, so it can grow
//! with the explored area.

use std::collections::HashMap;

use crate::Point3;

/// Configuration for building a [`VoxelMap`].
#[derive(Clone, Debug)]
pub struct VoxelMapParams {
    /// Voxel edge length in meters (e.g. `0.1`).
    pub resolution: f64,
    /// Log-odds increment for an occupied (hit) voxel.
    pub occ_log_odds: f64,
    /// Saturation bound for |log-odds|.
    pub saturate: f64,
}

impl Default for VoxelMapParams {
    fn default() -> Self {
        Self {
            resolution: 0.1,
            occ_log_odds: 0.6,
            saturate: 4.0,
        }
    }
}

/// Sparse 3D occupancy map.
#[derive(Clone, Debug)]
pub struct VoxelMap {
    params: VoxelMapParams,
    /// voxel index -> log-odds.
    voxels: HashMap<(i64, i64, i64), f64>,
}

impl VoxelMap {
    pub fn new(params: VoxelMapParams) -> Self {
        Self {
            params,
            voxels: HashMap::new(),
        }
    }

    pub fn params(&self) -> &VoxelMapParams {
        &self.params
    }

    /// World point -> voxel index.
    pub fn world_to_voxel(&self, p: Point3) -> (i64, i64, i64) {
        let r = self.params.resolution;
        (
            (p[0] / r).floor() as i64,
            (p[1] / r).floor() as i64,
            (p[2] / r).floor() as i64,
        )
    }

    /// Voxel index -> world coordinates of the voxel centre.
    pub fn voxel_to_world(&self, i: (i64, i64, i64)) -> Point3 {
        let r = self.params.resolution;
        [
            (i.0 as f64 + 0.5) * r,
            (i.1 as f64 + 0.5) * r,
            (i.2 as f64 + 0.5) * r,
        ]
    }

    pub fn voxel_log_odds(&self, i: (i64, i64, i64)) -> f64 {
        self.voxels.get(&i).copied().unwrap_or(0.0)
    }

    /// Occupancy probability of a voxel in `[0, 1]`; 0.5 = unknown.
    pub fn occupancy(&self, i: (i64, i64, i64)) -> f64 {
        let l = self.voxel_log_odds(i);
        1.0 / (1.0 + (-l).exp())
    }

    pub fn occupancy_at(&self, p: Point3) -> f64 {
        self.occupancy(self.world_to_voxel(p))
    }

    pub fn is_occupied(&self, p: Point3, threshold: f64) -> bool {
        self.occupancy_at(p) > threshold
    }

    /// Incrementally add occupied points (typically a world-frame scan).
    pub fn update(&mut self, points: &[Point3]) {
        for p in points {
            let i = self.world_to_voxel(*p);
            let e = self.voxels.entry(i).or_insert(0.0);
            *e = (*e + self.params.occ_log_odds).min(self.params.saturate);
        }
    }

    /// Iterate all observed voxels as `(index, log_odds)`.
    pub fn iter_voxels(&self) -> impl Iterator<Item = ((i64, i64, i64), f64)> + '_ {
        self.voxels.iter().map(|(&k, &v)| (k, v))
    }

    pub fn voxel_count(&self) -> usize {
        self.voxels.len()
    }

    pub fn clear(&mut self) {
        self.voxels.clear();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn params() -> VoxelMapParams {
        VoxelMapParams {
            resolution: 0.5,
            ..Default::default()
        }
    }

    #[test]
    fn world_to_voxel_roundtrip() {
        let m = VoxelMap::new(params());
        let i = m.world_to_voxel([1.2, -0.3, 2.7]);
        assert_eq!(i, (2, -1, 5));
        let w = m.voxel_to_world(i);
        assert!((w[0] - 1.25).abs() < 1e-9);
    }

    #[test]
    fn occupancy_default_unknown() {
        let m = VoxelMap::new(params());
        assert!((m.occupancy_at([1.0, 1.0, 1.0]) - 0.5).abs() < 1e-9);
    }

    #[test]
    fn update_marks_occupied() {
        let mut m = VoxelMap::new(params());
        m.update(&[[1.0, 1.0, 1.0], [1.1, 1.0, 1.0]]);
        // same voxel (resolution 0.5) -> two increments of 0.6 -> 1.2 log-odds
        assert!(m.occupancy_at([1.0, 1.0, 1.0]) > 0.7);
        assert_eq!(m.voxel_count(), 1);
    }

    #[test]
    fn saturation() {
        let mut m = VoxelMap::new(params());
        let pts: Vec<Point3> = (0..100).map(|_| [1.0, 1.0, 1.0]).collect();
        m.update(&pts);
        let l = m.voxel_log_odds((2, 2, 2));
        assert!(l <= m.params().saturate + 1e-9);
    }
}
