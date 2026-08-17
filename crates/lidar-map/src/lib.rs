//! Occupancy / voxel map built incrementally from FAST-LIO point clouds.
//!
//! This crate converts the world-frame point clouds produced by
//! [`fast_lio`] into representations that planning & navigation can query:
//!
//! - [`GridMap`] — a 2D occupancy grid (probability of a cell being occupied).
//!   Supports incremental ray-casting updates from a sensor pose + a scan,
//!   occupancy queries, and PGM / PNG export.
//! - [`VoxelMap`] — a sparse 3D occupancy voxel map with incremental updates
//!   and occupancy queries.
//!
//! The typical data flow is:
//!
//! ```text
//! fast_lio (pose + world point cloud)
//!     → lidar_map::GridMap::update_from_scan(pose, points)   // 2D
//!     → lidar_map::VoxelMap::update(points)                  // 3D
//!     → lidar-nav path planning
//! ```

pub mod grid;
pub mod voxel;

pub use grid::{GridMap, GridMapParams};
pub use voxel::VoxelMap;

/// Simple 2D point used by the map update / query APIs.
pub type Point2 = [f64; 2];
/// Simple 3D point (world frame, meters).
pub type Point3 = [f64; 3];
