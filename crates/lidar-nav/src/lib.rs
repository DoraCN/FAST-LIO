//! Path planning and navigation on top of the lidar map.
//!
//! This crate turns the occupancy map produced by [`lidar_map`] into executable
//! robot motion:
//!
//! - [`astar`] — grid-based global path planning (A*) that finds a collision-free
//!   route from a start to a goal on a [`GridMap`](lidar_map::GridMap).
//! - [`dwa`] — local obstacle avoidance (Dynamic Window Approach) that follows
//!   the global path while reacting to nearby obstacles in real time.
//!
//! ```text
//! lidar_map::GridMap  +  robot pose
//!     → astar (global plan)
//!     → dwa (local control: linear + angular velocity)
//! ```

pub mod astar;
pub mod dwa;

pub use astar::{astar, AStarOptions, Waypoint};
pub use dwa::{DwaParams, DwaState, dwa_step};
