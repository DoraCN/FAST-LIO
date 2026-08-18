//! A* grid path planning.
//!
//! Searches the 8-connected grid of a [`GridMap`](lidar_map::GridMap) for the
//! lowest-cost path from a start world point to a goal world point, avoiding
//! occupied cells (with optional inflation so the robot keeps a safety margin
//! from obstacles).

use std::cmp::Ordering;
use std::collections::{BinaryHeap, HashMap};

use lidar_map::GridMap;

/// A waypoint on the planned path (world coordinates, meters).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Waypoint {
    pub x: f64,
    pub y: f64,
}

/// Options controlling the A* search.
#[derive(Clone, Debug)]
pub struct AStarOptions {
    /// Cells within this distance (in meters) of an occupied cell are treated
    /// as blocked, keeping the robot away from obstacles. Default `0.3`.
    pub inflation: f64,
    /// Maximum number of cells expanded before giving up. Default 200_000.
    pub max_expansions: usize,
}

impl Default for AStarOptions {
    fn default() -> Self {
        Self {
            inflation: 0.3,
            max_expansions: 200_000,
        }
    }
}

/// A node in the open set, ordered by f-score.
#[derive(Clone, Copy, PartialEq)]
struct Node {
    /// (col, row)
    cell: (i64, i64),
    f: f64,
}

impl Eq for Node {}

impl PartialOrd for Node {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for Node {
    fn cmp(&self, other: &Self) -> Ordering {
        // reversed: BinaryHeap is a max-heap, we want the smallest f first
        other
            .f
            .partial_cmp(&self.f)
            .unwrap_or(Ordering::Equal)
            .then_with(|| self.cell.cmp(&other.cell))
    }
}

const NEIGHBORS: [(i64, i64); 8] = [
    (1, 0),
    (-1, 0),
    (0, 1),
    (0, -1),
    (1, 1),
    (1, -1),
    (-1, 1),
    (-1, -1),
];

/// A* search on the given grid.
///
/// Returns an ordered list of waypoints from `start` to `goal` (both inclusive),
/// or `None` if no path exists.
pub fn astar(
    grid: &GridMap,
    start: Waypoint,
    goal: Waypoint,
    opts: &AStarOptions,
) -> Option<Vec<Waypoint>> {
    let res = grid.params().resolution;
    let infl = (opts.inflation / res).ceil() as i64;

    let start_cell = grid.world_to_cell(start.x, start.y);
    let goal_cell = grid.world_to_cell(goal.x, goal.y);
    if !grid.in_bounds(start_cell.0, start_cell.1) || !grid.in_bounds(goal_cell.0, goal_cell.1) {
        let (w, h) = grid.dims();
        eprintln!(
            "astar: OUT OF MAP start({:.2},{:.2})->cell({},{}) in={} goal({:.2},{:.2})->cell({},{}) in={} (dims {}x{}, origin=({:.2},{:.2}), res={res})",
            start.x, start.y, start_cell.0, start_cell.1,
            grid.in_bounds(start_cell.0, start_cell.1),
            goal.x, goal.y, goal_cell.0, goal_cell.1,
            grid.in_bounds(goal_cell.0, goal_cell.1),
            w, h,
            grid.params().min_x,
            grid.params().min_y,
        );
        return None;
    }
    // Start AND goal may sit on stray map noise / an inflated cell: snap to
    // the nearest free cell (BFS, ~1 m) so planning still works. This mirrors
    // the reference nav behaviour.
    let snap_max = ((1.0 / res).ceil() as usize).max(4);
    let start_snapped = snap_to_free(grid, start_cell, infl, snap_max).unwrap_or(start_cell);
    let goal_snapped = snap_to_free(grid, goal_cell, infl, snap_max).unwrap_or(goal_cell);
    if !traversable(grid, start_snapped, infl) || !traversable(grid, goal_snapped, infl) {
        eprintln!(
            "astar: start({:.2},{:.2})->({},{}) trav={} goal({:.2},{:.2})->({},{}) trav={} (res={res} snap_max={snap_max})",
            start.x, start.y, start_cell.0, start_cell.1,
            traversable(grid, start_snapped, infl),
            goal.x, goal.y, goal_cell.0, goal_cell.1,
            traversable(grid, goal_snapped, infl),
        );
        return None;
    }
    let start_cell = start_snapped;
    let goal_cell = goal_snapped;

    let h = |c: (i64, i64)| -> f64 {
        let dx = (c.0 - goal_cell.0) as f64;
        let dy = (c.1 - goal_cell.1) as f64;
        // octile heuristic (admissible on an 8-connected grid)
        let d = dx.abs().max(dy.abs());
        (dx.abs() - d) + (dy.abs() - d) + 2.0f64.sqrt() * d
    };

    let mut open = BinaryHeap::new();
    let mut g_score: HashMap<(i64, i64), f64> = HashMap::new();
    let mut came_from: HashMap<(i64, i64), (i64, i64)> = HashMap::new();

    g_score.insert(start_cell, 0.0);
    open.push(Node {
        cell: start_cell,
        f: h(start_cell),
    });

    let mut expansions = 0usize;
    while let Some(node) = open.pop() {
        if node.cell == goal_cell {
            return Some(reconstruct(came_from, start_cell, goal_cell, grid));
        }
        expansions += 1;
        if expansions > opts.max_expansions {
            eprintln!(
                "astar: exceeded {} expansions from ({},{}) to ({},{})",
                opts.max_expansions, start_cell.0, start_cell.1, goal_cell.0, goal_cell.1
            );
            return None;
        }
        let cur_g = g_score[&node.cell];
        for (dc, dr) in NEIGHBORS {
            let nc = (node.cell.0 + dc, node.cell.1 + dr);
            if !grid.in_bounds(nc.0, nc.1) || !traversable(grid, nc, infl) {
                continue;
            }
            let step_cost = if dc != 0 && dr != 0 {
                2.0f64.sqrt()
            } else {
                1.0
            };
            let tent = cur_g + step_cost;
            if tent < g_score.get(&nc).copied().unwrap_or(f64::INFINITY) {
                g_score.insert(nc, tent);
                came_from.insert(nc, node.cell);
                open.push(Node {
                    cell: nc,
                    f: tent + h(nc),
                });
            }
        }
    }
    None
}

/// Is a cell free to traverse (not occupied, not within the inflation radius)?
fn traversable(grid: &GridMap, cell: (i64, i64), infl: i64) -> bool {
    for dr in -infl..=infl {
        for dc in -infl..=infl {
            let c = (cell.0 + dc, cell.1 + dr);
            if !grid.in_bounds(c.0, c.1) {
                continue;
            }
            if grid.occupancy(c.0, c.1) > 0.6 {
                return false;
            }
        }
    }
    true
}

/// Nearest traversable cell via BFS, within `max_cells` steps. Used so a start
/// sitting on stray map noise / an inflated cell is snapped to drivable space
/// (mirrors the reference nav).
fn snap_to_free(grid: &GridMap, start: (i64, i64), infl: i64, max_cells: usize) -> Option<(i64, i64)> {
    use std::collections::VecDeque;
    if traversable(grid, start, infl) {
        return Some(start);
    }
    let mut visited = std::collections::HashSet::new();
    let mut q = VecDeque::new();
    q.push_back((start, 0usize));
    visited.insert(start);
    while let Some((cell, depth)) = q.pop_front() {
        if depth >= max_cells {
            continue;
        }
        for (dc, dr) in NEIGHBORS {
            let nc = (cell.0 + dc, cell.1 + dr);
            if !grid.in_bounds(nc.0, nc.1) || !visited.insert(nc) {
                continue;
            }
            if traversable(grid, nc, infl) {
                return Some(nc);
            }
            q.push_back((nc, depth + 1));
        }
    }
    None
}

/// Walk `came_from` backwards to build the waypoint list.
fn reconstruct(
    came_from: HashMap<(i64, i64), (i64, i64)>,
    start: (i64, i64),
    goal: (i64, i64),
    grid: &GridMap,
) -> Vec<Waypoint> {
    let mut cells = vec![goal];
    let mut cur = goal;
    while let Some(&prev) = came_from.get(&cur) {
        cells.push(prev);
        if prev == start {
            break;
        }
        cur = prev;
    }
    cells.reverse();
    cells
        .into_iter()
        .map(|(c, r)| {
            let w = grid.cell_to_world(c, r);
            Waypoint { x: w[0], y: w[1] }
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use lidar_map::{GridMapParams, Point3};

    fn empty_grid() -> GridMap {
        GridMap::new(GridMapParams {
            resolution: 1.0,
            min_x: -10.0,
            min_y: -10.0,
            max_x: 10.0,
            max_y: 10.0,
            ..Default::default()
        })
    }

    #[test]
    fn straight_path_in_empty_map() {
        let grid = empty_grid();
        let path = astar(
            &grid,
            Waypoint { x: -5.0, y: 0.0 },
            Waypoint { x: 5.0, y: 0.0 },
            &AStarOptions::default(),
        )
        .unwrap();
        assert!(path.len() >= 2);
        assert!((path[0].x + 5.0).abs() < 1.0);
        assert!((path[path.len() - 1].x - 5.0).abs() < 1.0);
        // all waypoints stay on the y=0 row (within one cell)
        for w in &path {
            assert!(w.y.abs() < 1.0, "waypoint off line: {w:?}");
        }
    }

    #[test]
    fn wall_diverts_path() {
        let mut grid = empty_grid();
        // vertical wall at x=0 spanning y=-3..3
        let wall: Vec<Point3> = (0..7).map(|i| [0.0, -3.0 + i as f64, 0.0]).collect();
        grid.mark_occupied(&wall);
        let path = astar(
            &grid,
            Waypoint { x: -5.0, y: 0.0 },
            Waypoint { x: 5.0, y: 0.0 },
            &AStarOptions { inflation: 0.0, ..Default::default() },
        )
        .unwrap();
        // path must go around: some waypoint with |y| > 1
        assert!(
            path.iter().any(|w| w.y.abs() > 1.0),
            "path did not avoid wall: {path:?}"
        );
    }

    #[test]
    fn unreachable_goal_returns_none() {
        let mut grid = empty_grid();
        // fill the entire grid with obstacles: no free cell to snap to
        let blob: Vec<Point3> = (-9..9)
            .flat_map(|i| (-9..9).map(move |j| [i as f64, j as f64, 0.0]))
            .collect();
        grid.mark_occupied(&blob);
        let res = astar(
            &grid,
            Waypoint { x: -5.0, y: -5.0 },
            Waypoint { x: 5.0, y: 5.0 },
            &AStarOptions { inflation: 0.0, ..Default::default() },
        );
        assert!(res.is_none());
    }

    #[test]
    fn goal_on_obstacle_snaps_to_free() {
        let mut grid = empty_grid();
        // single-point obstacle at the goal: it snaps to a neighbouring free
        // cell and plans around it
        grid.mark_occupied(&[[3.0, 3.0, 0.0]]);
        let res = astar(
            &grid,
            Waypoint { x: 0.0, y: 0.0 },
            Waypoint { x: 3.0, y: 3.0 },
            &AStarOptions::default(),
        );
        assert!(res.is_some(), "goal on a single obstacle cell should snap to a free neighbour");
    }

    #[test]
    fn start_on_noise_snaps_to_free() {
        let mut grid = empty_grid();
        // stray noise at the start position
        grid.mark_occupied(&[[0.0, 0.0, 0.0]]);
        let path = astar(
            &grid,
            Waypoint { x: 0.0, y: 0.0 },
            Waypoint { x: 5.0, y: 5.0 },
            &AStarOptions { inflation: 0.0, ..Default::default() },
        )
        .expect("snapped start should still plan");
        assert!(path.len() >= 2);
    }

    #[test]
    fn start_fully_blocked_returns_none() {
        let mut grid = empty_grid();
        // block well beyond the snap radius (4 cells @ res 1.0) so the start
        // has no free cell to snap to
        let blob: Vec<Point3> = (-5..=5)
            .flat_map(|i| (-5..=5).map(move |j| [i as f64, j as f64, 0.0]))
            .collect();
        grid.mark_occupied(&blob);
        let res = astar(
            &grid,
            Waypoint { x: 0.0, y: 0.0 },
            Waypoint { x: 5.0, y: 5.0 },
            &AStarOptions { inflation: 0.0, ..Default::default() },
        );
        assert!(res.is_none());
    }
}
