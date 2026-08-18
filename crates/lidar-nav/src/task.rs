//! Multi-waypoint navigation task: A* plan → Pure Pursuit follow → face the
//! goal heading → dwell → next waypoint.
//!
//! A task is a text file with one waypoint per line:
//!
//! ```text
//! x y [yaw_deg] [dwell_sec]
//! ```
//!
//! - `x y` — target position in map meters (required).
//! - `yaw_deg` — heading to face after arriving (degrees, optional).
//! - `dwell_sec` — seconds to hold still before moving on (optional).
//!
//! Lines starting with `#` and blank lines are ignored. Example:
//!
//! ```text
//! # drive to the door, face it, wait 5 s, then go to the loading bay
//! 2.0 1.5 90 5
//! 8.0 -2.0 0
//! ```

use std::fs;
use std::time::{Duration, Instant};

use lidar_map::GridMap;

use crate::astar::{AStarOptions, Waypoint, astar};

/// One task waypoint.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TaskWaypoint {
    pub x: f64,
    pub y: f64,
    /// Heading to face after arriving, in degrees. `None` = skip alignment.
    pub yaw_deg: Option<f64>,
    /// Seconds to hold still before moving to the next waypoint.
    pub dwell_s: f64,
}

/// Parse a task file. One waypoint per line: `x y [yaw_deg] [dwell_sec]`.
pub fn load_task(path: &str) -> Result<Vec<TaskWaypoint>, String> {
    let text = fs::read_to_string(path).map_err(|e| format!("read {path}: {e}"))?;
    let mut wps = Vec::new();
    for (lineno, line) in text.lines().enumerate() {
        let line = line.trim();
        if line.is_empty() || line.starts_with('#') {
            continue;
        }
        let toks: Vec<&str> = line.split_whitespace().collect();
        let mut it = toks.iter();
        let x: f64 = it
            .next()
            .and_then(|s| s.parse().ok())
            .ok_or_else(|| format!("{path}:{}: bad x", lineno + 1))?;
        let y: f64 = it
            .next()
            .and_then(|s| s.parse().ok())
            .ok_or_else(|| format!("{path}:{}: bad y", lineno + 1))?;
        let yaw = it.next().and_then(|s| s.parse::<f64>().ok());
        let dwell = it
            .next()
            .and_then(|s| s.parse::<f64>().ok())
            .unwrap_or(0.0);
        wps.push(TaskWaypoint {
            x,
            y,
            yaw_deg: yaw.filter(|v| v.is_finite()),
            dwell_s: dwell.max(0.0),
        });
    }
    if wps.is_empty() {
        return Err(format!("{path}: no waypoints"));
    }
    Ok(wps)
}

/// Wrap an angle into (-PI, PI].
pub fn norm_angle(a: f64) -> f64 {
    let mut a = a;
    while a > std::f64::consts::PI {
        a -= 2.0 * std::f64::consts::PI;
    }
    while a < -std::f64::consts::PI {
        a += 2.0 * std::f64::consts::PI;
    }
    a
}

/// Phase of the current waypoint.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Phase {
    /// Driving along the planned path toward the waypoint position.
    Navigating,
    /// Reached the position; rotating in place to face `yaw_deg`.
    Aligning,
    /// Holding still (dwell) before moving on.
    Dwelling,
    /// All waypoints done; robot should stop.
    Done,
}

/// One control tick output.
#[derive(Clone, Copy, Debug)]
pub struct Control {
    pub vx: f64,
    pub wz: f64,
    pub phase: Phase,
    /// Index of the current waypoint (0-based); `None` when done.
    pub waypoint: Option<usize>,
}

/// Parameters of the task executor.
#[derive(Clone, Debug)]
pub struct TaskParams {
    /// Distance ahead on the path to aim at (m).
    pub lookahead: f64,
    /// Max forward speed (m/s).
    pub max_vx: f64,
    /// Max angular speed (rad/s).
    pub max_wz: f64,
    /// Within this distance of the waypoint the robot is considered arrived (m).
    pub arrive: f64,
    /// Angular tolerance for the "facing" step (radians).
    pub yaw_tol: f64,
    /// A* obstacle inflation radius (m).
    pub inflation: f64,
}

impl Default for TaskParams {
    fn default() -> Self {
        Self {
            lookahead: 0.6,
            max_vx: 0.4,
            max_wz: 1.0,
            arrive: 0.15,
            yaw_tol: 5f64.to_radians(),
            inflation: 0.3,
        }
    }
}

/// Executes a multi-waypoint task on a [`GridMap`].
///
/// Feed it the robot pose on every control tick via [`TaskExecutor::step`];
/// it returns the velocity command for the chassis and the current phase.
pub struct TaskExecutor<'a> {
    map: &'a GridMap,
    task: Vec<TaskWaypoint>,
    params: TaskParams,
    idx: usize,
    path: Vec<Waypoint>,
    planned: bool,
    phase: Phase,
    dwell_until: Option<Instant>,
}

impl<'a> TaskExecutor<'a> {
    pub fn new(map: &'a GridMap, task: Vec<TaskWaypoint>, params: TaskParams) -> Self {
        Self {
            map,
            task,
            params,
            idx: 0,
            path: Vec::new(),
            planned: false,
            phase: Phase::Navigating,
            dwell_until: None,
        }
    }

    /// One control tick at `pose` (`[x, y, yaw]` in the map frame).
    ///
    /// Returns the velocity command. The caller applies `vx`/`wz` to the
    /// chassis (e.g. via `dwa_step`-style local obstacle avoidance on top).
    pub fn step(&mut self, pose: [f64; 3], now: Instant) -> Control {
        let (x, y, yaw) = (pose[0], pose[1], pose[2]);

        if self.phase == Phase::Done {
            return Control {
                vx: 0.0,
                wz: 0.0,
                phase: Phase::Done,
                waypoint: None,
            };
        }
        let wp = self.task[self.idx];

        // Dwell expiry: move on to the next waypoint.
        if self.phase == Phase::Dwelling {
            if self.dwell_until.is_some_and(|until| now >= until) {
                self.phase = Phase::Done;
                self.advance();
            }
            if self.phase != Phase::Dwelling {
                return self.step(pose, now);
            }
            return Control {
                vx: 0.0,
                wz: 0.0,
                phase: Phase::Dwelling,
                waypoint: Some(self.idx),
            };
        }

        // Plan the path to the current waypoint once.
        if !self.planned {
            self.path = match astar(
                self.map,
                Waypoint { x, y },
                Waypoint { x: wp.x, y: wp.y },
                &AStarOptions {
                    inflation: self.params.inflation,
                    ..Default::default()
                },
            ) {
                Some(p) => resample(&p, (self.params.lookahead * 0.5).max(0.1)),
                None => {
                    // unreachable: give up this waypoint
                    eprintln!(
                        "nav: NO PATH to waypoint {}/{} ({:.2},{:.2}) from ({:.2},{:.2}) — skipping",
                        self.idx + 1,
                        self.task.len(),
                        wp.x,
                        wp.y,
                        x,
                        y
                    );
                    self.phase = Phase::Done;
                    self.advance();
                    return self.step(pose, now);
                }
            };
            self.planned = true;
            println!(
                "nav: waypoint {}/{} ({:.2},{:.2}) — plan {} points",
                self.idx + 1,
                self.task.len(),
                wp.x,
                wp.y,
                self.path.len()
            );
        }

        // Pure pursuit along the path.
        let (mut vx, mut wz, path_done) = self.follow(x, y, yaw);

        // Arrived at the waypoint (path end == nearest reachable cell).
        if path_done {
            match wp.yaw_deg {
                // No heading requirement: dwell (if any) then move on.
                None => {
                    self.enter_dwell(now);
                }
                Some(gy) => {
                    // Rotate in place to face the goal heading.
                    let diff = norm_angle(gy.to_radians() - yaw);
                    if diff.abs() <= self.params.yaw_tol {
                        println!("nav: waypoint {}/{} aligned to {gy:.1}°", self.idx + 1, self.task.len());
                        self.enter_dwell(now);
                    } else {
                        vx = 0.0;
                        wz = (self.params.max_wz * diff / std::f64::consts::PI * 2.0)
                            .clamp(-self.params.max_wz, self.params.max_wz);
                        self.phase = Phase::Aligning;
                    }
                }
            }
        }

        let phase = self.phase;
        Control {
            vx,
            wz,
            phase,
            waypoint: Some(self.idx),
        }
    }

    fn enter_dwell(&mut self, now: Instant) {
        if self.task[self.idx].dwell_s > 0.0 {
            self.phase = Phase::Dwelling;
            self.dwell_until = Some(now + Duration::from_secs_f64(self.task[self.idx].dwell_s));
            println!(
                "nav: arrived at waypoint {}/{} — dwell {:.1}s",
                self.idx + 1,
                self.task.len(),
                self.task[self.idx].dwell_s
            );
        } else {
            self.phase = Phase::Done;
            self.advance();
        }
    }

    fn advance(&mut self) {
        self.path.clear();
        self.planned = false;
        self.dwell_until = None;
        self.phase = Phase::Navigating;
        self.idx += 1;
        if self.idx >= self.task.len() {
            self.phase = Phase::Done;
            println!("nav: all {} waypoints done", self.task.len());
        }
    }

    /// Pure pursuit: aim at the path point `lookahead` ahead of the robot.
    /// Returns `(vx, wz, arrived)` — `arrived` true when within `arrive` of the
    /// path end (the nearest reachable cell to the waypoint).
    fn follow(&self, x: f64, y: f64, yaw: f64) -> (f64, f64, bool) {
        let last = self.path.last().expect("empty path");
        if dist(x, y, last.x, last.y) <= self.params.arrive {
            return (0.0, 0.0, true);
        }
        let (tx, ty) = self.lookahead_point(x, y);
        let angle_to = (ty - y).atan2(tx - x);
        let diff = norm_angle(angle_to - yaw);
        let wz = (self.params.max_wz * diff / std::f64::consts::PI * 2.0)
            .clamp(-self.params.max_wz, self.params.max_wz);
        let turn = (diff.abs() / std::f64::consts::PI).min(1.0);
        let vx = (self.params.max_vx * (1.0 - 1.6 * turn)).clamp(0.0, self.params.max_vx);
        (vx, wz, false)
    }

    fn lookahead_point(&self, x: f64, y: f64) -> (f64, f64) {
        let mut ni = 0;
        let mut nd = f64::MAX;
        for (i, p) in self.path.iter().enumerate() {
            let d = dist(x, y, p.x, p.y);
            if d < nd {
                nd = d;
                ni = i;
            }
        }
        let mut t = ni;
        while t + 1 < self.path.len() && dist(x, y, self.path[t + 1].x, self.path[t + 1].y) < self.params.lookahead {
            t += 1;
        }
        (self.path[t].x, self.path[t].y)
    }
    pub fn phase(&self) -> Phase {
        self.phase
    }

    pub fn current_waypoint(&self) -> Option<&TaskWaypoint> {
        if self.phase == Phase::Done {
            None
        } else {
            Some(&self.task[self.idx])
        }
    }
}

fn dist(x1: f64, y1: f64, x2: f64, y2: f64) -> f64 {
    ((x1 - x2).powi(2) + (y1 - y2).powi(2)).sqrt()
}

/// Re-sample a path so consecutive waypoints are at most `max_step` apart.
/// Pure pursuit needs a dense enough path to pick a lookahead point ahead of
/// the robot; A* on a coarse grid can otherwise produce a degenerate path.
fn resample(path: &[Waypoint], max_step: f64) -> Vec<Waypoint> {
    if max_step <= 0.0 || path.len() < 2 {
        return path.to_vec();
    }
    let mut out = Vec::with_capacity(path.len() * 2);
    for w in path.windows(2) {
        let (x0, y0) = (w[0].x, w[0].y);
        let (x1, y1) = (w[1].x, w[1].y);
        let d = ((x1 - x0).powi(2) + (y1 - y0).powi(2)).sqrt();
        let n = (d / max_step).ceil() as usize;
        for i in 0..n {
            let t = i as f64 / n as f64;
            out.push(Waypoint { x: x0 + t * (x1 - x0), y: y0 + t * (y1 - y0) });
        }
    }
    out.push(*path.last().unwrap());
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    fn norm(v: f64) -> f64 {
        let mut v = v;
        while v > std::f64::consts::PI {
            v -= 2.0 * std::f64::consts::PI;
        }
        while v < -std::f64::consts::PI {
            v += 2.0 * std::f64::consts::PI;
        }
        v
    }

    #[test]
    fn norm_angle_contract() {
        assert!((norm_angle(std::f64::consts::PI * 3.0) - norm(std::f64::consts::PI * 3.0)).abs() < 1e-9);
        assert!((norm_angle(-std::f64::consts::PI * 3.0) - norm(-std::f64::consts::PI * 3.0)).abs() < 1e-9);
        assert!((norm_angle(0.0)).abs() < 1e-9);
    }

    #[test]
    fn load_task_parses_yaw_and_dwell() {
        let dir = std::env::temp_dir();
        let path = dir.join("fast_lio_test_task.txt");
        std::fs::write(
            &path,
            "# comment\n1.0 2.0 90 5\n3.0 4.0\n\n5.0 6.0 180\n",
        )
        .unwrap();
        let wps = load_task(path.to_str().unwrap()).unwrap();
        assert_eq!(wps.len(), 3);
        assert_eq!(wps[0], TaskWaypoint { x: 1.0, y: 2.0, yaw_deg: Some(90.0), dwell_s: 5.0 });
        assert_eq!(wps[1], TaskWaypoint { x: 3.0, y: 4.0, yaw_deg: None, dwell_s: 0.0 });
        assert_eq!(wps[2], TaskWaypoint { x: 5.0, y: 6.0, yaw_deg: Some(180.0), dwell_s: 0.0 });
        std::fs::remove_file(&path).ok();
    }

    #[test]
    fn load_task_rejects_empty() {
        let dir = std::env::temp_dir();
        let path = dir.join("fast_lio_test_task_empty.txt");
        std::fs::write(&path, "# only a comment\n").unwrap();
        assert!(load_task(path.to_str().unwrap()).is_err());
        std::fs::remove_file(&path).ok();
    }

    fn task_map() -> GridMap {
        GridMap::new(lidar_map::GridMapParams {
            resolution: 1.0,
            min_x: -20.0,
            min_y: -20.0,
            max_x: 20.0,
            max_y: 20.0,
            ..Default::default()
        })
    }

    fn params() -> TaskParams {
        TaskParams {
            lookahead: 0.6,
            max_vx: 0.4,
            max_wz: 1.0,
            arrive: 0.15,
            yaw_tol: 5f64.to_radians(),
            inflation: 0.3,
        }
    }

    /// Drive the executor until done with a simple kinematic sim.
    fn drive_to_done(
        exec: &mut TaskExecutor<'_>,
        start: [f64; 3],
        dt: f64,
        max_steps: usize,
    ) -> Vec<Control> {
        let mut pose = start;
        let t0 = Instant::now();
        let mut out = Vec::new();
        for i in 0..max_steps {
            let now = t0 + Duration::from_secs_f64(i as f64 * dt);
            let c = exec.step(pose, now);
            out.push(c);
            if c.phase == Phase::Done {
                break;
            }
            pose[2] = norm_angle(pose[2] + c.wz * dt);
            pose[0] += c.vx * pose[2].cos() * dt;
            pose[1] += c.vx * pose[2].sin() * dt;
        }
        out
    }

    #[test]
    fn task_reaches_waypoint_and_dwells() {
        let map = task_map();
        let task = vec![
            TaskWaypoint { x: 3.0, y: 0.0, yaw_deg: None, dwell_s: 2.0 },
            TaskWaypoint { x: 3.0, y: 3.0, yaw_deg: None, dwell_s: 0.0 },
        ];
        let mut exec = TaskExecutor::new(&map, task, params());
        let controls = drive_to_done(&mut exec, [0.0, 0.0, 0.0], 0.1, 5000);
        let done = controls.last().unwrap();
        assert_eq!(done.phase, Phase::Done);
        // must have dwelled: some step reports Dwelling
        assert!(controls.iter().any(|c| c.phase == Phase::Dwelling));
        // final pose should be near the last waypoint
        let mut pose = [0.0f64, 0.0, 0.0];
        for c in &controls {
            pose[2] = norm_angle(pose[2] + c.wz * 0.1);
            pose[0] += c.vx * pose[2].cos() * 0.1;
            pose[1] += c.vx * pose[2].sin() * 0.1;
        }
        assert!((pose[0] - 3.0).abs() < 0.5, "final x = {}", pose[0]);
        assert!((pose[1] - 3.0).abs() < 0.5, "final y = {}", pose[1]);
    }

    #[test]
    fn task_faces_heading_before_done() {
        let map = task_map();
        let task = vec![TaskWaypoint { x: 3.0, y: 0.0, yaw_deg: Some(90.0), dwell_s: 0.0 }];
        let mut exec = TaskExecutor::new(&map, task, params());
        let controls = drive_to_done(&mut exec, [0.0, 0.0, 0.0], 0.1, 5000);
        let done = controls.last().unwrap();
        assert_eq!(done.phase, Phase::Done);
        // final heading should be ~90°
        let mut yaw = 0.0f64;
        for c in &controls {
            yaw = norm_angle(yaw + c.wz * 0.1);
        }
        assert!((yaw - std::f64::consts::FRAC_PI_2).abs() < 0.2, "final yaw = {yaw}");
    }
}
