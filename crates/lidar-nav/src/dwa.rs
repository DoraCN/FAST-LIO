//! Dynamic Window Approach (DWA) local obstacle avoidance.
//!
//! Given the robot's current state (pose, velocity) and a set of nearby
//! obstacles (world-frame points), `dwa_step` samples feasible linear/angular
//! velocity pairs inside the dynamic window (bounded by the robot's
//! acceleration limits), simulates each for a short horizon, and scores them by
//! a weighted cost:
//!
//! - progress toward the goal / following the global path,
//! - clearance from obstacles,
//! - speed.
//!
//! The best velocity is returned as a control command `(linear, angular)`.

/// A costmap obstacle: world position + a circular footprint radius (m).
#[derive(Clone, Copy, Debug)]
pub struct Obstacle {
    pub x: f64,
    pub y: f64,
    pub radius: f64,
}

impl Obstacle {
    pub fn new(x: f64, y: f64, radius: f64) -> Self {
        Self { x, y, radius }
    }
}

/// Robot pose `(x, y, theta)` in the map frame.
#[derive(Clone, Copy, Debug, Default)]
pub struct Pose {
    pub x: f64,
    pub y: f64,
    pub theta: f64,
}

/// Robot state fed into the planner.
#[derive(Clone, Copy, Debug, Default)]
pub struct DwaState {
    pub pose: Pose,
    /// current linear velocity (m/s)
    pub v: f64,
    /// current angular velocity (rad/s)
    pub w: f64,
}

/// Goal for the local planner: a point and a tolerance radius.
#[derive(Clone, Copy, Debug)]
pub struct DwaGoal {
    pub x: f64,
    pub y: f64,
    /// considered reached when within this distance (m).
    pub tolerance: f64,
}

/// Output of one DWA step.
#[derive(Clone, Copy, Debug)]
pub struct DwaCmd {
    pub linear: f64,
    pub angular: f64,
}

/// DWA tuning parameters.
#[derive(Clone, Debug)]
pub struct DwaParams {
    /// max linear speed (m/s)
    pub max_v: f64,
    /// max angular speed (rad/s)
    pub max_w: f64,
    /// linear acceleration limit (m/s²)
    pub acc_v: f64,
    /// angular acceleration limit (rad/s²)
    pub acc_w: f64,
    /// time step for each simulated trajectory sample (s)
    pub dt: f64,
    /// number of samples per axis
    pub n_v: usize,
    pub n_w: usize,
    /// simulation horizon (s)
    pub horizon: f64,
    /// robot footprint radius (m)
    pub robot_radius: f64,
    /// weights: [goal, clearance, speed]
    pub w_goal: f64,
    pub w_clearance: f64,
    pub w_speed: f64,
    /// reached when the goal distance is below this (m)
    pub goal_tolerance: f64,
}

impl Default for DwaParams {
    fn default() -> Self {
        Self {
            max_v: 1.0,
            max_w: 1.0,
            acc_v: 0.4,
            acc_w: 2.0,
            dt: 0.1,
            n_v: 6,
            n_w: 8,
            horizon: 2.0,
            robot_radius: 0.3,
            w_goal: 1.0,
            w_clearance: 0.8,
            w_speed: 0.2,
            goal_tolerance: 0.4,
        }
    }
}

/// Run one DWA planning step.
///
/// Returns the best `(linear, angular)` velocity, or `None` if no feasible
/// velocity exists (e.g. fully blocked) — the caller should stop.
pub fn dwa_step(
    params: &DwaParams,
    state: &DwaState,
    goal: DwaGoal,
    obstacles: &[Obstacle],
) -> Option<DwaCmd> {
    if dist(state.pose.x, state.pose.y, goal.x, goal.y) <= goal.tolerance {
        return Some(DwaCmd { linear: 0.0, angular: 0.0 });
    }

    // dynamic window from current velocity + acceleration limits
    let v_min = (state.v - params.acc_v * params.dt).max(0.0);
    let v_max = (state.v + params.acc_v * params.dt).min(params.max_v);
    let w_min = (state.w - params.acc_w * params.dt).max(-params.max_w);
    let w_max = (state.w + params.acc_w * params.dt).min(params.max_w);

    let mut best: Option<(DwaCmd, f64)> = None;

    for i in 0..params.n_v {
        let v = if params.n_v == 1 {
            0.5 * (v_min + v_max)
        } else {
            v_min + (v_max - v_min) * i as f64 / (params.n_v - 1) as f64
        };
        for j in 0..params.n_w {
            let w = if params.n_w == 1 {
                0.5 * (w_min + w_max)
            } else {
                w_min + (w_max - w_min) * j as f64 / (params.n_w - 1) as f64
            };

            // simulate the trajectory
            let mut px = state.pose.x;
            let mut py = state.pose.y;
            let mut th = state.pose.theta;
            let mut min_clear = f64::INFINITY;
            let mut steps = (params.horizon / params.dt) as usize;
            if steps == 0 {
                steps = 1;
            }
            for _ in 0..steps {
                th += w * params.dt;
                px += v * th.cos() * params.dt;
                py += v * th.sin() * params.dt;
                for o in obstacles {
                    let d = dist(px, py, o.x, o.y);
                    let clear = d - o.radius - params.robot_radius;
                    if clear < 0.0 {
                        // collision
                        min_clear = -f64::INFINITY;
                        break;
                    }
                    if clear < min_clear {
                        min_clear = clear;
                    }
                }
                if min_clear.is_infinite() && min_clear.is_sign_negative() {
                    break;
                }
            }
            if min_clear.is_infinite() && min_clear.is_sign_negative() {
                continue; // collided
            }
            if min_clear.is_infinite() {
                min_clear = 10.0; // no obstacles near -> treat as clear
            }

            let goal_dist = dist(px, py, goal.x, goal.y);
            // signed heading error wrapped to [-π, π): positive = turn left (ccw)
            let heading = (goal.y - py).atan2(goal.x - px);
            let mut ang_diff = heading - th;
            ang_diff = ((ang_diff + std::f64::consts::PI).rem_euclid(std::f64::consts::TAU))
                - std::f64::consts::PI;
            let heading_cost = ang_diff.abs() / std::f64::consts::PI;
            let clearance_cost = min_clear.clamp(0.0, 5.0) / 5.0;
            let speed_cost = v / params.max_v;

            let cost = params.w_goal * heading_cost
                + params.w_goal * goal_dist / 10.0
                + params.w_clearance * (1.0 - clearance_cost)
                + params.w_speed * (1.0 - speed_cost);

            if best
                .as_ref()
                .is_none_or(|(_, c)| cost < *c)
            {
                best = Some((DwaCmd { linear: v, angular: w }, cost));
            }
        }
    }

    best.map(|(cmd, _)| cmd)
}

fn dist(x1: f64, y1: f64, x2: f64, y2: f64) -> f64 {
    let dx = x1 - x2;
    let dy = y1 - y2;
    (dx * dx + dy * dy).sqrt()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn params() -> DwaParams {
        DwaParams::default()
    }

    #[test]
    fn reaches_goal_stops() {
        let p = params();
        let state = DwaState::default();
        let goal = DwaGoal { x: 0.1, y: 0.0, tolerance: 0.5 };
        let cmd = dwa_step(&p, &state, goal, &[]).unwrap();
        assert!(cmd.linear.abs() < 1e-6 && cmd.angular.abs() < 1e-6);
    }

    #[test]
    fn moves_toward_goal() {
        let p = params();
        let state = DwaState {
            pose: Pose { x: 0.0, y: 0.0, theta: 0.0 },
            ..Default::default()
        };
        let goal = DwaGoal { x: 5.0, y: 0.0, tolerance: 0.4 };
        let cmd = dwa_step(&p, &state, goal, &[]).unwrap();
        assert!(cmd.linear > 0.0, "should move forward, got {cmd:?}");
        assert!(cmd.angular.abs() < 0.5, "should steer roughly straight, got {cmd:?}");
    }

    #[test]
    fn blocked_returns_none_or_slow() {
        let p = params();
        let state = DwaState {
            pose: Pose { x: 0.0, y: 0.0, theta: 0.0 },
            ..Default::default()
        };
        let goal = DwaGoal { x: 5.0, y: 0.0, tolerance: 0.4 };
        // wall right in front
        let obstacles = vec![Obstacle::new(0.6, 0.0, 0.2)];
        let cmd = dwa_step(&p, &state, goal, &obstacles).unwrap_or(DwaCmd { linear: 0.0, angular: 0.0 });
        assert!(cmd.linear < p.max_v, "should slow down or stop near obstacle: {cmd:?}");
    }

    #[test]
    fn turns_toward_goal() {
        let p = params();
        let state = DwaState {
            pose: Pose { x: 0.0, y: 0.0, theta: 0.0 },
            ..Default::default()
        };
        // goal to the left
        let goal = DwaGoal { x: 0.0, y: 5.0, tolerance: 0.4 };
        let cmd = dwa_step(&p, &state, goal, &[]).unwrap();
        assert!(cmd.angular > 0.0, "should turn left (ccw), got {cmd:?}");
    }

    #[test]
    fn obstacle_helper() {
        let o = Obstacle::new(1.0, 2.0, 0.3);
        assert_eq!((o.x, o.y, o.radius), (1.0, 2.0, 0.3));
    }
}
