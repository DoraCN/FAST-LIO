use nalgebra::{Matrix3, Vector3};

pub type V3D = Vector3<f64>;
pub type M3D = Matrix3<f64>;

/// Skew-symmetric matrix of a 3-vector (so3_math.h `SKEW_SYM_MATRX`).
pub fn skew(v: &V3D) -> M3D {
    let mut m = M3D::zeros();
    m[(0, 1)] = -v[2];
    m[(0, 2)] = v[1];
    m[(1, 0)] = v[2];
    m[(1, 2)] = -v[0];
    m[(2, 0)] = -v[1];
    m[(2, 1)] = v[0];
    m
}

/// Rodrigues exponential map `Exp(ang)` (so3_math.h).
pub fn exp(ang: &V3D) -> M3D {
    let ang_norm = ang.norm();
    let eye = M3D::identity();
    if ang_norm > 0.0000001 {
        let r_axis = ang / ang_norm;
        let k = skew(&r_axis);
        eye + ang_norm.sin() * k + (1.0 - ang_norm.cos()) * (k * k)
    } else {
        eye
    }
}

/// Scaled exponential map `Exp(ang_vel * dt)` (so3_math.h).
pub fn exp_scaled(ang_vel: &V3D, dt: f64) -> M3D {
    let ang_vel_norm = ang_vel.norm();
    let eye = M3D::identity();
    if ang_vel_norm > 0.0000001 {
        let r_axis = ang_vel / ang_vel_norm;
        let k = skew(&r_axis);
        let r_ang = ang_vel_norm * dt;
        eye + r_ang.sin() * k + (1.0 - r_ang.cos()) * (k * k)
    } else {
        eye
    }
}

/// Logarithm map `Log(R)` (so3_math.h).
pub fn log(r: &M3D) -> V3D {
    let theta = if r.trace() > 3.0 - 1e-6 {
        0.0
    } else {
        (0.5 * (r.trace() - 1.0)).acos()
    };
    let k = V3D::new(r[(2, 1)] - r[(1, 2)], r[(0, 2)] - r[(2, 0)], r[(1, 0)] - r[(0, 1)]);
    if theta.abs() < 0.001 {
        0.5 * k
    } else {
        0.5 * theta / theta.sin() * k
    }
}

/// `A_matrix` from mtkmath.hpp (right-J / BCH approximation, also used for
/// correcting the covariance under a right perturbation).
pub fn a_matrix(v: &V3D) -> M3D {
    let squared_norm = v[0] * v[0] + v[1] * v[1] + v[2] * v[2];
    let norm = squared_norm.sqrt();
    if norm < 1e-11 {
        M3D::identity()
    } else {
        M3D::identity()
            + (1.0 - norm.cos()) / squared_norm * skew(v)
            + (1.0 - norm.sin() / norm) / squared_norm * (skew(v) * skew(v))
    }
}

/// Rotation matrix -> Euler angles (roll/pitch/yaw, radians), so3_math.h
/// `RotMtoEuler`.
pub fn rot_m_to_euler(rot: &M3D) -> V3D {
    let sy = (rot[(0, 0)] * rot[(0, 0)] + rot[(1, 0)] * rot[(1, 0)]).sqrt();
    let singular = sy < 1e-6;
    let (x, y, z) = if !singular {
        (
            rot[(2, 1)].atan2(rot[(2, 2)]),
            (-rot[(2, 0)]).atan2(sy),
            rot[(1, 0)].atan2(rot[(0, 0)]),
        )
    } else {
        (
            (-rot[(1, 2)]).atan2(rot[(1, 1)]),
            (-rot[(2, 0)]).atan2(sy),
            0.0,
        )
    };
    V3D::new(x, y, z)
}

#[cfg(test)]
mod tests {
    use super::*;

    const TOL: f64 = 1e-9;

    fn rand_vec() -> V3D {
        // deterministic pseudo-random vectors for reproducible tests
        V3D::new(0.73, -1.21, 2.45)
    }

    #[test]
    fn skew_is_antisymmetric() {
        let v = rand_vec();
        let m = skew(&v);
        assert!(m.magnitude() > 0.0);
        // skew(v) * x == v x x
        let x = V3D::new(0.5, -0.3, 1.1);
        let cross = v.cross(&x);
        let mv = m * x;
        for i in 0..3 {
            assert!((cross[i] - mv[i]).abs() < TOL);
        }
    }

    #[test]
    fn exp_zero_is_identity() {
        assert!((exp(&V3D::zeros()) - M3D::identity()).norm() < TOL);
    }

    #[test]
    fn exp_log_roundtrip() {
        let v = rand_vec();
        let r = exp(&v);
        let l = log(&r);
        assert!((l - v).norm() < TOL);
    }

    #[test]
    fn exp_is_rotation() {
        // exp must be orthogonal with det == 1
        let v = rand_vec();
        let r = exp(&v);
        assert!((r.transpose() * r - M3D::identity()).norm() < TOL);
        assert!((r.determinant() - 1.0).abs() < TOL);
    }

    #[test]
    fn log_near_identity() {
        let small = V3D::new(1e-5, -2e-5, 3e-5);
        let r = exp(&small);
        let l = log(&r);
        assert!((l - small).norm() < 1e-8);
    }

    #[test]
    fn exp_scaled_matches_exp() {
        let v = rand_vec();
        let dt = 0.5;
        assert!((exp_scaled(&v, dt) - exp(&(v * dt))).norm() < TOL);
    }

    #[test]
    fn a_matrix_small_angle() {
        // below the 1e-11 tolerance branch -> identity
        let tiny = V3D::new(1e-13, 1e-13, 1e-13);
        assert!((a_matrix(&tiny) - M3D::identity()).norm() < 1e-6);
        // small but above tolerance: a_matrix deviates from identity by ~|v|/2
        let small = V3D::new(1e-4, 0.0, 0.0);
        let d = a_matrix(&small) - M3D::identity();
        assert!(d.norm() > 1e-6 && d.norm() < 1e-3);
    }

    #[test]
    fn a_matrix_commutes_with_exp_for_small() {
        // A(v) * exp(v) ≈ exp(v) * A(v) for small v (approximately)
        let v = V3D::new(0.05, -0.03, 0.02);
        let a = a_matrix(&v);
        let e = exp(&v);
        assert!((a * e - e * a).norm() < 1e-3);
    }

    #[test]
    fn rot_to_euler_identity() {
        let e = rot_m_to_euler(&M3D::identity());
        assert!(e.norm() < TOL);
    }
}
