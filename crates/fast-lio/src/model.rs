use nalgebra::{SMatrix, UnitQuaternion};

use crate::math::manifold::{StateIkfom, STATE_DIM, STATE_DOF};
use crate::math::so3::{skew, M3D, V3D};

/// `input_ikfom` manifold: acc + gyro.
#[derive(Clone, Debug, Default)]
pub struct InputIkfom {
    pub acc: V3D,
    pub gyro: V3D,
}

/// Process noise manifold (`process_noise_ikfom`): ng/na/nbg/nba, 12 DOF.
pub const PROCESS_NOISE_DOF: usize = 12;

/// Process-noise covariance diagonal values (`process_noise_cov()` in
/// `use-ikfom.hpp`).
pub const NOISE_NG: f64 = 0.0001;
pub const NOISE_NA: f64 = 0.0001;
pub const NOISE_NBG: f64 = 0.00001;
pub const NOISE_NBA: f64 = 0.00001;

/// Build the 12x12 process noise covariance Q.
pub fn process_noise_cov() -> SMatrix<f64, PROCESS_NOISE_DOF, PROCESS_NOISE_DOF> {
    let mut q = SMatrix::<f64, PROCESS_NOISE_DOF, PROCESS_NOISE_DOF>::zeros();
    for i in 0..3 {
        q[(i, i)] = NOISE_NG;
        q[(3 + i, 3 + i)] = NOISE_NA;
        q[(6 + i, 6 + i)] = NOISE_NBG;
        q[(9 + i, 9 + i)] = NOISE_NBA;
    }
    q
}

/// `get_f`: continuous-time dynamics, returns the 24-DIM embedded vector.
/// Layout: [vel; omega; 0; 0; a_inertial+grav; 0; 0].
pub fn get_f(s: &StateIkfom, input: &InputIkfom) -> [f64; STATE_DIM] {
    let mut res = [0.0; STATE_DIM];
    let omega = input.gyro - s.bg;
    let a_inertial = s.rot * (input.acc - s.ba);
    // vel
    res[0..3].copy_from_slice(s.vel.as_slice());
    // omega (rot rate)
    res[3..6].copy_from_slice(omega.as_slice());
    // a_inertial + grav
    let a = a_inertial + s.grav.vec;
    res[12..15].copy_from_slice(a.as_slice());
    res
}

/// `df_dx`: Jacobian of f w.r.t. the 23-DOF state, 24x23.
pub fn df_dx(s: &StateIkfom, input: &InputIkfom) -> SMatrix<f64, STATE_DIM, STATE_DOF> {
    let mut cov = SMatrix::<f64, STATE_DIM, STATE_DOF>::zeros();
    // d(vel)/d(vel): block<3,3>(0,12) = I
    cov.fixed_view_mut::<3, 3>(0, 12).copy_from(&M3D::identity());

    let acc_ = input.acc - s.ba;
    let rot: M3D = s.rot.to_rotation_matrix().into();
    // d(a_inertial)/d(rot): block<3,3>(12,3) = -R·hat(acc_)
    cov.fixed_view_mut::<3, 3>(12, 3).copy_from(&(-rot * skew(&acc_)));
    // d(a_inertial)/d(ba): block<3,3>(12,18) = -R
    cov.fixed_view_mut::<3, 3>(12, 18).copy_from(&-rot);
    // d(a_inertial)/d(grav): block<3,2>(12,21) = S2_Mx(0) = -hat(grav)·Bx
    let gm = s.grav.s2_mx(&nalgebra::Vector2::zeros());
    cov.fixed_view_mut::<3, 2>(12, 21).copy_from(&gm);
    // d(omega)/d(bg): block<3,3>(3,15) = -I
    cov.fixed_view_mut::<3, 3>(3, 15).copy_from(&-M3D::identity());
    cov
}

/// `df_dw`: Jacobian of f w.r.t. the 12-DOF process noise, 24x12.
pub fn df_dw(s: &StateIkfom, _input: &InputIkfom) -> SMatrix<f64, STATE_DIM, PROCESS_NOISE_DOF> {
    let mut cov = SMatrix::<f64, STATE_DIM, PROCESS_NOISE_DOF>::zeros();
    let rot: M3D = s.rot.to_rotation_matrix().into();
    // d(omega)/d(ng): block<3,3>(3,0) = -I
    cov.fixed_view_mut::<3, 3>(3, 0).copy_from(&-M3D::identity());
    // d(a_inertial)/d(na): block<3,3>(12,3) = -R
    cov.fixed_view_mut::<3, 3>(12, 3).copy_from(&-rot);
    // d(bg)/d(nbg): block<3,3>(15,6) = I
    cov.fixed_view_mut::<3, 3>(15, 6).copy_from(&M3D::identity());
    // d(ba)/d(nba): block<3,3>(18,9) = I
    cov.fixed_view_mut::<3, 3>(18, 9).copy_from(&M3D::identity());
    cov
}

/// Quaternion -> Euler angles in degrees, ported from `SO3ToEuler` in
/// `use-ikfom.hpp` (roll/pitch/yaw order).
pub fn so3_to_euler(q: &UnitQuaternion<f64>) -> V3D {
    // nalgebra Quaternion fields: w (scalar) + i/j/k (vector part)
    let qr = q.as_ref();
    let qw = qr.w;
    let qx = qr.i;
    let qy = qr.j;
    let qz = qr.k;

    let sqw = qw * qw;
    let sqx = qx * qx;
    let sqy = qy * qy;
    let sqz = qz * qz;
    let unit = sqx + sqy + sqz + sqw;
    let test = qw * qy - qz * qx;

    let ang: V3D = if test > 0.49999 * unit {
        V3D::new(2.0 * qx.atan2(qw), std::f64::consts::PI / 2.0, 0.0)
    } else if test < -0.49999 * unit {
        V3D::new(-2.0 * qx.atan2(qw), -std::f64::consts::PI / 2.0, 0.0)
    } else {
        V3D::new(
            (2.0 * qx * qw + 2.0 * qy * qz).atan2(-sqx - sqy + sqz + sqw),
            (2.0 * test / unit).asin(),
            (2.0 * qz * qw + 2.0 * qy * qx).atan2(sqx - sqy - sqz + sqw),
        )
    };
    ang * 57.3
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::math::manifold::quat_exp;
    use nalgebra::Vector3;

    const TOL: f64 = 1e-9;

    #[allow(clippy::field_reassign_with_default)]
    fn sample() -> (StateIkfom, InputIkfom) {
        let mut s = StateIkfom::default();
        s.pos = Vector3::new(1.0, 2.0, 3.0);
        s.vel = Vector3::new(0.5, -0.5, 0.1);
        s.bg = Vector3::new(0.01, -0.02, 0.03);
        s.ba = Vector3::new(0.1, 0.05, -0.05);
        s.rot = quat_exp(&Vector3::new(0.3, -0.2, 0.1));
        s.offset_t_l_i = Vector3::new(0.04, 0.02, -0.03);
        s.grav = crate::math::s2::S2::from_vec(
            &Vector3::new(0.2, -0.3, 9.79),
            crate::math::manifold::S2_GRAV_LENGTH,
            crate::math::manifold::S2_GRAV_TYP,
        );
        let input = InputIkfom {
            acc: Vector3::new(1.1, -0.4, 9.2),
            gyro: Vector3::new(0.2, -0.1, 0.3),
        };
        (s, input)
    }

    #[test]
    fn process_noise_cov_diagonal() {
        let q = process_noise_cov();
        assert_eq!(q.shape(), (12, 12));
        for i in 0..12 {
            assert!(q[(i, i)] > 0.0);
        }
        assert_eq!(q[(0, 0)], NOISE_NG);
        assert_eq!(q[(3, 3)], NOISE_NA);
        assert_eq!(q[(6, 6)], NOISE_NBG);
        assert_eq!(q[(9, 9)], NOISE_NBA);
    }

    #[test]
    fn get_f_layout() {
        let (s, input) = sample();
        let f = get_f(&s, &input);
        // vel
        assert!((Vector3::new(f[0], f[1], f[2]) - s.vel).norm() < TOL);
        // omega = gyro - bg
        let omega = input.gyro - s.bg;
        assert!((Vector3::new(f[3], f[4], f[5]) - omega).norm() < TOL);
        // a_inertial + grav
        let a = s.rot * (input.acc - s.ba) + s.grav.vec;
        assert!((Vector3::new(f[12], f[13], f[14]) - a).norm() < TOL);
        // all other rows zero
        for i in [6, 7, 8, 9, 10, 11, 15, 16, 17, 18, 19, 20, 21, 22, 23] {
            assert_eq!(f[i], 0.0);
        }
    }

    #[test]
    fn df_dx_numeric_check() {
        let (s, input) = sample();
        let jac = df_dx(&s, &input);
        // h must be > 1e-7 (so3::exp threshold) yet small for finite-difference accuracy
        let h = 1e-6;
        // numeric directional derivative along each state axis
        for k in 0..STATE_DOF {
            let mut dx = [0.0; STATE_DOF];
            dx[k] = h;
            let mut sp = s.clone();
            sp.boxplus(&dx);
            let mut sm = s.clone();
            let mut dxn = dx;
            dxn[k] = -h;
            sm.boxplus(&dxn);
            let fp = get_f(&sp, &input);
            let fm = get_f(&sm, &input);
            for i in 0..STATE_DIM {
                let num = (fp[i] - fm[i]) / (2.0 * h);
                let ana = jac[(i, k)];
                // grav entries use S2 tangent; both are w.r.t. delta, so comparable
                assert!(
                    (num - ana).abs() < 1e-6,
                    "dfdx[{i},{k}] num={num} ana={ana}"
                );
            }
        }
    }

    #[test]
    fn df_dw_numeric_check() {
        let (s, input) = sample();
        let jac = df_dw(&s, &input);
        // noise enters additively in f: check the constant blocks directly
        // d(omega)/d(ng) = -I
        for i in 0..3 {
            for j in 0..3 {
                let expected = if i == j { -1.0 } else { 0.0 };
                assert!((jac[(3 + i, j)] - expected).abs() < TOL);
            }
        }
        // d(bg)/d(nbg) = I
        for i in 0..3 {
            for j in 0..3 {
                let expected = if i == j { 1.0 } else { 0.0 };
                assert!((jac[(15 + i, 6 + j)] - expected).abs() < TOL);
            }
        }
        // d(ba)/d(nba) = I
        for i in 0..3 {
            for j in 0..3 {
                let expected = if i == j { 1.0 } else { 0.0 };
                assert!((jac[(18 + i, 9 + j)] - expected).abs() < TOL);
            }
        }
    }

    #[test]
    fn so3_to_euler_identity() {
        let e = so3_to_euler(&UnitQuaternion::identity());
        assert!(e.norm() < 1e-9);
    }

    #[test]
    fn so3_to_euler_known() {
        // 90 deg about z
        let q = quat_exp(&Vector3::new(0.0, 0.0, std::f64::consts::PI / 2.0));
        let e = so3_to_euler(&q);
        // allow small numerical noise (~1e-4 rad) in the atan2 conversions
        assert!(e[0].abs() < 0.1, "euler={e}");
        assert!(e[1].abs() < 0.1, "euler={e}");
        assert!((e[2] - 90.0).abs() < 0.1, "euler={e}");
    }
}
