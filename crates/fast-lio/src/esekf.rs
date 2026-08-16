use std::time::Instant;

use nalgebra::{DMatrix, DVector, SMatrix, SVector, Vector2};

use crate::math::manifold::{quat_exp, StateIkfom, StateVec};
use crate::math::so3::{a_matrix, exp, V3D};
use crate::model::{df_dx, df_dw, get_f, InputIkfom};

/// State DOF (state_ikfom).
pub const STATE_N: usize = 23;
/// Process noise DOF.
pub const PROC_N: usize = 12;

/// vect sub-state layout: (DOF index, DIM index, DOF).
const VECT_STATES: [(usize, usize, usize); 5] = [
    (0, 0, 3),   // pos
    (9, 9, 3),   // offset_T_L_I
    (12, 12, 3), // vel
    (15, 15, 3), // bg
    (18, 18, 3), // ba
];

/// SO(3) sub-state layout: (DOF index, DIM index).
const SO3_STATES: [(usize, usize); 2] = [
    (3, 3), // rot
    (6, 6), // offset_R_L_I
];

/// S2 sub-state layout: (DOF index, DIM index).
const S2_STATES: [(usize, usize); 1] = [(21, 21)]; // grav

/// Covariance type.
pub type Cov = SMatrix<f64, STATE_N, STATE_N>;

/// Shared data passed to the measurement model (`dyn_share_datastruct`).
pub struct DynShareData {
    pub valid: bool,
    pub converge: bool,
    /// Residual vector (length = number of effective feature points).
    pub h: DVector<f64>,
    /// Measurement Jacobian, `m x 12` (the 12 pose / extrinsic DOF).
    pub h_x: DMatrix<f64>,
}

/// Iterated error-state Kalman filter (`esekfom::esekf`), FAST-LIO2 instantiation
/// with `state = state_ikfom` (23 DOF), `process_noise_dof = 12`, `input = input_ikfom`.
pub struct EseKf {
    pub x: StateIkfom,
    pub p: Cov,
    pub maximum_iter: usize,
    pub limit: [f64; STATE_N],
}

impl EseKf {
    pub fn new(x: StateIkfom, p: Cov) -> Self {
        Self {
            x,
            p,
            maximum_iter: 4,
            limit: [0.001; STATE_N],
        }
    }

    pub fn get_x(&self) -> &StateIkfom {
        &self.x
    }

    pub fn get_p(&self) -> &Cov {
        &self.p
    }

    pub fn change_x(&mut self, x: StateIkfom) {
        self.x = x;
    }

    pub fn change_p(&mut self, p: Cov) {
        self.p = p;
    }

    /// Forward propagation (`predict`). Integrates the continuous-time model by
    /// `dt` and updates the covariance via `F·P·Fᵀ + Q`.
    pub fn predict(&mut self, dt: f64, q: &SMatrix<f64, PROC_N, PROC_N>, input: &InputIkfom) {
        let f_ = get_f(&self.x, input);
        let f_x_ = df_dx(&self.x, input);
        let f_w_ = df_dw(&self.x, input);

        let mut f_x_final = SMatrix::<f64, STATE_N, STATE_N>::zeros();
        let mut f_w_final = SMatrix::<f64, STATE_N, PROC_N>::zeros();
        let x_before = self.x.clone();

        // oplus: x = x ⊕ (f * dt)
        {
            let x = &mut self.x;
            x.pos += V3D::new(f_[0], f_[1], f_[2]) * dt;
            x.rot *= quat_exp(&(V3D::new(f_[3], f_[4], f_[5]) * dt));
            x.offset_r_l_i *= quat_exp(&(V3D::new(f_[6], f_[7], f_[8]) * dt));
            x.offset_t_l_i += V3D::new(f_[9], f_[10], f_[11]) * dt;
            x.vel += V3D::new(f_[12], f_[13], f_[14]) * dt;
            x.bg += V3D::new(f_[15], f_[16], f_[17]) * dt;
            x.ba += V3D::new(f_[18], f_[19], f_[20]) * dt;
            // S2 oplus: vec = Exp(Δ·dt)·vec (f_ grav rows are zero here, kept faithful)
            x.grav.oplus(&V3D::new(f_[21], f_[22], f_[23]), dt);
        }

        let mut f_x1 = Cov::identity();

        // vect sub-states: copy DIM-indexed rows into DOF-indexed rows
        for &(idx, dim, dof) in &VECT_STATES {
            for i in 0..STATE_N {
                for j in 0..dof {
                    f_x_final[(idx + j, i)] = f_x_[(dim + j, i)];
                }
            }
            for i in 0..PROC_N {
                for j in 0..dof {
                    f_w_final[(idx + j, i)] = f_w_[(dim + j, i)];
                }
            }
        }

        // SO(3) sub-states
        for &(idx, dim) in &SO3_STATES {
            let seg = -V3D::new(f_[dim], f_[dim + 1], f_[dim + 2]) * dt;
            f_x1.fixed_view_mut::<3, 3>(idx, idx).copy_from(&exp(&seg));
            let a = a_matrix(&seg);
            for i in 0..STATE_N {
                let col = a * f_x_.fixed_view::<3, 1>(dim, i);
                f_x_final.fixed_view_mut::<3, 1>(idx, i).copy_from(&col);
            }
            for i in 0..PROC_N {
                let col = a * f_w_.fixed_view::<3, 1>(dim, i);
                f_w_final.fixed_view_mut::<3, 1>(idx, i).copy_from(&col);
            }
        }

        // S2 sub-state (gravity)
        for &(idx, dim) in &S2_STATES {
            let seg = V3D::new(f_[dim], f_[dim + 1], f_[dim + 2]) * dt;
            let res_mat = exp(&seg);
            let nx = self.x.grav.s2_nx_yy();
            let mx = x_before.grav.s2_mx(&Vector2::zeros());
            f_x1.fixed_view_mut::<2, 2>(idx, idx).copy_from(&(nx * res_mat * mx));
            let x_before_hat = x_before.grav.s2_hat();
            let res_temp_s2 = -nx * res_mat * x_before_hat * a_matrix(&seg).transpose();
            for i in 0..STATE_N {
                let col = res_temp_s2 * f_x_.fixed_view::<3, 1>(dim, i);
                f_x_final.fixed_view_mut::<2, 1>(idx, i).copy_from(&col);
            }
            for i in 0..PROC_N {
                let col = res_temp_s2 * f_w_.fixed_view::<3, 1>(dim, i);
                f_w_final.fixed_view_mut::<2, 1>(idx, i).copy_from(&col);
            }
        }

        f_x1 += f_x_final * dt;
        let fw = dt * f_w_final;
        self.p = f_x1 * self.p * f_x1.transpose() + fw * q * fw.transpose();
    }

    /// Iterated error-state EKF update with dynamic measurement (`update_iterated_dyn_share_modified`).
    /// `h_share` fills `DynShareData` given the current state.
    pub fn update_iterated_dyn_share_modified<F>(
        &mut self,
        r: f64,
        solve_time: &mut f64,
        mut h_share: F,
    ) where
        F: FnMut(&StateIkfom, &mut DynShareData),
    {
        let mut dyn_share = DynShareData {
            valid: true,
            converge: true,
            h: DVector::zeros(0),
            h_x: DMatrix::zeros(0, 12),
        };
        let mut t = 0usize;
        let x_propagated = self.x.clone();
        let p_propagated = self.p;
        let mut k_h = SVector::<f64, STATE_N>::zeros();
        let mut k_x = SMatrix::<f64, STATE_N, STATE_N>::zeros();
        #[allow(unused_assignments)]
        let mut dx_new = SVector::<f64, STATE_N>::zeros();

        let maximum_iter = self.maximum_iter as i32;
        for i in -1..maximum_iter {
            dyn_share.valid = true;
            h_share(&self.x, &mut dyn_share);
            if !dyn_share.valid {
                continue;
            }

            let solve_start = Instant::now();
            let dof_measurement = dyn_share.h_x.nrows();

            let dx = self.x.boxminus(&x_propagated);
            let dxv = SVector::<f64, STATE_N>::from_column_slice(&dx);
            dx_new = dxv;

            self.p = p_propagated;

            // SO(3) correction of dx_new and P under a right perturbation
            for &(idx, _dim) in &SO3_STATES {
                let seg = V3D::new(dxv[idx], dxv[idx + 1], dxv[idx + 2]);
                let a = a_matrix(&seg).transpose();
                let dn = a * dx_new.fixed_view::<3, 1>(idx, 0);
                dx_new.fixed_view_mut::<3, 1>(idx, 0).copy_from(&dn);
                for i in 0..STATE_N {
                    let col = a * self.p.fixed_view::<3, 1>(idx, i);
                    self.p.fixed_view_mut::<3, 1>(idx, i).copy_from(&col);
                }
                for i in 0..STATE_N {
                    let row = self.p.fixed_view::<1, 3>(i, idx) * a.transpose();
                    self.p.fixed_view_mut::<1, 3>(i, idx).copy_from(&row);
                }
            }

            // S2 correction
            for &(idx, _dim) in &S2_STATES {
                let seg2 = Vector2::new(dxv[idx], dxv[idx + 1]);
                let nx = self.x.grav.s2_nx_yy();
                let mx = x_propagated.grav.s2_mx(&seg2);
                let res = nx * mx;
                let dn = res * dx_new.fixed_view::<2, 1>(idx, 0);
                dx_new.fixed_view_mut::<2, 1>(idx, 0).copy_from(&dn);
                for i in 0..STATE_N {
                    let col = res * self.p.fixed_view::<2, 1>(idx, i);
                    self.p.fixed_view_mut::<2, 1>(idx, i).copy_from(&col);
                }
                for i in 0..STATE_N {
                    let row = self.p.fixed_view::<1, 2>(i, idx) * res.transpose();
                    self.p.fixed_view_mut::<1, 2>(i, idx).copy_from(&row);
                }
            }

            // Kalman gain
            if STATE_N > dof_measurement {
                let mut h_x_cur = DMatrix::<f64>::zeros(dof_measurement, STATE_N);
                h_x_cur
                    .view_mut((0, 0), (dof_measurement, 12))
                    .copy_from(&dyn_share.h_x);
                let hph = &h_x_cur * self.p * h_x_cur.transpose();
                let innov_cov =
                    (hph / r + DMatrix::<f64>::identity(dof_measurement, dof_measurement))
                        .try_inverse()
                        .expect("innovation covariance singular");
                let k_ = self.p * h_x_cur.transpose() * innov_cov / r;
                let kres_h = &k_ * &dyn_share.h;
                for i in 0..STATE_N {
                    k_h[i] = kres_h[i];
                }
                let kres = k_ * h_x_cur;
                k_x.fill(0.0);
                for i in 0..STATE_N {
                    for j in 0..dof_measurement {
                        k_x[(i, j)] = kres[(i, j)];
                    }
                }
            } else {
                let mut p_temp = (self.p / r).try_inverse().expect("P/R singular");
                let hth = dyn_share.h_x.transpose() * &dyn_share.h_x;
                let mut addend = DMatrix::<f64>::zeros(STATE_N, STATE_N);
                addend.view_mut((0, 0), (12, 12)).copy_from(&hth);
                p_temp += addend;
                let p_inv = p_temp.try_inverse().expect("P_inv singular");
                let tmp: DVector<f64> =
                    p_inv.view((0, 0), (STATE_N, 12)) * dyn_share.h_x.transpose() * &dyn_share.h;
                for i in 0..STATE_N {
                    k_h[i] = tmp[i];
                }
                k_x.fill(0.0);
                let blk: DMatrix<f64> = p_inv.view((0, 0), (STATE_N, 12)) * hth;
                k_x.view_mut((0, 0), (STATE_N, 12)).copy_from(&blk);
            }

            let dx_ = k_h + (k_x - SMatrix::<f64, STATE_N, STATE_N>::identity()) * dx_new;
            let dx_arr: StateVec = dx_.as_slice().try_into().expect("length 23");
            self.x.boxplus(&dx_arr);

            dyn_share.converge = true;
            for k in 0..STATE_N {
                if dx_[k].abs() > self.limit[k] {
                    dyn_share.converge = false;
                    break;
                }
            }
            if dyn_share.converge {
                t += 1;
            }

            // force "converged" so the final covariance correction runs near the end
            if t == 0 && i == maximum_iter - 2 {
                dyn_share.converge = true;
            }

            if t > 1 || i == maximum_iter - 1 {
                let mut l_ = self.p;

                // SO(3) correction of L, P and K_x
                for &(idx, _dim) in &SO3_STATES {
                    let seg = V3D::new(dx_[idx], dx_[idx + 1], dx_[idx + 2]);
                    let a = a_matrix(&seg).transpose();
                    for i in 0..STATE_N {
                        let col = a * self.p.fixed_view::<3, 1>(idx, i);
                        l_.fixed_view_mut::<3, 1>(idx, i).copy_from(&col);
                    }
                    for i in 0..12 {
                        let col = a * k_x.fixed_view::<3, 1>(idx, i);
                        k_x.fixed_view_mut::<3, 1>(idx, i).copy_from(&col);
                    }
                    for i in 0..STATE_N {
                        let row_l = l_.fixed_view::<1, 3>(i, idx) * a.transpose();
                        l_.fixed_view_mut::<1, 3>(i, idx).copy_from(&row_l);
                        let row_p = self.p.fixed_view::<1, 3>(i, idx) * a.transpose();
                        self.p.fixed_view_mut::<1, 3>(i, idx).copy_from(&row_p);
                    }
                }

                // S2 correction of L, P and K_x
                for &(idx, _dim) in &S2_STATES {
                    let seg2 = Vector2::new(dx_[idx], dx_[idx + 1]);
                    let nx = self.x.grav.s2_nx_yy();
                    let mx = x_propagated.grav.s2_mx(&seg2);
                    let res = nx * mx;
                    for i in 0..STATE_N {
                        let col = res * self.p.fixed_view::<2, 1>(idx, i);
                        l_.fixed_view_mut::<2, 1>(idx, i).copy_from(&col);
                    }
                    for i in 0..12 {
                        let col = res * k_x.fixed_view::<2, 1>(idx, i);
                        k_x.fixed_view_mut::<2, 1>(idx, i).copy_from(&col);
                    }
                    for i in 0..STATE_N {
                        let row_l = l_.fixed_view::<1, 2>(i, idx) * res.transpose();
                        l_.fixed_view_mut::<1, 2>(i, idx).copy_from(&row_l);
                        let row_p = self.p.fixed_view::<1, 2>(i, idx) * res.transpose();
                        self.p.fixed_view_mut::<1, 2>(i, idx).copy_from(&row_p);
                    }
                }

                // FAST-LIO2 modified covariance update: only the first 12 DOF
                let kx12 = k_x.view((0, 0), (STATE_N, 12));
                let p12 = self.p.view((0, 0), (12, STATE_N));
                self.p = l_ - kx12 * p12;

                *solve_time += solve_start.elapsed().as_secs_f64();
                return;
            }
            *solve_time += solve_start.elapsed().as_secs_f64();
        }
    }
}

impl Default for EseKf {
    fn default() -> Self {
        Self::new(StateIkfom::default(), Cov::identity())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::Vector3;

    #[test]
    fn predict_zero_input_keeps_state() {
        let mut kf = EseKf::default();
        let q = crate::model::process_noise_cov();
        let input = InputIkfom::default();
        let pos_before = kf.x.pos;
        let vel_before = kf.x.vel;
        kf.predict(0.1, &q, &input);
        // f = (0,0,0, 0,0,0, ..., grav, ...) -> vel += grav*dt, pos unchanged
        assert!((kf.x.pos - pos_before).norm() < 1e-12);
        // a = R*(acc-ba) + grav = grav.vec = (9.809, 0, 0)
        let expected = vel_before + V3D::new(9.809, 0.0, 0.0) * 0.1;
        assert!((kf.x.vel - expected).norm() < 1e-9);
        // P stays symmetric
        assert!((kf.p - kf.p.transpose()).norm() < 1e-9);
    }

    #[test]
    fn predict_gravity_noop() {
        let mut kf = EseKf::default();
        let q = crate::model::process_noise_cov();
        let input = InputIkfom::default();
        let grav_before = kf.x.grav.vec;
        kf.predict(0.5, &q, &input);
        assert!((kf.x.grav.vec - grav_before).norm() < 1e-12);
    }

    #[test]
    fn predict_updates_covariance_formula() {
        let mut kf = EseKf::default();
        let q = crate::model::process_noise_cov();
        let input = InputIkfom::default();
        let p_before = kf.p;
        kf.predict(0.01, &q, &input);
        // P must remain symmetric positive (trace grows slightly due to Q)
        let diff = kf.p - kf.p.transpose();
        assert!(diff.norm() < 1e-9);
        assert!(kf.p.trace() > 0.0);
        // P cannot shrink: F·P·Fᵀ + noise >= noise
        assert!(kf.p.trace() >= p_before.trace() - 1e-12);
    }

    #[test]
    fn update_pulls_position_to_zero() {
        let mut kf = EseKf::default();
        kf.x.pos = Vector3::new(1.0, 2.0, 3.0);
        let mut solve_time = 0.0;
        let r = 0.001; // LASER_POINT_COV
        kf.update_iterated_dyn_share_modified(r, &mut solve_time, |x, share| {
            // residual r = pos (want -> 0); convention: h = -residual
            share.h = DVector::from_vec(vec![-x.pos[0], -x.pos[1], -x.pos[2]]);
            let mut hx = DMatrix::zeros(3, 12);
            hx[(0, 0)] = 1.0;
            hx[(1, 1)] = 1.0;
            hx[(2, 2)] = 1.0;
            share.h_x = hx;
            share.valid = true;
        });
        assert!(kf.x.pos.norm() < 0.05, "pos = {}", kf.x.pos);
        // P updated and symmetric
        assert!((kf.p - kf.p.transpose()).norm() < 1e-9);
        // state must stay a valid manifold (unit quaternions)
        assert!((kf.x.rot.norm() - 1.0).abs() < 1e-9);
        assert!(solve_time >= 0.0);
    }

    #[test]
    fn update_does_not_touch_grav() {
        // measurement only couples to pos/rot -> grav must remain unchanged
        let mut kf = EseKf::default();
        kf.x.pos = Vector3::new(0.5, -0.5, 0.5);
        let grav_before = kf.x.grav.vec;
        let mut solve_time = 0.0;
        kf.update_iterated_dyn_share_modified(0.001, &mut solve_time, |x, share| {
            share.h = DVector::from_vec(vec![-x.pos[0], -x.pos[1], -x.pos[2]]);
            let mut hx = DMatrix::zeros(3, 12);
            hx[(0, 0)] = 1.0;
            hx[(1, 1)] = 1.0;
            hx[(2, 2)] = 1.0;
            share.h_x = hx;
            share.valid = true;
        });
        assert!((kf.x.grav.vec - grav_before).norm() < 1e-9);
    }

    #[test]
    fn invalid_measurement_is_skipped() {
        let mut kf = EseKf::default();
        kf.x.pos = Vector3::new(1.0, 0.0, 0.0);
        let mut solve_time = 0.0;
        let mut calls = 0;
        kf.update_iterated_dyn_share_modified(0.001, &mut solve_time, |_x, share| {
            calls += 1;
            share.valid = false;
        });
        // C++ behavior: on invalid the iteration `continue`s but the loop still
        // invokes the measurement model every iteration (maximum_iter + 1 calls)
        assert_eq!(calls, kf.maximum_iter + 1);
        // state and P unchanged
        assert!((kf.x.pos - Vector3::new(1.0, 0.0, 0.0)).norm() < 1e-12);
        assert!(solve_time == 0.0);
    }

    #[test]
    fn measurement_rotation_jacobian_moves_rot() {
        // A measurement sensing yaw (rot DOF index 5) with residual = -yaw
        let mut kf = EseKf::default();
        kf.x.rot = quat_exp(&Vector3::new(0.0, 0.0, 0.5)); // 0.5 rad yaw
        let mut solve_time = 0.0;
        kf.update_iterated_dyn_share_modified(0.001, &mut solve_time, |x, share| {
            let e = crate::model::so3_to_euler(&x.rot);
            share.h = DVector::from_vec(vec![-e[2] * std::f64::consts::PI / 180.0]);
            // h_x row: ∂r/∂[pos, rot, offsetR, offsetT] -> only rot yaw (col 5)
            let mut hx = DMatrix::zeros(1, 12);
            hx[(0, 5)] = 1.0;
            share.h_x = hx;
            share.valid = true;
        });
        // yaw should shrink toward 0
        let e = crate::model::so3_to_euler(&kf.x.rot);
        assert!(e[2].abs() < 10.0, "yaw = {}", e[2]);
    }
}
