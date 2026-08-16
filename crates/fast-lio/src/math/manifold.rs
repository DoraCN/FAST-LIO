use nalgebra::{UnitQuaternion};

use super::s2::S2;
use super::so3::{exp, V3D};

/// DOF count of `state_ikfom` (23). DIM is 24 (grav embedded in R³).
pub const STATE_DOF: usize = 23;
pub const STATE_DIM: usize = 24;

/// DOF layout of `state_ikfom`, ported from `MTK_BUILD_MANIFOLD` in
/// `use-ikfom.hpp`. Order and indices must match exactly.
pub const IDX_POS: usize = 0; // 0..3
pub const IDX_ROT: usize = 3; // 3..6
pub const IDX_OFFSET_R: usize = 6; // 6..9
pub const IDX_OFFSET_T: usize = 9; // 9..12
pub const IDX_VEL: usize = 12; // 12..15
pub const IDX_BG: usize = 15; // 15..18
pub const IDX_BA: usize = 18; // 18..21
pub const IDX_GRAV: usize = 21; // 21..23 (S2, 2-DOF)

/// Gravity length for the S2 alias `S2<double, 98090, 10000, 1>` = 9.809.
pub const S2_GRAV_LENGTH: f64 = 9.809;
/// Base axis typ for the gravity S2 alias.
pub const S2_GRAV_TYP: i32 = 1;

/// 23-DOF error-state vector.
pub type StateVec = [f64; STATE_DOF];

/// `state_ikfom` manifold. Represents the full EKF state.
#[derive(Clone, Debug)]
pub struct StateIkfom {
    pub pos: V3D,
    pub rot: UnitQuaternion<f64>,
    pub offset_r_l_i: UnitQuaternion<f64>,
    pub offset_t_l_i: V3D,
    pub vel: V3D,
    pub bg: V3D,
    pub ba: V3D,
    pub grav: S2,
}

impl Default for StateIkfom {
    fn default() -> Self {
        Self {
            pos: V3D::zeros(),
            rot: UnitQuaternion::identity(),
            offset_r_l_i: UnitQuaternion::identity(),
            offset_t_l_i: V3D::zeros(),
            vel: V3D::zeros(),
            bg: V3D::zeros(),
            ba: V3D::zeros(),
            grav: S2::new_with_type(S2_GRAV_LENGTH, S2_GRAV_TYP),
        }
    }
}

/// Quaternion exponential: `q = [cos(θ/2), sin(θ/2)·unit(v)]` for θ = |v|.
/// This matches `MTK::SO3::exp(vec)` used by `boxplus`.
pub fn quat_exp(v: &V3D) -> UnitQuaternion<f64> {
    let ang = v.norm();
    if ang > 1e-9 {
        UnitQuaternion::from_axis_angle(&nalgebra::Unit::new_normalize(*v), ang)
    } else {
        UnitQuaternion::identity()
    }
}

/// Quaternion logarithm matching `MTK::SO3::log` (scale=2, periodic):
/// `res = 2/|vec| · atan(|vec|/w) · vec`.
pub fn quat_log(q: &UnitQuaternion<f64>) -> V3D {
    let qr = q.as_ref();
    let w = qr.w;
    let vec = V3D::new(qr.i, qr.j, qr.k);
    let nv = vec.norm();
    let nv = if nv < 1e-11 { 1e-11 } else { nv };
    2.0 / nv * (nv / w).atan() * vec
}

impl StateIkfom {
    /// Manifold addition `x ⊞ dx` (MTK `boxplus`).
    pub fn boxplus(&mut self, dx: &StateVec) {
        self.pos += V3D::new(dx[0], dx[1], dx[2]);
        self.rot *= quat_exp(&V3D::new(dx[3], dx[4], dx[5]));
        self.offset_r_l_i *= quat_exp(&V3D::new(dx[6], dx[7], dx[8]));
        self.offset_t_l_i += V3D::new(dx[9], dx[10], dx[11]);
        self.vel += V3D::new(dx[12], dx[13], dx[14]);
        self.bg += V3D::new(dx[15], dx[16], dx[17]);
        self.ba += V3D::new(dx[18], dx[19], dx[20]);
        self.grav.boxplus(&nalgebra::Vector2::new(dx[21], dx[22]));
    }

    /// Manifold subtraction `x ⊟ other` (MTK `boxminus`).
    pub fn boxminus(&self, other: &Self) -> StateVec {
        let mut res = [0.0; STATE_DOF];
        let dp = self.pos - other.pos;
        res[0..3].copy_from_slice(dp.as_slice());
        let dr = quat_log(&(other.rot.conjugate() * self.rot));
        res[3..6].copy_from_slice(dr.as_slice());
        let dorf = quat_log(&(other.offset_r_l_i.conjugate() * self.offset_r_l_i));
        res[6..9].copy_from_slice(dorf.as_slice());
        let dt = self.offset_t_l_i - other.offset_t_l_i;
        res[9..12].copy_from_slice(dt.as_slice());
        let dv = self.vel - other.vel;
        res[12..15].copy_from_slice(dv.as_slice());
        let dbg = self.bg - other.bg;
        res[15..18].copy_from_slice(dbg.as_slice());
        let dba = self.ba - other.ba;
        res[18..21].copy_from_slice(dba.as_slice());
        let dg = self.grav.boxminus(&other.grav);
        res[21] = dg[0];
        res[22] = dg[1];
        res
    }

    /// Flatten to the 24-DIM embedded representation (grav uses its 3-vector).
    pub fn to_dim24(&self) -> [f64; STATE_DIM] {
        let mut v = [0.0; STATE_DIM];
        v[0..3].copy_from_slice(self.pos.as_slice());
        v[3..6].copy_from_slice(quat_log(&self.rot).as_slice());
        v[6..9].copy_from_slice(quat_log(&self.offset_r_l_i).as_slice());
        v[9..12].copy_from_slice(self.offset_t_l_i.as_slice());
        v[12..15].copy_from_slice(self.vel.as_slice());
        v[15..18].copy_from_slice(self.bg.as_slice());
        v[18..21].copy_from_slice(self.ba.as_slice());
        v[21..24].copy_from_slice(self.grav.vec.as_slice());
        v
    }

    /// Helper: multiply self by `exp` of rotation-vector segment (used by esekfom
    /// oplus). Left for completeness; esekfom uses `boxplus` directly.
    pub fn oplus_rot(&mut self, rot_vec: &V3D) {
        self.rot *= quat_exp(rot_vec);
    }

    /// `Exp(v)` on rotation matrix, re-exported for convenience in this module.
    pub fn rot_exp(v: &V3D) -> nalgebra::Matrix3<f64> {
        exp(v)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::{Matrix3, Vector3};

    const TOL: f64 = 1e-6;

    #[allow(clippy::field_reassign_with_default)]
    fn rand_state() -> StateIkfom {
        let mut s = StateIkfom::default();
        s.pos = Vector3::new(1.0, 2.0, 3.0);
        s.vel = Vector3::new(0.5, -0.5, 0.1);
        s.bg = Vector3::new(0.01, -0.02, 0.03);
        s.ba = Vector3::new(0.1, 0.05, -0.05);
        s.rot = quat_exp(&Vector3::new(0.3, -0.2, 0.1));
        s.offset_r_l_i = quat_exp(&Vector3::new(0.1, 0.2, -0.3));
        s.offset_t_l_i = Vector3::new(0.04, 0.02, -0.03);
        s.grav = S2::from_vec(&Vector3::new(0.2, -0.3, 9.79), S2_GRAV_LENGTH, S2_GRAV_TYP);
        s
    }

    #[test]
    fn default_state_layout() {
        let s = StateIkfom::default();
        // grav base vector = length * (1,0,0) for typ=1
        assert!((s.grav.vec.norm() - S2_GRAV_LENGTH).abs() < TOL);
        assert!(s.grav.vec[0] > 0.0);
        assert_eq!(s.grav.vec[1], 0.0);
        assert_eq!(s.grav.vec[2], 0.0);
    }

    #[test]
    fn boxplus_updates_fields() {
        let mut s = rand_state();
        let dx: StateVec = [
            0.1, 0.2, 0.3, // pos
            0.01, 0.02, 0.03, // rot
            0.0, 0.0, 0.0, // offset_R
            0.1, 0.0, 0.0, // offset_T
            0.0, 0.0, 0.0, // vel
            0.0, 0.0, 0.0, // bg
            0.0, 0.0, 0.0, // ba
            0.0, 0.0, // grav
        ];
        let before_pos = s.pos;
        s.boxplus(&dx);
        assert!((s.pos - (before_pos + Vector3::new(0.1, 0.2, 0.3))).norm() < TOL);
        assert!(s.offset_t_l_i[0] > 0.099);
    }

    #[test]
    fn boxplus_rot_has_unit_norm() {
        let mut s = rand_state();
        s.boxplus(&[0.5, 0.4, 0.3, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]);
        assert!((s.rot.norm() - 1.0).abs() < TOL);
    }

    #[test]
    fn boxminus_after_boxplus_roundtrip() {
        let base = rand_state();
        let mut moved = base.clone();
        let dx: StateVec = [
            0.1, -0.2, 0.3, 0.05, 0.02, -0.01, 0.0, 0.0, 0.0, 0.02, 0.0, -0.01, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, -0.05,
        ];
        moved.boxplus(&dx);
        let back = moved.boxminus(&base);
        for i in 0..STATE_DOF {
            assert!((back[i] - dx[i]).abs() < 1e-3, "mismatch at {}: {} vs {}", i, back[i], dx[i]);
        }
    }

    #[test]
    fn quat_exp_log_roundtrip() {
        let v = Vector3::new(0.4, -0.3, 0.2);
        let q = quat_exp(&v);
        let back = quat_log(&q);
        assert!((back - v).norm() < TOL);
    }

    #[test]
    fn rot_quat_rotation_matrix_consistent() {
        let v = Vector3::new(0.6, -0.4, 0.3);
        let q = quat_exp(&v);
        let rm = q.to_rotation_matrix();
        let rm2 = Matrix3::from(rm);
        let direct = exp(&v);
        assert!((rm2 - direct).norm() < TOL);
    }
}
