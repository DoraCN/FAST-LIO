use nalgebra::{Matrix2x3, Matrix3x2, Vector2};

use super::so3::{skew, a_matrix, exp, M3D, V3D};

/// S2 manifold: unit-length directions on the sphere, represented by an
/// embedded 3-vector of fixed length `length`. Ported from
/// `IKFoM_toolkit/mtk/types/S2.hpp`.
///
/// In FAST-LIO the type alias is `S2<double, 98090, 10000, 1>`, i.e. length =
/// 9.809 (the local gravity magnitude) and `typ = 1` (base axis = x).
#[derive(Clone, Debug)]
pub struct S2 {
    pub vec: V3D,
    pub length: f64,
    pub typ: i32,
}

/// MTK `tolerance<double>()`.
pub const S2_TOLERANCE: f64 = 1e-11;

impl S2 {
    /// Default constructor matching `MTK::S2()` with the given `typ` (1/2/3
    /// selects the x/y/z base axis).
    pub fn new_with_type(length: f64, typ: i32) -> Self {
        let base = match typ {
            3 => V3D::new(0.0, 0.0, 1.0),
            2 => V3D::new(0.0, 1.0, 0.0),
            _ => V3D::new(1.0, 0.0, 0.0),
        };
        Self { vec: length * base, length, typ }
    }

    /// Construct from an arbitrary 3-vector: normalized then scaled to `length`.
    pub fn from_vec(vec: &V3D, length: f64, typ: i32) -> Self {
        let n = vec.norm();
        let unit = if n > 0.0 { *vec / n } else { V3D::new(1.0, 0.0, 0.0) };
        Self { vec: length * unit, length, typ }
    }

    /// Tangent-space basis `Bx` (`S2_Bx`, 3x2).
    pub fn s2_bx(&self) -> Matrix3x2<f64> {
        let l = self.length;
        let v = &self.vec;
        let mut res = Matrix3x2::zeros();
        match self.typ {
            3 => {
                if v[2] + l > S2_TOLERANCE {
                    res[(0, 0)] = l - v[0] * v[0] / (l + v[2]);
                    res[(0, 1)] = -v[0] * v[1] / (l + v[2]);
                    res[(1, 0)] = -v[0] * v[1] / (l + v[2]);
                    res[(1, 1)] = l - v[1] * v[1] / (l + v[2]);
                    res[(2, 0)] = -v[0];
                    res[(2, 1)] = -v[1];
                    res /= l;
                } else {
                    res[(1, 1)] = -1.0;
                    res[(2, 0)] = 1.0;
                }
            }
            2 => {
                if v[1] + l > S2_TOLERANCE {
                    res[(0, 0)] = l - v[0] * v[0] / (l + v[1]);
                    res[(0, 1)] = -v[0] * v[2] / (l + v[1]);
                    res[(1, 0)] = -v[0];
                    res[(1, 1)] = -v[2];
                    res[(2, 0)] = -v[0] * v[2] / (l + v[1]);
                    res[(2, 1)] = l - v[2] * v[2] / (l + v[1]);
                    res /= l;
                } else {
                    res[(1, 1)] = -1.0;
                    res[(2, 0)] = 1.0;
                }
            }
            _ => {
                if v[0] + l > S2_TOLERANCE {
                    res[(0, 0)] = -v[1];
                    res[(0, 1)] = -v[2];
                    res[(1, 0)] = l - v[1] * v[1] / (l + v[0]);
                    res[(1, 1)] = -v[2] * v[1] / (l + v[0]);
                    res[(2, 0)] = -v[2] * v[1] / (l + v[0]);
                    res[(2, 1)] = l - v[2] * v[2] / (l + v[0]);
                    res /= l;
                } else {
                    res[(1, 1)] = -1.0;
                    res[(2, 0)] = 1.0;
                }
            }
        }
        res
    }

    /// Skew matrix of the embedded vector (`S2_hat`).
    pub fn s2_hat(&self) -> M3D {
        skew(&self.vec)
    }

    /// `S2_Nx_yy`: differential of the projection, `Bxᵀ·hat(vec)/length²`.
    pub fn s2_nx_yy(&self) -> Matrix2x3<f64> {
        self.s2_bx().transpose() * self.s2_hat() / (self.length * self.length)
    }

    /// `S2_Mx`: differential of `oplus` w.r.t. the tangent delta.
    pub fn s2_mx(&self, delta: &Vector2<f64>) -> Matrix3x2<f64> {
        let bx = self.s2_bx();
        let bu = bx * delta;
        if delta.norm() < S2_TOLERANCE {
            -self.s2_hat() * bx
        } else {
            -exp(&bu) * self.s2_hat() * a_matrix(&bu).transpose() * bx
        }
    }

    /// Manifold addition (`boxplus`, 2-DOF delta). `vec = Exp(Bx·delta)·vec`.
    pub fn boxplus(&mut self, delta: &Vector2<f64>) {
        let bx = self.s2_bx();
        let bu = bx * delta;
        self.vec = exp(&bu) * self.vec;
    }

    /// Embedded-space addition (`oplus`, 3-DOF delta): `vec = Exp(Δ·scale)·vec`.
    /// Used by `esekf::predict` (scale = dt).
    pub fn oplus(&mut self, delta: &V3D, scale: f64) {
        self.vec = exp(&(delta * scale)) * self.vec;
    }

    /// Manifold subtraction (`boxminus`), returns a 2-vector.
    // Note: `3.1415926` intentionally matches the C++ `S2.hpp` value (not PI).
    #[allow(clippy::approx_constant)]
    pub fn boxminus(&self, other: &S2) -> Vector2<f64> {
        let v_sin = (self.s2_hat() * other.vec).norm();
        let v_cos = self.vec.dot(&other.vec);
        let theta = v_sin.atan2(v_cos);
        let mut res = Vector2::zeros();
        if v_sin < S2_TOLERANCE {
            if theta.abs() > S2_TOLERANCE {
                res[0] = 3.1415926;
                res[1] = 0.0;
            }
        } else {
            let bx = other.s2_bx();
            res = theta / v_sin * (bx.transpose() * (other.s2_hat() * self.vec));
        }
        res
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const TOL: f64 = 1e-9;

    fn s2() -> S2 {
        // gravity-like instance used by FAST-LIO
        S2::from_vec(&V3D::new(0.1, -0.2, 9.8), 9.809, 1)
    }

    #[test]
    fn norm_is_length() {
        let s = s2();
        assert!((s.vec.norm() - 9.809).abs() < TOL);
    }

    #[test]
    fn boxplus_preserves_norm() {
        let mut s = s2();
        s.boxplus(&Vector2::new(0.3, -0.2));
        assert!((s.vec.norm() - 9.809).abs() < 1e-6);
    }

    #[test]
    fn boxplus_boxminus_roundtrip() {
        let mut s = s2();
        let delta = Vector2::new(0.4, -0.1);
        s.boxplus(&delta);
        let back = s.boxminus(&s2());
        // tangential delta should be recovered approximately
        assert!((back - delta).norm() < 1e-3);
    }

    #[test]
    fn s2_bx_orthogonal_to_vec() {
        let s = s2();
        let bx = s.s2_bx();
        // each column of Bx should be orthogonal to vec (up to length scaling)
        for j in 0..2 {
            let col = bx.column(j);
            assert!(s.vec.dot(&col).abs() < 1e-6);
        }
    }

    #[test]
    fn s2_hat_is_skew() {
        let s = s2();
        assert!((s.s2_hat() + s.s2_hat().transpose()).norm() < TOL);
    }

    #[test]
    fn s2_nx_yy_shape() {
        let s = s2();
        let nx = s.s2_nx_yy();
        assert_eq!(nx.shape(), (2, 3));
        // key identity: Bxᵀ·Bx = I2 (Bx is an orthonormal tangent basis)
        let bx = s.s2_bx();
        let btb = bx.transpose() * bx;
        let expected = nalgebra::Matrix2::<f64>::identity();
        assert!((btb - expected).norm() < 1e-6);
    }

    #[test]
    fn s2_mx_zero_delta() {
        let s = s2();
        let mx = s.s2_mx(&Vector2::zeros());
        let bx = s.s2_bx();
        let expected = -s.s2_hat() * bx;
        assert!((mx - expected).norm() < TOL);
    }
}
