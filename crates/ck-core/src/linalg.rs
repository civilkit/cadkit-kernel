//! Linear algebra: Vec3, quaternion, frames. f64 throughout.
//!
//! Conventions pinned by the oracle + decompilation:
//! - Vec3 = plain f64x3, inline in structs (decomp: plane normal at +0x48
//!   as 3 consecutive f64s — never boxed)
//! - Frame/orientation = origin `o` + basis `x`,`y`,`z` (the `Ot` table:
//!   front/top/right), right-handed, basis vectors orthonormal
//! - Quaternions: (w,x,y,z)? — pinned by f4200_cold (the cos-half-angle
//!   Taylor series 1 - t²/2 used in sin/cos of half angles)

use std::ops::{Add, Div, Mul, Neg, Sub};

#[derive(Debug, Clone, Copy, PartialEq, Default)]
#[repr(C)]
pub struct Vec3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Vec3 {
    pub const ZERO: Vec3 = Vec3 { x: 0.0, y: 0.0, z: 0.0 };
    pub const X: Vec3 = Vec3 { x: 1.0, y: 0.0, z: 0.0 };
    pub const Y: Vec3 = Vec3 { x: 0.0, y: 1.0, z: 0.0 };
    pub const Z: Vec3 = Vec3 { x: 0.0, y: 0.0, z: 1.0 };

    pub fn new(x: f64, y: f64, z: f64) -> Self {
        Vec3 { x, y, z }
    }

    pub fn dot(self, o: Vec3) -> f64 {
        self.x * o.x + self.y * o.y + self.z * o.z
    }

    pub fn cross(self, o: Vec3) -> Vec3 {
        Vec3 {
            x: self.y * o.z - self.z * o.y,
            y: self.z * o.x - self.x * o.z,
            z: self.x * o.y - self.y * o.x,
        }
    }

    pub fn len(self) -> f64 {
        self.dot(self).sqrt()
    }

    pub fn len2(self) -> f64 {
        self.dot(self)
    }

    pub fn dist(self, o: Vec3) -> f64 {
        (self - o).len()
    }

    /// Point-plane signed distance: |n·(p - q)| semantics of kernel
    /// func[912]/func[1000] (they take abs).
    pub fn plane_dist(self, n: Vec3, q: Vec3) -> f64 {
        (n.dot(self - q)).abs()
    }

    /// Unit vector, kernel semantics (inlined at hundreds of sites, e.g.
    /// kernel.wat ~L58871): `l = sqrt(x²+y²+z²)`; if `!(l > 1e-300)` or `l`
    /// is not finite the result is the ZERO vector; otherwise each component
    /// is multiplied by the reciprocal `1/l` (NOT divided — matters for
    /// bit-exactness).
    pub fn normalize(self) -> Vec3 {
        self.try_normalize().unwrap_or(Vec3::ZERO)
    }

    /// `Some(unit)` or `None` for degenerate / non-finite input (same guard
    /// as [`Vec3::normalize`]).
    pub fn try_normalize(self) -> Option<Vec3> {
        let l = self.len();
        if !(l > TINY) || !l.is_finite() {
            return None;
        }
        let inv = 1.0 / l;
        Some(Vec3 { x: self.x * inv, y: self.y * inv, z: self.z * inv })
    }

    pub fn from_array(a: [f64; 3]) -> Vec3 {
        Vec3 { x: a[0], y: a[1], z: a[2] }
    }

    pub fn to_array(self) -> [f64; 3] {
        [self.x, self.y, self.z]
    }

    /// Component-wise min / max (bounding boxes).
    pub fn min(self, o: Vec3) -> Vec3 {
        Vec3::new(self.x.min(o.x), self.y.min(o.y), self.z.min(o.z))
    }

    pub fn max(self, o: Vec3) -> Vec3 {
        Vec3::new(self.x.max(o.x), self.y.max(o.y), self.z.max(o.z))
    }

    /// Component by axis index 0/1/2 (kd-tree splits).
    pub fn get(self, i: usize) -> f64 {
        match i {
            0 => self.x,
            1 => self.y,
            _ => self.z,
        }
    }

    pub fn is_finite(self) -> bool {
        self.x.is_finite() && self.y.is_finite() && self.z.is_finite()
    }

    pub fn dist2(self, o: Vec3) -> f64 {
        (self - o).len2()
    }

    /// Unsigned angle between two vectors in [0, π], robust near 0/π
    /// (atan2 of |a×b| and a·b rather than acos).
    pub fn angle(self, o: Vec3) -> f64 {
        self.cross(o).len().atan2(self.dot(o))
    }

    /// Signed angle from `self` to `o` about axis `n` (right-hand rule).
    pub fn signed_angle(self, o: Vec3, n: Vec3) -> f64 {
        let c = self.cross(o);
        let s = c.len();
        let a = s.atan2(self.dot(o));
        if c.dot(n) < 0.0 {
            -a
        } else {
            a
        }
    }

    /// Component of `self` orthogonal to unit `n`: `v - n (n·v)`.
    pub fn reject(self, n: Vec3) -> Vec3 {
        self - n.scale(n.dot(self))
    }

    /// Projection of `self` onto unit `n`.
    pub fn project(self, n: Vec3) -> Vec3 {
        n.scale(n.dot(self))
    }

    pub fn is_parallel(self, o: Vec3, tol: f64) -> bool {
        self.cross(o).len() <= tol * self.len() * o.len()
    }

    /// Some unit vector perpendicular to unit `n`. Mirrors the frame-
    /// building idiom at kernel.wat ~L1387164: reference axis
    /// `a = X if |n.x| < 0.9 else Y`, result `normalize(n × a)`.
    pub fn any_perp(self) -> Vec3 {
        let a = if self.x.abs() < 0.9 { Vec3::X } else { Vec3::Y };
        self.cross(a).normalize()
    }

    pub fn scale(self, s: f64) -> Vec3 {
        Vec3 { x: self.x * s, y: self.y * s, z: self.z * s }
    }

    pub fn lerp(self, o: Vec3, t: f64) -> Vec3 {
        self + (o - self).scale(t)
    }

    pub fn abs(self) -> Vec3 {
        Vec3 { x: self.x.abs(), y: self.y.abs(), z: self.z.abs() }
    }
}

impl Add for Vec3 {
    type Output = Vec3;
    fn add(self, o: Vec3) -> Vec3 {
        Vec3 { x: self.x + o.x, y: self.y + o.y, z: self.z + o.z }
    }
}
impl Sub for Vec3 {
    type Output = Vec3;
    fn sub(self, o: Vec3) -> Vec3 {
        Vec3 { x: self.x - o.x, y: self.y - o.y, z: self.z - o.z }
    }
}
impl Mul<f64> for Vec3 {
    type Output = Vec3;
    fn mul(self, s: f64) -> Vec3 {
        self.scale(s)
    }
}
impl Div<f64> for Vec3 {
    type Output = Vec3;
    fn div(self, s: f64) -> Vec3 {
        Vec3 { x: self.x / s, y: self.y / s, z: self.z / s }
    }
}
impl Neg for Vec3 {
    type Output = Vec3;
    fn neg(self) -> Vec3 {
        Vec3 { x: -self.x, y: -self.y, z: -self.z }
    }
}

/// Tolerance / guard constants. Values are the most frequent `f64.const`
/// literals in kernel.wat (count in parentheses). Their *names* and exact
/// role at each site are GUESSes; the values are pinned.
/// Normalisation guard: `len > 1e-300` (708 sites, always right after
/// `f64.sqrt`, see [`Vec3::normalize`]).
pub const TINY: f64 = 1e-300;
/// 1e-6 (660 sites) — GUESS: general linear (model-space) tolerance.
pub const EPS_LINEAR: f64 = 1e-6;
/// 1e-9 (443 sites) — GUESS: tight point-coincidence tolerance.
pub const EPS_POINT: f64 = 1e-9;
/// 1e-12 (371 sites) — GUESS: parametric / near-zero tolerance.
pub const EPS_PARAM: f64 = 1e-12;
/// 1e-15 (56 sites).
pub const EPS_MACHINE: f64 = 1e-15;
/// 1e-3 (194 sites).
pub const EPS_COARSE: f64 = 1e-3;

/// Degrees → radians (π/180 appears as 0.0174533 at 27 sites).
pub fn deg(d: f64) -> f64 {
    d * (std::f64::consts::PI / 180.0)
}

/// Radians → degrees.
pub fn to_deg(r: f64) -> f64 {
    r * (180.0 / std::f64::consts::PI)
}

/// Wrap an angle into [0, 2π).
pub fn wrap_2pi(a: f64) -> f64 {
    let t = std::f64::consts::TAU;
    let r = a.rem_euclid(t);
    if r >= t {
        0.0
    } else {
        r
    }
}

/// Wrap an angle into (-π, π].
pub fn wrap_pi(a: f64) -> f64 {
    let pi = std::f64::consts::PI;
    let r = wrap_2pi(a);
    if r > pi {
        r - std::f64::consts::TAU
    } else {
        r
    }
}

/// 2D vector (sketch / parameter space).
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Vec2 {
    pub x: f64,
    pub y: f64,
}

impl Vec2 {
    pub const ZERO: Vec2 = Vec2 { x: 0.0, y: 0.0 };
    pub const X: Vec2 = Vec2 { x: 1.0, y: 0.0 };
    pub const Y: Vec2 = Vec2 { x: 0.0, y: 1.0 };

    pub fn new(x: f64, y: f64) -> Self {
        Vec2 { x, y }
    }
    pub fn dot(self, o: Vec2) -> f64 {
        self.x * o.x + self.y * o.y
    }
    /// z-component of the 3D cross product (signed area ×2).
    pub fn cross(self, o: Vec2) -> f64 {
        self.x * o.y - self.y * o.x
    }
    pub fn len(self) -> f64 {
        self.dot(self).sqrt()
    }
    pub fn len2(self) -> f64 {
        self.dot(self)
    }
    pub fn dist(self, o: Vec2) -> f64 {
        (self - o).len()
    }
    pub fn scale(self, s: f64) -> Vec2 {
        Vec2 { x: self.x * s, y: self.y * s }
    }
    /// Same guard as [`Vec3::normalize`] (zero for degenerate input).
    pub fn normalize(self) -> Vec2 {
        let l = self.len();
        if !(l > TINY) || !l.is_finite() {
            return Vec2::ZERO;
        }
        let inv = 1.0 / l;
        Vec2 { x: self.x * inv, y: self.y * inv }
    }
    /// Counter-clockwise perpendicular (-y, x).
    pub fn perp(self) -> Vec2 {
        Vec2 { x: -self.y, y: self.x }
    }
    pub fn lerp(self, o: Vec2, t: f64) -> Vec2 {
        self + (o - self).scale(t)
    }
    pub fn angle(self) -> f64 {
        self.y.atan2(self.x)
    }
    pub fn rotate(self, a: f64) -> Vec2 {
        let (s, c) = a.sin_cos();
        Vec2 { x: c * self.x - s * self.y, y: s * self.x + c * self.y }
    }
    pub fn to_array(self) -> [f64; 2] {
        [self.x, self.y]
    }
    /// Lift into 3D (z = 0).
    pub fn extend(self, z: f64) -> Vec3 {
        Vec3::new(self.x, self.y, z)
    }
}

impl Add for Vec2 {
    type Output = Vec2;
    fn add(self, o: Vec2) -> Vec2 {
        Vec2 { x: self.x + o.x, y: self.y + o.y }
    }
}
impl Sub for Vec2 {
    type Output = Vec2;
    fn sub(self, o: Vec2) -> Vec2 {
        Vec2 { x: self.x - o.x, y: self.y - o.y }
    }
}
impl Mul<f64> for Vec2 {
    type Output = Vec2;
    fn mul(self, s: f64) -> Vec2 {
        self.scale(s)
    }
}
impl Div<f64> for Vec2 {
    type Output = Vec2;
    fn div(self, s: f64) -> Vec2 {
        Vec2 { x: self.x / s, y: self.y / s }
    }
}
impl Neg for Vec2 {
    type Output = Vec2;
    fn neg(self) -> Vec2 {
        Vec2 { x: -self.x, y: -self.y }
    }
}

/// 3×3 matrix stored as COLUMNS `c[0..3]` (so a rotation matrix's columns
/// are the rotated X/Y/Z axes — the same convention as [`Frame`] x/y/z).
/// GUESS: column storage chosen to match Frame; no evidence of the kernel's
/// own matrix layout yet.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Mat3 {
    pub c: [Vec3; 3],
}

impl Mat3 {
    pub const IDENTITY: Mat3 = Mat3 { c: [Vec3::X, Vec3::Y, Vec3::Z] };
    pub const ZERO: Mat3 = Mat3 { c: [Vec3::ZERO, Vec3::ZERO, Vec3::ZERO] };

    pub fn from_cols(a: Vec3, b: Vec3, c: Vec3) -> Mat3 {
        Mat3 { c: [a, b, c] }
    }
    pub fn from_rows(a: Vec3, b: Vec3, c: Vec3) -> Mat3 {
        Mat3::from_cols(a, b, c).transpose()
    }
    pub fn row(&self, i: usize) -> Vec3 {
        Vec3::new(self.c[0].get(i), self.c[1].get(i), self.c[2].get(i))
    }
    pub fn transpose(&self) -> Mat3 {
        Mat3 { c: [self.row(0), self.row(1), self.row(2)] }
    }
    /// M · v
    pub fn mul_vec(&self, v: Vec3) -> Vec3 {
        self.c[0].scale(v.x) + self.c[1].scale(v.y) + self.c[2].scale(v.z)
    }
    /// Mᵀ · v
    pub fn tmul_vec(&self, v: Vec3) -> Vec3 {
        Vec3::new(self.c[0].dot(v), self.c[1].dot(v), self.c[2].dot(v))
    }
    pub fn mul_mat(&self, o: &Mat3) -> Mat3 {
        Mat3 { c: [self.mul_vec(o.c[0]), self.mul_vec(o.c[1]), self.mul_vec(o.c[2])] }
    }
    pub fn det(&self) -> f64 {
        self.c[0].dot(self.c[1].cross(self.c[2]))
    }
    /// Inverse via adjugate; `None` if |det| <= TINY.
    pub fn inverse(&self) -> Option<Mat3> {
        let d = self.det();
        if !(d.abs() > TINY) || !d.is_finite() {
            return None;
        }
        let inv = 1.0 / d;
        let (a, b, c) = (self.c[0], self.c[1], self.c[2]);
        // rows of the inverse are (b×c, c×a, a×b)/det
        Some(Mat3::from_rows(b.cross(c).scale(inv), c.cross(a).scale(inv), a.cross(b).scale(inv)))
    }
    pub fn scale(&self, s: f64) -> Mat3 {
        Mat3 { c: [self.c[0].scale(s), self.c[1].scale(s), self.c[2].scale(s)] }
    }
    /// Outer product a bᵀ.
    pub fn outer(a: Vec3, b: Vec3) -> Mat3 {
        Mat3 { c: [a.scale(b.x), a.scale(b.y), a.scale(b.z)] }
    }
    /// Rotation by `angle` about unit `axis` (Rodrigues).
    pub fn rotation(axis: Vec3, angle: f64) -> Mat3 {
        let (s, c) = angle.sin_cos();
        let t = 1.0 - c;
        let (x, y, z) = (axis.x, axis.y, axis.z);
        Mat3::from_rows(
            Vec3::new(t * x * x + c, t * x * y - s * z, t * x * z + s * y),
            Vec3::new(t * x * y + s * z, t * y * y + c, t * y * z - s * x),
            Vec3::new(t * x * z - s * y, t * y * z + s * x, t * z * z + c),
        )
    }
    /// Gram–Schmidt re-orthonormalisation of the columns (x kept, y made
    /// orthogonal to x, z = x × y). Returns a proper rotation.
    pub fn orthonormalize(&self) -> Mat3 {
        let (x, y, _) = orthonormalize(self.c[0], self.c[1]);
        Mat3::from_cols(x, y, x.cross(y))
    }
    /// Rotation → unit quaternion (Shepperd's method).
    pub fn to_quat(&self) -> Quat {
        let m = |r: usize, c: usize| self.c[c].get(r);
        let tr = m(0, 0) + m(1, 1) + m(2, 2);
        let q = if tr > 0.0 {
            let s = (tr + 1.0).sqrt() * 2.0;
            Quat { w: 0.25 * s, x: (m(2, 1) - m(1, 2)) / s, y: (m(0, 2) - m(2, 0)) / s, z: (m(1, 0) - m(0, 1)) / s }
        } else if m(0, 0) > m(1, 1) && m(0, 0) > m(2, 2) {
            let s = (1.0 + m(0, 0) - m(1, 1) - m(2, 2)).sqrt() * 2.0;
            Quat { w: (m(2, 1) - m(1, 2)) / s, x: 0.25 * s, y: (m(0, 1) + m(1, 0)) / s, z: (m(0, 2) + m(2, 0)) / s }
        } else if m(1, 1) > m(2, 2) {
            let s = (1.0 + m(1, 1) - m(0, 0) - m(2, 2)).sqrt() * 2.0;
            Quat { w: (m(0, 2) - m(2, 0)) / s, x: (m(0, 1) + m(1, 0)) / s, y: 0.25 * s, z: (m(1, 2) + m(2, 1)) / s }
        } else {
            let s = (1.0 + m(2, 2) - m(0, 0) - m(1, 1)).sqrt() * 2.0;
            Quat { w: (m(1, 0) - m(0, 1)) / s, x: (m(0, 2) + m(2, 0)) / s, y: (m(1, 2) + m(2, 1)) / s, z: 0.25 * s }
        };
        q.normalize()
    }
}

impl Mul for Mat3 {
    type Output = Mat3;
    fn mul(self, o: Mat3) -> Mat3 {
        self.mul_mat(&o)
    }
}
impl Mul<Vec3> for Mat3 {
    type Output = Vec3;
    fn mul(self, v: Vec3) -> Vec3 {
        self.mul_vec(v)
    }
}

/// 4×4 homogeneous matrix, column-major `m[col][row]` (GUESS: layout for
/// interop only — e.g. WebGL-style column-major; kernel layout unknown).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Mat4 {
    pub m: [[f64; 4]; 4],
}

impl Mat4 {
    pub const IDENTITY: Mat4 = Mat4 {
        m: [[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]],
    };
    /// Affine matrix from linear part + translation.
    pub fn from_affine(r: &Mat3, t: Vec3) -> Mat4 {
        let c = |v: Vec3, w: f64| [v.x, v.y, v.z, w];
        Mat4 { m: [c(r.c[0], 0.0), c(r.c[1], 0.0), c(r.c[2], 0.0), c(t, 1.0)] }
    }
    pub fn mul_mat(&self, o: &Mat4) -> Mat4 {
        let mut r = [[0.0; 4]; 4];
        for (c, rc) in r.iter_mut().enumerate() {
            for (row, v) in rc.iter_mut().enumerate() {
                *v = (0..4).map(|k| self.m[k][row] * o.m[c][k]).sum();
            }
        }
        Mat4 { m: r }
    }
    /// Transform a point (w = 1, with perspective divide when w' != 1).
    pub fn apply_point(&self, p: Vec3) -> Vec3 {
        let v = [p.x, p.y, p.z, 1.0];
        let mut r = [0.0; 4];
        for (row, rv) in r.iter_mut().enumerate() {
            *rv = (0..4).map(|k| self.m[k][row] * v[k]).sum();
        }
        if r[3] != 1.0 && r[3] != 0.0 {
            Vec3::new(r[0] / r[3], r[1] / r[3], r[2] / r[3])
        } else {
            Vec3::new(r[0], r[1], r[2])
        }
    }
    /// Transform a direction (w = 0).
    pub fn apply_vec(&self, v: Vec3) -> Vec3 {
        let a = [v.x, v.y, v.z];
        let f = |row: usize| (0..3).map(|k| self.m[k][row] * a[k]).sum::<f64>();
        Vec3::new(f(0), f(1), f(2))
    }
    pub fn transpose(&self) -> Mat4 {
        let mut r = [[0.0; 4]; 4];
        for (c, rc) in r.iter_mut().enumerate() {
            for (row, v) in rc.iter_mut().enumerate() {
                *v = self.m[row][c];
            }
        }
        Mat4 { m: r }
    }
}

/// Gram–Schmidt: from a primary direction `a` and a secondary hint `b`
/// return a right-handed orthonormal triple `(x, y, z)` with `x ∥ a`,
/// `y` in the (a, b) plane, `z = x × y`. If `b` is (nearly) parallel to `a`
/// a perpendicular is chosen with [`Vec3::any_perp`].
pub fn orthonormalize(a: Vec3, b: Vec3) -> (Vec3, Vec3, Vec3) {
    let x = a.normalize();
    let y = match b.reject(x).try_normalize() {
        Some(y) if b.reject(x).len() > EPS_PARAM * b.len().max(1.0) => y,
        _ => x.any_perp(),
    };
    let z = x.cross(y);
    (x, y, z)
}

/// Infinite plane: point + unit normal.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Plane {
    pub o: Vec3,
    pub n: Vec3,
}

impl Plane {
    /// Normal is normalised (zero if degenerate).
    pub fn new(o: Vec3, n: Vec3) -> Plane {
        Plane { o, n: n.normalize() }
    }
    /// Plane through three points (normal = (b-a)×(c-a)); `None` if
    /// collinear.
    pub fn from_points(a: Vec3, b: Vec3, c: Vec3) -> Option<Plane> {
        (b - a).cross(c - a).try_normalize().map(|n| Plane { o: a, n })
    }
    /// Signed distance n·(p − o).
    pub fn signed_dist(&self, p: Vec3) -> f64 {
        self.n.dot(p - self.o)
    }
    /// |n·(p − o)| — kernel func[912] / func[1000] (verified by the
    /// differential test tests/ffi_diff.rs).
    pub fn dist(&self, p: Vec3) -> f64 {
        p.plane_dist(self.n, self.o)
    }
    pub fn project(&self, p: Vec3) -> Vec3 {
        p - self.n.scale(self.signed_dist(p))
    }
    /// Line `p + t d` intersection parameter; `None` if parallel.
    pub fn intersect_line(&self, p: Vec3, d: Vec3) -> Option<f64> {
        let den = self.n.dot(d);
        if den.abs() <= TINY {
            return None;
        }
        Some(self.n.dot(self.o - p) / den)
    }
    /// Intersection line of two planes: (point, unit direction).
    pub fn intersect_plane(&self, o: &Plane) -> Option<(Vec3, Vec3)> {
        let d = self.n.cross(o.n);
        let l2 = d.len2();
        if l2 <= EPS_PARAM * EPS_PARAM {
            return None;
        }
        let h1 = self.n.dot(self.o);
        let h2 = o.n.dot(o.o);
        // point = (h1 (n2 × d) + h2 (d × n1)) / |d|²  (d = n1 × n2)
        let p = (o.n.cross(d).scale(h1) + d.cross(self.n).scale(h2)).scale(1.0 / l2);
        Some((p, d.normalize()))
    }
    pub fn mirror_point(&self, p: Vec3) -> Vec3 {
        p - self.n.scale(2.0 * self.signed_dist(p))
    }
    pub fn mirror_vec(&self, v: Vec3) -> Vec3 {
        v - self.n.scale(2.0 * self.n.dot(v))
    }
    pub fn flipped(&self) -> Plane {
        Plane { o: self.o, n: -self.n }
    }
}

/// Orientation frame: origin + orthonormal basis (the `Ot` std-plane table).
///
/// Memory layout pinned by kernel func[912]: a frame-like plane struct is
/// `{o @+0, x @+0x18, y @+0x30, z @+0x48}`, each an inline f64×3; the
/// point-plane distance reads the normal from `+0x48` (= `z`).
#[derive(Debug, Clone, Copy, PartialEq)]
#[repr(C)]
pub struct Frame {
    pub o: Vec3,
    pub x: Vec3,
    pub y: Vec3,
    pub z: Vec3,
}

impl Frame {
    pub fn identity() -> Self {
        Frame { o: Vec3::ZERO, x: Vec3::X, y: Vec3::Y, z: Vec3::Z }
    }

    /// To world: p_world = o + x*px + y*py + z*pz
    pub fn to_world(self, p: Vec3) -> Vec3 {
        self.o + self.x.scale(p.x) + self.y.scale(p.y) + self.z.scale(p.z)
    }

    /// To frame-local: p_local = [(p-o)·x, (p-o)·y, (p-o)·z]
    pub fn to_local(self, p: Vec3) -> Vec3 {
        let d = p - self.o;
        Vec3::new(d.dot(self.x), d.dot(self.y), d.dot(self.z))
    }

    /// Direction to world: x*vx + y*vy + z*vz (no origin).
    pub fn vec_to_world(self, v: Vec3) -> Vec3 {
        self.x.scale(v.x) + self.y.scale(v.y) + self.z.scale(v.z)
    }

    /// Direction to frame-local.
    pub fn vec_to_local(self, v: Vec3) -> Vec3 {
        Vec3::new(v.dot(self.x), v.dot(self.y), v.dot(self.z))
    }

    /// Frame-local 2D sketch point (u, v) → world.
    pub fn uv_to_world(self, p: Vec2) -> Vec3 {
        self.o + self.x.scale(p.x) + self.y.scale(p.y)
    }

    /// World point → sketch (u, v) (drops the z coordinate).
    pub fn world_to_uv(self, p: Vec3) -> Vec2 {
        let d = p - self.o;
        Vec2::new(d.dot(self.x), d.dot(self.y))
    }

    /// Frame from origin + normal (z); x from [`Vec3::any_perp`]
    /// (`a = X if |n.x| < 0.9 else Y`, x = normalize(n × a), the idiom at
    /// kernel.wat ~L1387164), y = z × x.
    pub fn from_normal(o: Vec3, n: Vec3) -> Frame {
        let z = n.normalize();
        let x = z.any_perp();
        Frame { o, x, y: z.cross(x), z }
    }

    /// Frame from origin, normal (z) and an x-direction hint (projected
    /// into the plane). Falls back to [`Frame::from_normal`] when the hint
    /// is parallel to the normal.
    pub fn from_normal_x(o: Vec3, n: Vec3, xhint: Vec3) -> Frame {
        let z = n.normalize();
        match xhint.reject(z).try_normalize() {
            Some(x) => Frame { o, x, y: z.cross(x), z },
            None => Frame::from_normal(o, z),
        }
    }

    /// Frame from origin + x + y hint (Gram–Schmidt, z = x × y).
    pub fn from_xy(o: Vec3, x: Vec3, y: Vec3) -> Frame {
        let (x, y, z) = orthonormalize(x, y);
        Frame { o, x, y, z }
    }

    /// Re-orthonormalise the basis (x kept, y fixed up, z = x × y).
    pub fn orthonormalize(self) -> Frame {
        Frame::from_xy(self.o, self.x, self.y)
    }

    /// Rotation part as a column matrix (cols = x, y, z).
    pub fn mat3(self) -> Mat3 {
        Mat3::from_cols(self.x, self.y, self.z)
    }

    /// The frame's xy-plane.
    pub fn plane(self) -> Plane {
        Plane { o: self.o, n: self.z }
    }

    /// Wire form `{o,x,y,z}` as arrays (components[].place in the regen
    /// reply, oracle-flange.json).
    pub fn to_arrays(self) -> [[f64; 3]; 4] {
        [self.o.to_array(), self.x.to_array(), self.y.to_array(), self.z.to_array()]
    }

    pub fn from_arrays(a: [[f64; 3]; 4]) -> Frame {
        Frame {
            o: Vec3::from_array(a[0]),
            x: Vec3::from_array(a[1]),
            y: Vec3::from_array(a[2]),
            z: Vec3::from_array(a[3]),
        }
    }

    /// Max deviation from orthonormality (|xx−1|, |x·y|, ...,
    /// |(x×y)−z|); 0 for a perfect right-handed frame.
    pub fn ortho_error(self) -> f64 {
        let e = [
            (self.x.len2() - 1.0).abs(),
            (self.y.len2() - 1.0).abs(),
            (self.z.len2() - 1.0).abs(),
            self.x.dot(self.y).abs(),
            self.y.dot(self.z).abs(),
            self.z.dot(self.x).abs(),
            (self.x.cross(self.y) - self.z).len(),
        ];
        e.iter().cloned().fold(0.0, f64::max)
    }
}

/// Standard planes, matching the `Ot` orientation table in app.pretty.js:
/// front: x=[1,0,0] y=[0,1,0] z=[0,0,1]; top: x=[1,0,0] y=[0,0,-1] z=[0,1,0];
/// right: x=[0,0,-1] y=[0,1,0] z=[1,0,0].
pub fn std_plane(kind: &str) -> Frame {
    match kind {
        "front" => Frame::identity(),
        "top" => Frame { o: Vec3::ZERO, x: Vec3::X, y: Vec3::new(0.0, 0.0, -1.0), z: Vec3::Y },
        "right" => Frame { o: Vec3::ZERO, x: Vec3::new(0.0, 0.0, -1.0), y: Vec3::Y, z: Vec3::X },
        _ => Frame::identity(),
    }
}

/// Quaternion (w, x, y, z).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Quat {
    pub w: f64,
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Quat {
    pub fn identity() -> Self {
        Quat { w: 1.0, x: 0.0, y: 0.0, z: 0.0 }
    }

    /// Rotation about unit axis by angle (radians).
    pub fn axis_angle(axis: Vec3, angle: f64) -> Self {
        let h = angle * 0.5;
        let s = h.sin();
        Quat { w: h.cos(), x: axis.x * s, y: axis.y * s, z: axis.z * s }
    }

    pub fn rotate(self, v: Vec3) -> Vec3 {
        // Rodrigues via quaternion products: v + 2*qv×(qv×v) + 2*w*(qv×v)
        let qv = Vec3::new(self.x, self.y, self.z);
        let t = qv.cross(v);
        v + t.scale(2.0 * self.w) + qv.cross(t).scale(2.0)
    }

    pub fn len(self) -> f64 {
        (self.w * self.w + self.x * self.x + self.y * self.y + self.z * self.z).sqrt()
    }

    pub fn normalize(self) -> Quat {
        let l = self.len();
        if !(l > TINY) || !l.is_finite() {
            return Quat::identity();
        }
        let inv = 1.0 / l;
        Quat { w: self.w * inv, x: self.x * inv, y: self.y * inv, z: self.z * inv }
    }

    pub fn conj(self) -> Quat {
        Quat { w: self.w, x: -self.x, y: -self.y, z: -self.z }
    }

    pub fn dot(self, o: Quat) -> f64 {
        self.w * o.w + self.x * o.x + self.y * o.y + self.z * o.z
    }

    /// Spherical interpolation (shortest arc).
    pub fn slerp(self, o: Quat, t: f64) -> Quat {
        let mut d = self.dot(o);
        let mut o = o;
        if d < 0.0 {
            d = -d;
            o = Quat { w: -o.w, x: -o.x, y: -o.y, z: -o.z };
        }
        if d > 1.0 - EPS_PARAM {
            let l = |a: f64, b: f64| a + (b - a) * t;
            return Quat { w: l(self.w, o.w), x: l(self.x, o.x), y: l(self.y, o.y), z: l(self.z, o.z) }.normalize();
        }
        let th = d.acos();
        let s = th.sin();
        let (a, b) = (((1.0 - t) * th).sin() / s, (t * th).sin() / s);
        Quat { w: a * self.w + b * o.w, x: a * self.x + b * o.x, y: a * self.y + b * o.y, z: a * self.z + b * o.z }
    }

    /// Rotation as a [`Mat3`] (cols = images of X, Y, Z).
    pub fn to_mat3(self) -> Mat3 {
        let m = self.mat3();
        Mat3::from_cols(m[0], m[1], m[2])
    }

    /// `[R·X, R·Y, R·Z]` — the rotation's columns (see [`Quat::to_mat3`]).
    pub fn mat3(self) -> [Vec3; 3] {
        let (w, x, y, z) = (self.w, self.x, self.y, self.z);
        [
            Vec3::new(1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y + z * w), 2.0 * (x * z - y * w)),
            Vec3::new(2.0 * (x * y - z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z + x * w)),
            Vec3::new(2.0 * (x * z + y * w), 2.0 * (y * z - x * w), 1.0 - 2.0 * (x * x + y * y)),
        ]
    }
}

impl Mul for Quat {
    type Output = Quat;
    fn mul(self, o: Quat) -> Quat {
        Quat {
            w: self.w * o.w - self.x * o.x - self.y * o.y - self.z * o.z,
            x: self.w * o.x + self.x * o.w + self.y * o.z - self.z * o.y,
            y: self.w * o.y - self.x * o.z + self.y * o.w + self.z * o.x,
            z: self.w * o.z + self.x * o.y - self.y * o.x + self.z * o.w,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn close(a: f64, b: f64) -> bool {
        (a - b).abs() < 1e-12
    }

    #[test]
    fn vec_basics() {
        let a = Vec3::new(1.0, 2.0, 2.0);
        assert!(close(a.len(), 3.0));
        assert!(close(a.normalize().len(), 1.0));
        assert!(close(a.dot(Vec3::X), 1.0));
        assert!(Vec3::X.cross(Vec3::Y) == Vec3::Z);
    }

    #[test]
    fn std_planes_right_handed() {
        for k in ["front", "top", "right"] {
            let f = std_plane(k);
            assert!(close(f.x.cross(f.y).dot(f.z), 1.0), "{k} not right-handed");
            assert!(close(f.x.len(), 1.0) && close(f.y.len(), 1.0) && close(f.z.len(), 1.0));
        }
    }

    #[test]
    fn quat_rotate_axis() {
        let q = Quat::axis_angle(Vec3::Z, std::f64::consts::FRAC_PI_2);
        let r = q.rotate(Vec3::X);
        assert!(close(r.x, 0.0) && close(r.y, 1.0) && close(r.z, 0.0));
    }

    fn vclose(a: Vec3, b: Vec3) -> bool {
        (a - b).len() < 1e-12
    }

    #[test]
    fn normalize_kernel_semantics() {
        assert_eq!(Vec3::ZERO.normalize(), Vec3::ZERO);
        assert_eq!(Vec3::new(1e-301, 0.0, 0.0).normalize(), Vec3::ZERO);
        assert_eq!(Vec3::new(f64::INFINITY, 0.0, 0.0).normalize(), Vec3::ZERO);
        assert_eq!(Vec3::new(f64::NAN, 0.0, 0.0).normalize(), Vec3::ZERO);
        // reciprocal multiply, bit-exact
        let v = Vec3::new(3.0, 1.0, 7.0);
        let inv = 1.0 / v.len();
        assert_eq!(v.normalize(), Vec3::new(3.0 * inv, 1.0 * inv, 7.0 * inv));
    }

    #[test]
    fn quat_matrix_agrees_with_rotate() {
        let q = Quat::axis_angle(Vec3::new(1.0, 2.0, -0.5).normalize(), 0.7);
        let m = q.to_mat3();
        for v in [Vec3::X, Vec3::Y, Vec3::Z, Vec3::new(0.3, -2.0, 5.0)] {
            assert!(vclose(m * v, q.rotate(v)));
        }
        let q2 = m.to_quat();
        assert!((q2.dot(q).abs() - 1.0).abs() < 1e-12);
        assert!(vclose(Mat3::rotation(Vec3::new(1.0, 2.0, -0.5).normalize(), 0.7) * Vec3::Y, q.rotate(Vec3::Y)));
    }

    #[test]
    fn mat3_inverse() {
        let m = Mat3::from_cols(Vec3::new(2.0, 0.0, 1.0), Vec3::new(0.0, 3.0, 0.0), Vec3::new(1.0, 1.0, 4.0));
        let i = m.inverse().unwrap();
        let p = m * i;
        for k in 0..3 {
            assert!(vclose(p.c[k], Mat3::IDENTITY.c[k]));
        }
        assert!(Mat3::ZERO.inverse().is_none());
    }

    #[test]
    fn mat4_affine() {
        let r = Mat3::rotation(Vec3::Z, std::f64::consts::FRAC_PI_2);
        let m = Mat4::from_affine(&r, Vec3::new(1.0, 2.0, 3.0));
        assert!(vclose(m.apply_point(Vec3::X), Vec3::new(1.0, 3.0, 3.0)));
        assert!(vclose(m.apply_vec(Vec3::X), Vec3::Y));
        let mm = m.mul_mat(&Mat4::IDENTITY);
        assert_eq!(mm, m);
    }

    #[test]
    fn frames() {
        for n in [Vec3::X, Vec3::Y, Vec3::Z, Vec3::new(0.95, 0.1, 0.2), Vec3::new(-1.0, 1.0, 1.0)] {
            let f = Frame::from_normal(Vec3::new(1.0, 2.0, 3.0), n);
            assert!(f.ortho_error() < 1e-14, "{n:?}");
            assert!(vclose(f.z, n.normalize()));
        }
        let f = Frame::from_normal_x(Vec3::ZERO, Vec3::Z, Vec3::new(1.0, 1.0, 5.0));
        assert!(vclose(f.x, Vec3::new(1.0, 1.0, 0.0).normalize()));
        let f = Frame::from_xy(Vec3::ZERO, Vec3::new(2.0, 0.0, 0.0), Vec3::new(1.0, 1.0, 0.0));
        assert_eq!(f.y, Vec3::Y);
        assert_eq!(f.z, Vec3::Z);
        let p = Vec3::new(4.0, -1.0, 2.0);
        let g = std_plane("top");
        assert!(vclose(g.to_world(g.to_local(p)), p));
        assert!(vclose(g.uv_to_world(g.world_to_uv(p)), g.plane().project(p)));
    }

    #[test]
    fn planes() {
        let a = Plane::new(Vec3::new(0.0, 0.0, 2.0), Vec3::new(0.0, 0.0, 5.0));
        assert_eq!(a.signed_dist(Vec3::new(1.0, 1.0, -1.0)), -3.0);
        assert_eq!(a.dist(Vec3::new(1.0, 1.0, -1.0)), 3.0);
        assert!(vclose(a.mirror_point(Vec3::new(1.0, 1.0, 3.0)), Vec3::new(1.0, 1.0, 1.0)));
        let t = a.intersect_line(Vec3::ZERO, Vec3::new(0.0, 0.0, 4.0)).unwrap();
        assert_eq!(t, 0.5);
        let b = Plane::new(Vec3::new(1.0, 0.0, 0.0), Vec3::X);
        let (p, d) = a.intersect_plane(&b).unwrap();
        assert!(a.dist(p) < 1e-12 && b.dist(p) < 1e-12);
        assert!(d.is_parallel(Vec3::Y, 1e-12));
        assert!(Plane::from_points(Vec3::ZERO, Vec3::X, Vec3::X * 2.0).is_none());
        let c = Plane::from_points(Vec3::ZERO, Vec3::X, Vec3::Y).unwrap();
        assert_eq!(c.n, Vec3::Z);
    }

    #[test]
    fn angles() {
        assert!(close(Vec3::X.angle(Vec3::Y), std::f64::consts::FRAC_PI_2));
        assert!(close(Vec3::X.signed_angle(Vec3::Y, -Vec3::Z), -std::f64::consts::FRAC_PI_2));
        assert!(close(wrap_pi(3.0 * std::f64::consts::PI), std::f64::consts::PI));
        assert!(close(wrap_2pi(-0.5), std::f64::consts::TAU - 0.5));
        assert!(close(to_deg(deg(37.0)), 37.0));
        assert!(close(Vec2::new(1.0, 0.0).rotate(std::f64::consts::FRAC_PI_2).y, 1.0));
        assert_eq!(Vec2::new(1.0, 0.0).cross(Vec2::Y), 1.0);
        let q = Quat::axis_angle(Vec3::Z, 1.0);
        let h = Quat::identity().slerp(q, 0.5);
        assert!((h.dot(Quat::axis_angle(Vec3::Z, 0.5)) - 1.0).abs() < 1e-12);
    }
}
