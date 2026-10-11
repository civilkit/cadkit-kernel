//! Affine / rigid transforms applied to geometry (ck-core/src/xform.rs).
//!
//! `Xform` maps `p ↦ r·p + t` with `r` a [`Mat3`] (columns = images of the
//! X/Y/Z axes). A [`Frame`] `{o,x,y,z}` *is* a rigid transform from frame-
//! local to world coordinates: `r = [x y z]`, `t = o` — this is how
//! `components[].place` (`{"o":[..],"x":[..],"y":[..],"z":[..]}` in the
//! regen reply, oracle-flange.json) is interpreted: the component's local
//! geometry is mapped into the assembly by its place frame.
//!
//! GUESS: the kernel's in-memory transform layout is unknown; only the frame
//! wire form is pinned. Composition order here: `a * b` applies `b` first.

use crate::linalg::{Frame, Mat3, Plane, Quat, Vec3, TINY};
use std::ops::Mul;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Xform {
    /// Linear part (columns = images of X, Y, Z).
    pub r: Mat3,
    /// Translation.
    pub t: Vec3,
}

impl Default for Xform {
    fn default() -> Self {
        Xform::IDENTITY
    }
}

impl Xform {
    pub const IDENTITY: Xform = Xform { r: Mat3::IDENTITY, t: Vec3::ZERO };

    pub fn new(r: Mat3, t: Vec3) -> Xform {
        Xform { r, t }
    }

    pub fn translation(t: Vec3) -> Xform {
        Xform { r: Mat3::IDENTITY, t }
    }

    /// Rotation by `angle` (radians, right-hand rule) about the axis through
    /// `p` with unit direction `axis`.
    pub fn rotation(p: Vec3, axis: Vec3, angle: f64) -> Xform {
        let r = Mat3::rotation(axis.normalize(), angle);
        Xform { r, t: p - r.mul_vec(p) }
    }

    pub fn from_quat(q: Quat, t: Vec3) -> Xform {
        Xform { r: q.to_mat3(), t }
    }

    /// Uniform scale about point `c`.
    pub fn scale_about(c: Vec3, s: f64) -> Xform {
        let r = Mat3::IDENTITY.scale(s);
        Xform { r, t: c - c.scale(s) }
    }

    /// Reflection in a plane (det = −1).
    pub fn mirror(pl: &Plane) -> Xform {
        let n = pl.n;
        let r = Mat3 { c: [pl.mirror_vec(Vec3::X), pl.mirror_vec(Vec3::Y), pl.mirror_vec(Vec3::Z)] };
        Xform { r, t: n.scale(2.0 * n.dot(pl.o)) }
    }

    /// Local → world transform of a frame (`r = [x y z]`, `t = o`).
    pub fn from_frame(f: &Frame) -> Xform {
        Xform { r: Mat3::from_cols(f.x, f.y, f.z), t: f.o }
    }

    /// World → local transform of a frame (inverse of [`Xform::from_frame`],
    /// assuming an orthonormal basis).
    pub fn to_frame_local(f: &Frame) -> Xform {
        Xform::from_frame(f).inverse_rigid()
    }

    /// Image of the world frame: `{o: t, x: r·X, y: r·Y, z: r·Z}`.
    pub fn to_frame(&self) -> Frame {
        Frame { o: self.t, x: self.r.c[0], y: self.r.c[1], z: self.r.c[2] }
    }

    /// Change of basis taking frame `a` onto frame `b`
    /// (`from_frame(b) * from_frame(a)⁻¹`).
    pub fn frame_to_frame(a: &Frame, b: &Frame) -> Xform {
        Xform::from_frame(b) * Xform::to_frame_local(a)
    }

    /// `self ∘ o` (apply `o` first).
    pub fn compose(&self, o: &Xform) -> Xform {
        Xform { r: self.r.mul_mat(&o.r), t: self.r.mul_vec(o.t) + self.t }
    }

    /// Inverse for rigid (orthonormal `r`): `(rᵀ, −rᵀ t)`. Exact and cheap.
    pub fn inverse_rigid(&self) -> Xform {
        let rt = self.r.transpose();
        Xform { r: rt, t: -rt.mul_vec(self.t) }
    }

    /// General affine inverse; `None` when singular.
    pub fn inverse(&self) -> Option<Xform> {
        let ri = self.r.inverse()?;
        Some(Xform { r: ri, t: -ri.mul_vec(self.t) })
    }

    /// `t + c0·px + c1·py + c2·pz`, evaluated in the same order as
    /// [`Frame::to_world`] so the two agree bit-for-bit.
    pub fn apply_point(&self, p: Vec3) -> Vec3 {
        self.t + self.r.c[0].scale(p.x) + self.r.c[1].scale(p.y) + self.r.c[2].scale(p.z)
    }

    pub fn apply_vec(&self, v: Vec3) -> Vec3 {
        self.r.mul_vec(v)
    }

    /// Normals transform with the inverse transpose (renormalised). For a
    /// rigid transform this equals [`Xform::apply_vec`].
    pub fn apply_normal(&self, n: Vec3) -> Vec3 {
        match self.r.inverse() {
            Some(ri) => ri.transpose().mul_vec(n).normalize(),
            None => Vec3::ZERO,
        }
    }

    /// Transform a frame: origin as a point, axes as vectors. For a mirror
    /// (det < 0) the result would be left-handed, so z is recomputed as
    /// x × y to keep frames right-handed — GUESS (mirror feature behaviour).
    pub fn apply_frame(&self, f: &Frame) -> Frame {
        let x = self.apply_vec(f.x);
        let y = self.apply_vec(f.y);
        let z = if self.is_mirror() { x.cross(y) } else { self.apply_vec(f.z) };
        Frame { o: self.apply_point(f.o), x, y, z }
    }

    pub fn apply_plane(&self, p: &Plane) -> Plane {
        Plane { o: self.apply_point(p.o), n: self.apply_normal(p.n) }
    }

    pub fn det(&self) -> f64 {
        self.r.det()
    }

    pub fn is_mirror(&self) -> bool {
        self.det() < 0.0
    }

    pub fn is_identity(&self, tol: f64) -> bool {
        (0..3).all(|k| (self.r.c[k] - Mat3::IDENTITY.c[k]).len() <= tol) && self.t.len() <= tol
    }

    /// True when `r` is orthonormal within `tol` (rotation or reflection).
    pub fn is_rigid(&self, tol: f64) -> bool {
        let p = self.r.transpose().mul_mat(&self.r);
        (0..3).all(|k| (p.c[k] - Mat3::IDENTITY.c[k]).len() <= tol) && (self.det().abs() - 1.0).abs() <= tol
    }

    /// Rotation part as a quaternion (`None` for reflections / degenerate).
    pub fn rotation_quat(&self) -> Option<Quat> {
        if !(self.det() > TINY) {
            return None;
        }
        Some(self.r.to_quat())
    }
}

impl Mul for Xform {
    type Output = Xform;
    fn mul(self, o: Xform) -> Xform {
        self.compose(&o)
    }
}

impl Mul<Vec3> for Xform {
    type Output = Vec3;
    /// Point application.
    fn mul(self, p: Vec3) -> Vec3 {
        self.apply_point(p)
    }
}

impl From<Frame> for Xform {
    fn from(f: Frame) -> Xform {
        Xform::from_frame(&f)
    }
}

impl From<Xform> for Frame {
    fn from(x: Xform) -> Frame {
        x.to_frame()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::linalg::std_plane;

    fn vclose(a: Vec3, b: Vec3, tol: f64) -> bool {
        (a - b).len() <= tol
    }

    /// components[].place values from oracle-flange.json (F2, F6, F9), as
    /// printed by the original kernel.
    fn oracle_places() -> Vec<Frame> {
        vec![
            Frame::from_arrays([
                [1.3187749835348084e-15, 1.0533336165618934e-17, 4.08203712346448e-15],
                [-1.0000000000000004, 7.150460731134032e-10, 4.19634530539017e-11],
                [-7.150461542315644e-10, -0.9999999999999999, 8.03706656079415e-17],
                [4.1963480900617005e-11, 6.68882196260781e-17, 1.0],
            ]),
            Frame::from_arrays([
                [12.000000000000002, 0.0, 37.500000000000014],
                [3.175614121497932e-11, 2.1694373094081018e-16, 0.9999999999999999],
                [-2.382439550618696e-11, 1.0, -1.7667670210959259e-16],
                [-0.9999999999999999, -2.382439550618823e-11, 3.1756141214972496e-11],
            ]),
            Frame::from_arrays([
                [-11.999999998426375, -5.717854921110292e-10, 37.50000000076216],
                [-3.515858791293553e-11, 6.344575094375483e-10, -1.0000000000000002],
                [-3.4561087956860205e-10, -1.0, -6.344574680578292e-10],
                [-1.0000000000000002, 3.4561087952763344e-10, 3.515858759970482e-11],
            ]),
        ]
    }

    #[test]
    fn frame_bridge_roundtrip_is_bit_exact() {
        for f in oracle_places() {
            let x = Xform::from_frame(&f);
            assert_eq!(x.to_frame(), f);
            assert_eq!(Frame::from(Xform::from(f)), f);
            assert_eq!(Frame::from_arrays(f.to_arrays()), f);
            // Xform application == Frame::to_world (bit-exact: same op order)
            let p = Vec3::new(3.5, -2.0, 7.25);
            assert_eq!(x.apply_point(p), f.to_world(p));
        }
    }

    #[test]
    fn oracle_places_are_rigid() {
        for f in oracle_places() {
            let x = Xform::from_frame(&f);
            assert!(x.is_rigid(1e-9) && !x.is_mirror());
            assert!(f.ortho_error() < 1e-9);
            let id = x * x.inverse_rigid();
            assert!(id.is_identity(1e-9));
            let p = Vec3::new(1.0, 2.0, 3.0);
            assert!(vclose(Xform::to_frame_local(&f).apply_point(f.to_world(p)), p, 1e-12));
        }
    }

    #[test]
    fn compose_order_and_inverse() {
        let a = Xform::translation(Vec3::new(1.0, 0.0, 0.0));
        let b = Xform::rotation(Vec3::ZERO, Vec3::Z, std::f64::consts::FRAC_PI_2);
        // a*b: rotate first, then translate
        assert!(vclose((a * b) * Vec3::X, Vec3::new(1.0, 1.0, 0.0), 1e-15));
        assert!(vclose((b * a) * Vec3::X, Vec3::new(0.0, 2.0, 0.0), 1e-15));
        let c = a * b * Xform::scale_about(Vec3::new(1.0, 1.0, 1.0), 2.0);
        let ci = c.inverse().unwrap();
        assert!((c * ci).is_identity(1e-14));
        let p = Vec3::new(0.3, 0.4, -5.0);
        assert!(vclose(ci * (c * p), p, 1e-14));
    }

    #[test]
    fn rotation_about_offset_axis() {
        let r = Xform::rotation(Vec3::new(1.0, 0.0, 0.0), Vec3::Z, std::f64::consts::PI);
        assert!(vclose(r * Vec3::ZERO, Vec3::new(2.0, 0.0, 0.0), 1e-15));
        assert!(vclose(r * Vec3::new(1.0, 0.0, 5.0), Vec3::new(1.0, 0.0, 5.0), 1e-15));
        let q = r.rotation_quat().unwrap();
        assert!(vclose(q.rotate(Vec3::X), -Vec3::X, 1e-15));
    }

    #[test]
    fn mirror_and_frames() {
        let pl = Plane::new(Vec3::new(2.0, 0.0, 0.0), Vec3::X);
        let m = Xform::mirror(&pl);
        assert!(m.is_mirror());
        assert!(vclose(m * Vec3::new(3.0, 1.0, 1.0), Vec3::new(1.0, 1.0, 1.0), 1e-15));
        let f = m.apply_frame(&std_plane("front"));
        assert!(f.ortho_error() < 1e-15, "mirrored frame stays right-handed");
        assert_eq!(f.o, Vec3::new(4.0, 0.0, 0.0));
        let tp = m.apply_plane(&Plane::new(Vec3::ZERO, Vec3::new(1.0, 1.0, 0.0)));
        assert!(vclose(tp.n, Vec3::new(-1.0, 1.0, 0.0).normalize(), 1e-15));
        // frame_to_frame takes a onto b
        let a = std_plane("top");
        let b = Frame::from_normal(Vec3::new(5.0, 6.0, 7.0), Vec3::new(1.0, 1.0, 1.0));
        let t = Xform::frame_to_frame(&a, &b);
        let g = t.apply_frame(&a);
        assert!(vclose(g.o, b.o, 1e-14) && vclose(g.x, b.x, 1e-14) && vclose(g.z, b.z, 1e-14));
    }
}
