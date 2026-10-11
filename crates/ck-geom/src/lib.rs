//! ck-geom: curve and surface geometry (analytic + NURBS), between ck-core
//! math and ck-topo topology. Representation only — no topology here.
//!
//! Evidence: module layout from rs_paths.txt (crates/ck-geom/src/*); kind
//! string tables from kernel.strings:
//! - surfaces: `plane cylinder cone sphere torus spline offset extrusion revolution`
//! - curves:   `line circle ellipse spline curve conic sinusoid pcurve`
//! - STEP writer entities: PLANE, CYLINDRICAL_SURFACE, CONICAL_SURFACE,
//!   SPHERICAL_SURFACE, TOROIDAL_SURFACE, SURFACE_OF_LINEAR_EXTRUSION,
//!   SURFACE_OF_REVOLUTION, OFFSET_SURFACE, B_SPLINE_SURFACE_WITH_KNOTS,
//!   RATIONAL_B_SPLINE_SURFACE, LINE, CIRCLE, ELLIPSE, B_SPLINE_CURVE_WITH_KNOTS.
//!   Analytic parameterisations below follow ISO 10303-42 so the STEP writer
//!   can emit them 1:1.

pub mod analysis;
pub mod convert;
pub mod curve;
pub mod curve2;
pub mod fit;
pub mod nurbs;
pub mod pcurve;
pub mod surface;

pub use ck_core::linalg::{Frame, Vec3};

use std::ops::{Add, Div, Mul, Neg, Sub};

/// Default geometric tolerance (model units, mm).
/// GUESS: 1e-9 relative-ish epsilon for parameter/zero checks; the kernel's
/// own modelling tolerance lives in ck-core.
pub const EPS: f64 = 1e-12;

/// 2D vector for sketch space and surface parameter (pcurve) space.
/// Local to ck-geom until ck-core grows one.
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
    /// z-component of the 3D cross product.
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
    pub fn normalize(self) -> Vec2 {
        let l = self.len();
        if l == 0.0 {
            self
        } else {
            self / l
        }
    }
    /// Rotate +90° (left normal).
    pub fn perp(self) -> Vec2 {
        Vec2::new(-self.y, self.x)
    }
    pub fn lerp(self, o: Vec2, t: f64) -> Vec2 {
        self + (o - self) * t
    }
    pub fn to3(self) -> Vec3 {
        Vec3::new(self.x, self.y, 0.0)
    }
}

impl Add for Vec2 {
    type Output = Vec2;
    fn add(self, o: Vec2) -> Vec2 {
        Vec2::new(self.x + o.x, self.y + o.y)
    }
}
impl Sub for Vec2 {
    type Output = Vec2;
    fn sub(self, o: Vec2) -> Vec2 {
        Vec2::new(self.x - o.x, self.y - o.y)
    }
}
impl Mul<f64> for Vec2 {
    type Output = Vec2;
    fn mul(self, s: f64) -> Vec2 {
        Vec2::new(self.x * s, self.y * s)
    }
}
impl Div<f64> for Vec2 {
    type Output = Vec2;
    fn div(self, s: f64) -> Vec2 {
        Vec2::new(self.x / s, self.y / s)
    }
}
impl Neg for Vec2 {
    type Output = Vec2;
    fn neg(self) -> Vec2 {
        Vec2::new(-self.x, -self.y)
    }
}

/// Point/vector abstraction so NURBS code is shared between 2D and 3D.
pub trait Pt:
    Copy
    + Default
    + PartialEq
    + std::fmt::Debug
    + Add<Output = Self>
    + Sub<Output = Self>
    + Mul<f64, Output = Self>
    + Div<f64, Output = Self>
    + Neg<Output = Self>
{
    fn dot(self, o: Self) -> f64;
    fn len(self) -> f64 {
        self.dot(self).sqrt()
    }
    fn dist(self, o: Self) -> f64 {
        (self - o).len()
    }
}

impl Pt for Vec2 {
    fn dot(self, o: Vec2) -> f64 {
        Vec2::dot(self, o)
    }
}
impl Pt for Vec3 {
    fn dot(self, o: Vec3) -> f64 {
        Vec3::dot(self, o)
    }
}

/// Any unit vector perpendicular to `n` (deterministic choice: crosses with
/// the world axis least aligned with `n`).
/// GUESS: the kernel's exact ref-direction choice for circles without an
/// explicit x axis is unknown; this is the common "least-aligned axis" rule.
pub fn any_perp(n: Vec3) -> Vec3 {
    let a = n.abs();
    let t = if a.x <= a.y && a.x <= a.z {
        Vec3::X
    } else if a.y <= a.z {
        Vec3::Y
    } else {
        Vec3::Z
    };
    n.cross(t).normalize()
}

/// Build a right-handed orthonormal frame with origin `o`, z = `z`, and x as
/// close as possible to `xref` (Gram-Schmidt). Matches STEP AXIS2_PLACEMENT_3D
/// semantics (axis + ref_direction).
pub fn frame_from(o: Vec3, z: Vec3, xref: Vec3) -> Frame {
    let z = z.normalize();
    let mut x = xref - z * xref.dot(z);
    if x.len() < 1e-12 {
        x = any_perp(z);
    }
    let x = x.normalize();
    let y = z.cross(x);
    Frame { o, x, y, z }
}

#[cfg(test)]
pub(crate) mod testutil {
    use super::*;
    pub fn close(a: f64, b: f64, tol: f64) -> bool {
        (a - b).abs() <= tol * (1.0 + a.abs().max(b.abs()))
    }
    pub fn vclose(a: Vec3, b: Vec3, tol: f64) -> bool {
        (a - b).len() <= tol * (1.0 + a.len().max(b.len()))
    }
    pub fn v2close(a: Vec2, b: Vec2, tol: f64) -> bool {
        (a - b).len() <= tol * (1.0 + a.len().max(b.len()))
    }
}
