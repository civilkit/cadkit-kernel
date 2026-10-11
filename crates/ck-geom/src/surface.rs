//! Surfaces: analytic (plane, cylinder, cone, sphere, torus), swept
//! (extrusion, revolution), offset, and B-spline.
//!
//! Evidence:
//! - kernel.strings kind runs `plane cylinder cone sphere torus spline
//!   offset extrusion revolution` (and the same without `spline`).
//! - STEP writer entities: PLANE, CYLINDRICAL_SURFACE, CONICAL_SURFACE
//!   (radius + semi_angle), SPHERICAL_SURFACE, TOROIDAL_SURFACE,
//!   SURFACE_OF_LINEAR_EXTRUSION, SURFACE_OF_REVOLUTION, OFFSET_SURFACE,
//!   B_SPLINE_SURFACE_WITH_KNOTS. Parameterisations are the ISO 10303-42
//!   ones so they map 1:1 onto those entities:
//!   - cylinder σ = C + R(cos u·x + sin u·y) + v·z
//!   - cone     σ = C + (R + v tan α)(cos u·x + sin u·y) + v·z
//!   - sphere   σ = C + R cos v(cos u·x + sin u·y) + R sin v·z
//!   - torus    σ = C + (R + r cos v)(cos u·x + sin u·y) + r sin v·z
//!   - revolution: u = angle about the axis, v = profile parameter
//!   - extrusion:  σ = λ(u) + v·V
//! - oracle-flange.json face kinds: 82 plane, 38 cylinder, 40 cone,
//!   6 torus, 16 spline.
//!
//! Natural normal = Su × Sv (outward for the analytic closed forms). Face
//! orientation (sense) is a topology concern.

use crate::curve::{wrap_angle, Curve};
use crate::nurbs::bsurf::BSplineSurface;
use crate::{Frame, Vec3};
use std::f64::consts::{FRAC_PI_2, TAU};

/// Point + first and second partial derivatives.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct SurfDerivs {
    pub p: Vec3,
    pub du: Vec3,
    pub dv: Vec3,
    pub duu: Vec3,
    pub duv: Vec3,
    pub dvv: Vec3,
}

impl SurfDerivs {
    /// Unit normal Su × Sv (zero at degenerate points).
    pub fn normal(&self) -> Vec3 {
        self.du.cross(self.dv).normalize()
    }
}

#[derive(Debug, Clone, PartialEq)]
pub enum Surface {
    /// σ = o + u·x + v·y, normal z.
    Plane { frame: Frame },
    Cylinder { frame: Frame, r: f64 },
    /// `r` = radius at v = 0, `semi_angle` in radians (STEP CONICAL_SURFACE).
    Cone { frame: Frame, r: f64, semi_angle: f64 },
    Sphere { frame: Frame, r: f64 },
    Torus { frame: Frame, major: f64, minor: f64 },
    /// Profile swept about the axis through `axis_o` along unit `axis`.
    Revolution { profile: Box<Curve>, axis_o: Vec3, axis: Vec3 },
    /// Profile swept along `dir` (magnitude kept: v=1 is one `dir`).
    Extrusion { profile: Box<Curve>, dir: Vec3 },
    BSpline(Box<BSplineSurface>),
    /// σ = base + d·N_base.
    Offset { base: Box<Surface>, dist: f64 },
}

const INF: f64 = f64::INFINITY;

impl Surface {
    pub fn plane(o: Vec3, n: Vec3) -> Surface {
        Surface::Plane { frame: crate::frame_from(o, n, crate::any_perp(n)) }
    }

    /// Wire kind string (kernel.strings table).
    /// GUESS: regen face JSON may fold extrusion/revolution/offset into
    /// `spline`; the flange golden contains none of those.
    pub fn kind(&self) -> &'static str {
        match self {
            Surface::Plane { .. } => "plane",
            Surface::Cylinder { .. } => "cylinder",
            Surface::Cone { .. } => "cone",
            Surface::Sphere { .. } => "sphere",
            Surface::Torus { .. } => "torus",
            Surface::Revolution { .. } => "revolution",
            Surface::Extrusion { .. } => "extrusion",
            Surface::BSpline(_) => "spline",
            Surface::Offset { .. } => "offset",
        }
    }

    /// The face `planar` flag: true for planes and for geometrically flat
    /// splines / line extrusions / offsets of planes.
    pub fn is_planar(&self, tol: f64) -> bool {
        match self {
            Surface::Plane { .. } => true,
            Surface::Offset { base, .. } => base.is_planar(tol),
            Surface::Extrusion { profile, .. } => matches!(**profile, Curve::Line { .. }),
            Surface::BSpline(s) => points_coplanar(&s.ctrl, tol),
            _ => false,
        }
    }

    /// ((u0,u1),(v0,v1)) natural domain (may be infinite).
    pub fn domain(&self) -> ((f64, f64), (f64, f64)) {
        match self {
            Surface::Plane { .. } => ((-INF, INF), (-INF, INF)),
            Surface::Cylinder { .. } | Surface::Cone { .. } => ((0.0, TAU), (-INF, INF)),
            Surface::Sphere { .. } => ((0.0, TAU), (-FRAC_PI_2, FRAC_PI_2)),
            Surface::Torus { .. } => ((0.0, TAU), (0.0, TAU)),
            Surface::Revolution { profile, .. } => ((0.0, TAU), profile.range()),
            Surface::Extrusion { profile, .. } => (profile.range(), (-INF, INF)),
            Surface::BSpline(s) => s.domain(),
            Surface::Offset { base, .. } => base.domain(),
        }
    }

    /// Periods in (u, v).
    pub fn periods(&self) -> (Option<f64>, Option<f64>) {
        match self {
            Surface::Cylinder { .. } | Surface::Cone { .. } | Surface::Sphere { .. } | Surface::Revolution { .. } => {
                (Some(TAU), None)
            }
            Surface::Torus { .. } => (Some(TAU), Some(TAU)),
            Surface::Extrusion { profile, .. } => (profile.period(), None),
            Surface::Offset { base, .. } => base.periods(),
            _ => (None, None),
        }
    }

    pub fn eval(&self, u: f64, v: f64) -> Vec3 {
        match self {
            Surface::Plane { frame } => frame.o + frame.x * u + frame.y * v,
            Surface::Cylinder { frame, r } => frame.o + radial(frame, u) * *r + frame.z * v,
            Surface::Cone { frame, r, semi_angle } => {
                frame.o + radial(frame, u) * (r + v * semi_angle.tan()) + frame.z * v
            }
            Surface::Sphere { frame, r } => frame.o + radial(frame, u) * (r * v.cos()) + frame.z * (r * v.sin()),
            Surface::Torus { frame, major, minor } => {
                frame.o + radial(frame, u) * (major + minor * v.cos()) + frame.z * (minor * v.sin())
            }
            Surface::BSpline(s) => s.eval(u, v),
            _ => self.derivs2(u, v).p,
        }
    }

    /// Point + first/second partials.
    pub fn derivs2(&self, u: f64, v: f64) -> SurfDerivs {
        match self {
            Surface::Plane { frame } => SurfDerivs {
                p: frame.o + frame.x * u + frame.y * v,
                du: frame.x,
                dv: frame.y,
                ..Default::default()
            },
            Surface::Cylinder { frame, r } => {
                let (rad, tan) = (radial(frame, u), tangential(frame, u));
                SurfDerivs {
                    p: frame.o + rad * *r + frame.z * v,
                    du: tan * *r,
                    dv: frame.z,
                    duu: -rad * *r,
                    ..Default::default()
                }
            }
            Surface::Cone { frame, r, semi_angle } => {
                let (rad, tan) = (radial(frame, u), tangential(frame, u));
                let t = semi_angle.tan();
                let rho = r + v * t;
                SurfDerivs {
                    p: frame.o + rad * rho + frame.z * v,
                    du: tan * rho,
                    dv: rad * t + frame.z,
                    duu: -rad * rho,
                    duv: tan * t,
                    dvv: Vec3::ZERO,
                }
            }
            Surface::Sphere { frame, r } => {
                let (rad, tan) = (radial(frame, u), tangential(frame, u));
                let (sv, cv) = v.sin_cos();
                SurfDerivs {
                    p: frame.o + rad * (r * cv) + frame.z * (r * sv),
                    du: tan * (r * cv),
                    dv: rad * (-r * sv) + frame.z * (r * cv),
                    duu: rad * (-r * cv),
                    duv: tan * (-r * sv),
                    dvv: rad * (-r * cv) + frame.z * (-r * sv),
                }
            }
            Surface::Torus { frame, major, minor } => {
                let (rad, tan) = (radial(frame, u), tangential(frame, u));
                let (sv, cv) = v.sin_cos();
                let rho = major + minor * cv;
                SurfDerivs {
                    p: frame.o + rad * rho + frame.z * (minor * sv),
                    du: tan * rho,
                    dv: rad * (-minor * sv) + frame.z * (minor * cv),
                    duu: -rad * rho,
                    duv: tan * (-minor * sv),
                    dvv: rad * (-minor * cv) + frame.z * (-minor * sv),
                }
            }
            Surface::Revolution { profile, axis_o, axis } => {
                let c = profile.derivs(v, 2);
                let (su, cu) = u.sin_cos();
                let z = *axis;
                // Rodrigues: R(u)w = w cos u + (z·w) z (1 - cos u) + (z×w) sin u
                let rot = |w: Vec3| w * cu + z * (z.dot(w) * (1.0 - cu)) + z.cross(w) * su;
                let rot_u = |w: Vec3| w * (-su) + z * (z.dot(w) * su) + z.cross(w) * cu;
                let rot_uu = |w: Vec3| w * (-cu) + z * (z.dot(w) * cu) - z.cross(w) * su;
                let w = c[0] - *axis_o;
                SurfDerivs {
                    p: *axis_o + rot(w),
                    du: rot_u(w),
                    dv: rot(c[1]),
                    duu: rot_uu(w),
                    duv: rot_u(c[1]),
                    dvv: rot(c[2]),
                }
            }
            Surface::Extrusion { profile, dir } => {
                let c = profile.derivs(u, 2);
                SurfDerivs { p: c[0] + *dir * v, du: c[1], dv: *dir, duu: c[2], ..Default::default() }
            }
            Surface::BSpline(s) => s.derivs2(u, v),
            Surface::Offset { base, dist } => {
                // GUESS/approx: normal derivatives by central differences of
                // the base normal; exact for analytic bases up to O(h²).
                let b = base.derivs2(u, v);
                let n = b.normal();
                let h = 1e-6;
                let nu = (base.normal(u + h, v) - base.normal(u - h, v)) / (2.0 * h);
                let nv = (base.normal(u, v + h) - base.normal(u, v - h)) / (2.0 * h);
                let nuu = (base.normal(u + h, v) - n * 2.0 + base.normal(u - h, v)) / (h * h);
                let nvv = (base.normal(u, v + h) - n * 2.0 + base.normal(u, v - h)) / (h * h);
                let nuv = (base.normal(u + h, v + h) - base.normal(u + h, v - h) - base.normal(u - h, v + h)
                    + base.normal(u - h, v - h))
                    / (4.0 * h * h);
                SurfDerivs {
                    p: b.p + n * *dist,
                    du: b.du + nu * *dist,
                    dv: b.dv + nv * *dist,
                    duu: b.duu + nuu * *dist,
                    duv: b.duv + nuv * *dist,
                    dvv: b.dvv + nvv * *dist,
                }
            }
        }
    }

    /// (Su, Sv)
    pub fn partials(&self, u: f64, v: f64) -> (Vec3, Vec3) {
        let d = self.derivs2(u, v);
        (d.du, d.dv)
    }

    /// Unit natural normal. Handles analytic poles (sphere/cone apex) by
    /// returning the limiting normal.
    pub fn normal(&self, u: f64, v: f64) -> Vec3 {
        match self {
            Surface::Plane { frame } => frame.z,
            Surface::Cylinder { frame, .. } => radial(frame, u),
            Surface::Sphere { frame, .. } => radial(frame, u) * v.cos() + frame.z * v.sin(),
            Surface::Torus { frame, .. } => radial(frame, u) * v.cos() + frame.z * v.sin(),
            Surface::Cone { frame, r, semi_angle } => {
                // Su×Sv = ρ(radial − tan α·z), ρ = R + v tan α: outward
                // from the axis on the ρ > 0 nappe, flipped past the apex.
                let rho = r + v * semi_angle.tan();
                let n = radial(frame, u) * semi_angle.cos() - frame.z * semi_angle.sin();
                if rho < 0.0 {
                    -n
                } else {
                    n
                }
            }
            _ => self.derivs2(u, v).normal(),
        }
    }

    /// Parameters of the closest point to `q`.
    pub fn closest_param(&self, q: Vec3) -> (f64, f64) {
        match self {
            Surface::Plane { frame } => {
                let l = frame.to_local(q);
                (l.x, l.y)
            }
            Surface::Cylinder { frame, .. } => {
                let l = frame.to_local(q);
                (ang(l.x, l.y), l.z)
            }
            Surface::Cone { frame, r, semi_angle } => {
                let l = frame.to_local(q);
                let rho = (l.x * l.x + l.y * l.y).sqrt();
                let t = semi_angle.tan();
                // project (rho, z) onto the generator (r + v t, v)
                let v = ((rho - r) * t + l.z) / (1.0 + t * t);
                (ang(l.x, l.y), v)
            }
            Surface::Sphere { frame, .. } => {
                let l = frame.to_local(q);
                let rho = (l.x * l.x + l.y * l.y).sqrt();
                (ang(l.x, l.y), l.z.atan2(rho))
            }
            Surface::Torus { frame, major, .. } => {
                let l = frame.to_local(q);
                let rho = (l.x * l.x + l.y * l.y).sqrt();
                (ang(l.x, l.y), wrap_angle(l.z.atan2(rho - major)))
            }
            Surface::BSpline(s) => s.closest_param(q),
            Surface::Offset { base, .. } => base.closest_param(q),
            Surface::Extrusion { profile, dir } => {
                // exact: project q along dir onto the profile's "base" by
                // minimising over u with v = (q - λ(u))·V/|V|²
                let d2 = dir.len2();
                let (a, b) = finite_range(profile.range(), q, profile);
                let n = 64;
                let mut best = (INF, a, 0.0);
                for i in 0..=n {
                    let u = a + (b - a) * i as f64 / n as f64;
                    let c = profile.eval(u);
                    let v = (q - c).dot(*dir) / d2;
                    let dd = (c + *dir * v).dist(q);
                    if dd < best.0 {
                        best = (dd, u, v);
                    }
                }
                let dom = self.domain();
                newton_surface(|u, v| self.derivs2(u, v), q, (best.1, best.2), dom)
            }
            Surface::Revolution { profile, axis_o, axis } => {
                let (a, b) = finite_range(profile.range(), q, profile);
                let nu = 36;
                let nv = 48;
                let mut best = (INF, 0.0, a);
                for i in 0..nu {
                    let u = TAU * i as f64 / nu as f64;
                    for j in 0..=nv {
                        let v = a + (b - a) * j as f64 / nv as f64;
                        let d = self.eval(u, v).dist(q);
                        if d < best.0 {
                            best = (d, u, v);
                        }
                    }
                }
                let _ = (axis_o, axis);
                let ((_, _), (v0, v1)) = self.domain();
                let (u, v) = newton_surface(|u, v| self.derivs2(u, v), q, (best.1, best.2), ((-INF, INF), (v0, v1)));
                (wrap_angle(u), v)
            }
        }
    }

    /// (u, v, point)
    pub fn closest_point(&self, q: Vec3) -> (f64, f64, Vec3) {
        let (u, v) = self.closest_param(q);
        (u, v, self.eval(u, v))
    }

    /// Distance from `q` to the (untrimmed) surface.
    pub fn distance(&self, q: Vec3) -> f64 {
        self.closest_point(q).2.dist(q)
    }
}

#[inline]
fn radial(f: &Frame, u: f64) -> Vec3 {
    let (s, c) = u.sin_cos();
    f.x * c + f.y * s
}
#[inline]
fn tangential(f: &Frame, u: f64) -> Vec3 {
    let (s, c) = u.sin_cos();
    f.x * (-s) + f.y * c
}
#[inline]
fn ang(x: f64, y: f64) -> f64 {
    if x == 0.0 && y == 0.0 {
        0.0
    } else {
        wrap_angle(y.atan2(x))
    }
}

/// Finite search range for a (possibly unbounded) curve parameter range:
/// lines use the projection of q ± a generous window.
fn finite_range(r: (f64, f64), q: Vec3, c: &Curve) -> (f64, f64) {
    if r.0.is_finite() && r.1.is_finite() {
        return r;
    }
    let t = c.closest_param(q);
    (t - 1.0, t + 1.0)
}

fn points_coplanar(pts: &[Vec3], tol: f64) -> bool {
    if pts.len() < 4 {
        return true;
    }
    let p0 = pts[0];
    let mut n = Vec3::ZERO;
    'outer: for i in 1..pts.len() {
        for j in (i + 1)..pts.len() {
            let c = (pts[i] - p0).cross(pts[j] - p0);
            if c.len() > 1e-9 * (pts[i] - p0).len().max(1.0) * (pts[j] - p0).len().max(1.0) {
                n = c.normalize();
                break 'outer;
            }
        }
    }
    if n == Vec3::ZERO {
        return true;
    }
    pts.iter().all(|&p| (p - p0).dot(n).abs() <= tol)
}

/// 2D Newton for the foot point on a parametric surface given a
/// second-derivative oracle; clamps to `dom` (infinite bounds = free).
pub(crate) fn newton_surface(
    d: impl Fn(f64, f64) -> SurfDerivs,
    q: Vec3,
    seed: (f64, f64),
    dom: ((f64, f64), (f64, f64)),
) -> (f64, f64) {
    let ((u0, u1), (v0, v1)) = dom;
    let (mut u, mut v) = seed;
    for _ in 0..50 {
        let s = d(u, v);
        let r = s.p - q;
        let f = s.du.dot(r);
        let g = s.dv.dot(r);
        let j11 = s.du.dot(s.du) + r.dot(s.duu);
        let j12 = s.du.dot(s.dv) + r.dot(s.duv);
        let j22 = s.dv.dot(s.dv) + r.dot(s.dvv);
        let det = j11 * j22 - j12 * j12;
        let (du, dv) = if det.abs() > 1e-300 {
            (-(f * j22 - g * j12) / det, -(g * j11 - f * j12) / det)
        } else {
            // degenerate: gradient step on whichever direction is alive
            let a = if j11.abs() > 1e-300 { -f / j11 } else { 0.0 };
            let b = if j22.abs() > 1e-300 { -g / j22 } else { 0.0 };
            (a, b)
        };
        let nu = (u + du).clamp(u0, u1);
        let nv = (v + dv).clamp(v0, v1);
        let done = (nu - u).abs() <= 1e-14 * (1.0 + u.abs()) && (nv - v).abs() <= 1e-14 * (1.0 + v.abs());
        u = nu;
        v = nv;
        if done {
            break;
        }
    }
    (u, v)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::frame_from;
    use crate::testutil::*;

    fn fr() -> Frame {
        frame_from(Vec3::new(1., -2., 0.5), Vec3::new(0.3, 0.4, 1.0), Vec3::X)
    }

    fn all() -> Vec<Surface> {
        let f = fr();
        let prof = Curve::circle_x(Vec3::new(5., 0., 0.), Vec3::Y, Vec3::X, 1.5);
        vec![
            Surface::Plane { frame: f },
            Surface::Cylinder { frame: f, r: 2.0 },
            Surface::Cone { frame: f, r: 2.0, semi_angle: 0.4 },
            Surface::Sphere { frame: f, r: 3.0 },
            Surface::Torus { frame: f, major: 5.0, minor: 1.5 },
            Surface::Revolution { profile: Box::new(prof.clone()), axis_o: Vec3::ZERO, axis: Vec3::Z },
            Surface::Extrusion {
                profile: Box::new(Curve::ellipse(Vec3::ZERO, Vec3::Z, Vec3::X, 3.0, 1.0)),
                dir: Vec3::new(0.2, 0.0, 2.0),
            },
            Surface::Offset { base: Box::new(Surface::Sphere { frame: f, r: 3.0 }), dist: 0.5 },
        ]
    }

    #[test]
    fn kinds() {
        let k: Vec<_> = all().iter().map(|s| s.kind()).collect();
        assert_eq!(k, ["plane", "cylinder", "cone", "sphere", "torus", "revolution", "extrusion", "offset"]);
    }

    #[test]
    fn partials_fd() {
        let h = 1e-5;
        for s in all() {
            for &(u, v) in &[(0.3, 0.2), (2.0, -0.7), (4.5, 1.1)] {
                let d = s.derivs2(u, v);
                assert!(vclose(d.p, s.eval(u, v), 1e-14), "{}", s.kind());
                let fu = (s.eval(u + h, v) - s.eval(u - h, v)) / (2.0 * h);
                let fv = (s.eval(u, v + h) - s.eval(u, v - h)) / (2.0 * h);
                assert!(vclose(d.du, fu, 1e-7), "{} du", s.kind());
                assert!(vclose(d.dv, fv, 1e-7), "{} dv", s.kind());
                let fuu = (s.derivs2(u + h, v).du - s.derivs2(u - h, v).du) / (2.0 * h);
                let fuv = (s.derivs2(u, v + h).du - s.derivs2(u, v - h).du) / (2.0 * h);
                let fvv = (s.derivs2(u, v + h).dv - s.derivs2(u, v - h).dv) / (2.0 * h);
                let tol = if s.kind() == "offset" { 1e-3 } else { 1e-6 };
                assert!(vclose(d.duu, fuu, tol), "{} duu", s.kind());
                assert!(vclose(d.duv, fuv, tol), "{} duv", s.kind());
                assert!(vclose(d.dvv, fvv, tol), "{} dvv", s.kind());
                // closed-form normal agrees with Su x Sv
                assert!(vclose(s.normal(u, v), d.normal(), 1e-9), "{} normal", s.kind());
            }
        }
    }

    #[test]
    fn closest_param_roundtrip() {
        for s in all() {
            for &(u, v) in &[(0.3, 0.2), (2.0, -0.7), (4.5, 1.1)] {
                let p = s.eval(u, v);
                let n = s.normal(u, v);
                let q = p + n * 0.1;
                let (cu, cv) = s.closest_param(q);
                let back = s.eval(cu, cv);
                assert!(vclose(back, p, 1e-8), "{} ({u},{v}) -> ({cu},{cv})", s.kind());
            }
        }
    }

    #[test]
    fn analytic_outward_normals() {
        let f = Frame::identity();
        let c = Surface::Cylinder { frame: f, r: 2.0 };
        assert!(vclose(c.normal(0.0, 0.0), Vec3::X, 1e-15));
        let s = Surface::Sphere { frame: f, r: 2.0 };
        assert!(vclose(s.normal(0.0, FRAC_PI_2), Vec3::Z, 1e-15));
        let k = Surface::Cone { frame: f, r: 1.0, semi_angle: 0.0 };
        assert!(vclose(k.eval(0.5, 3.0), c.eval(0.5, 3.0) * 0.5 + Vec3::Z * 1.5, 1e-14));
        assert!(Surface::Plane { frame: f }.is_planar(1e-9));
        assert!(!c.is_planar(1e-9));
    }
}
