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
//! - oracle probe (surfaceOffset on an extruded circle R=10, then
//!   `curvatureAt`): the offset face at [12,10,0] reports n=[1,0,0],
//!   k1=1/12, k2=0 — offset of a cylinder R by d is the cylinder R+d
//!   (outward normal kept), so offset second derivatives are pinned by the
//!   adjusted-radius analytic forms.
//!
//! Natural normal = Su × Sv (outward for the analytic closed forms). Face
//! orientation (sense) is a topology concern.
//!
//! Offsets are exact: O = B + d·n with the unit normal's derivatives
//! differentiated from the cross-product field W = Su × Sv (see
//! `derivs2`). This needs third partials of the base (`thirds`); nested
//! offsets collapse to one level because offset surfaces keep the base
//! normals.

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

/// Third partial derivatives ∂³S/∂u³, ∂³S/∂u²∂v, ∂³S/∂u∂v², ∂³S/∂v³.
/// Needed for the analytic derivatives of offset surfaces (the unit
/// normal's second derivatives differentiate the cross-product field, which
/// is genuinely third-order data — no second-order shortcut exists).
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct SurfThirds {
    pub duuu: Vec3,
    pub duuv: Vec3,
    pub duvv: Vec3,
    pub dvvv: Vec3,
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
    /// Screw (helicoidal) patch `σ(u, v) = o + R(u) (c(v) - o) + rise·u·axis`: the profile `c` carried by a
    /// rotation `u` about the axis through `axis_o` along unit `axis` plus an axial advance `rise` per
    /// radian. Bounded like a B-spline patch: `u` in `urange`, `v` in `vrange` (closest points clamp to it).
    Screw { profile: Box<Curve>, axis_o: Vec3, axis: Vec3, rise: f64, urange: (f64, f64), vrange: (f64, f64) },
    /// σ = base + d·N_base.
    Offset { base: Box<Surface>, dist: f64 },
}

const INF: f64 = f64::INFINITY;

/// First-order evaluator of a screw patch along one iso-v line (see [`Surface::screw_vline`]).
#[derive(Debug, Clone, Copy)]
pub struct ScrewVLine {
    w: Vec3,
    c1: Vec3,
    axis: Vec3,
    axis_o: Vec3,
    rise: f64,
}

impl ScrewVLine {
    /// (point, S_u, S_v) at rotation `u`.
    #[inline]
    pub fn eval1(&self, u: f64) -> (Vec3, Vec3, Vec3) {
        let (su, cu) = u.sin_cos();
        let z = self.axis;
        let rot = |w: Vec3| w * cu + z * (z.dot(w) * (1.0 - cu)) + z.cross(w) * su;
        let rot_u = |w: Vec3| w * (-su) + z * (z.dot(w) * su) + z.cross(w) * cu;
        (self.axis_o + rot(self.w) + z * (self.rise * u), rot_u(self.w) + z * self.rise, rot(self.c1))
    }
}

impl Surface {
    /// Hash identifying the surface's geometry (equal surfaces hash equal;
    /// for memoising pure functions of surfaces).
    pub fn fingerprint(&self) -> u64 {
        use std::hash::{Hash, Hasher};
        match self {
            Surface::BSpline(b) => b.fingerprint(),
            Surface::Offset { base, dist } => base.fingerprint().rotate_left(17) ^ dist.to_bits().wrapping_mul(0x9e3779b97f4a7c15),
            other => {
                let mut h = std::collections::hash_map::DefaultHasher::new();
                format!("{other:?}").hash(&mut h);
                h.finish()
            }
        }
    }

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
            Surface::BSpline(_) | Surface::Screw { .. } => "spline",
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

    /// Evaluator along the iso-v line of a screw patch (the profile is evaluated once): `None` for other
    /// surfaces.
    pub fn screw_vline(&self, v: f64) -> Option<ScrewVLine> {
        let Surface::Screw { profile, axis_o, axis, rise, .. } = self else { return None };
        let c = profile.derivs(v, 1);
        Some(ScrewVLine { w: c[0] - *axis_o, c1: c[1], axis: *axis, axis_o: *axis_o, rise: *rise })
    }

    /// A multi-turn sweep patch (a long multi-span B-spline or a screw of more than ~1.3 turns): global
    /// searches and winding numbers over it are costly, so callers use windowed marching and trust the
    /// nearest-feature test.
    pub fn is_long_sweep(&self) -> bool {
        match self {
            Surface::BSpline(bs) => bs.knots_u.len() > 64,
            Surface::Screw { urange, .. } => urange.1 - urange.0 > 8.0,
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
            Surface::Screw { urange, vrange, .. } => (*urange, *vrange),
            Surface::Offset { base, .. } => base.domain(),
        }
    }

    /// Is `(u, v)` a pole of a u-periodic surface (sphere pole, cone apex): the
    /// whole iso-v circle collapses to a point within `1e-5` relative (sketch
    /// noise leaves seam parameters a few 1e-6 rad short of the exact pole).
    pub fn is_pole(&self, u: f64, v: f64) -> bool {
        let Some(per) = self.periods().0 else { return false };
        let (a, b) = (self.eval(u, v), self.eval(u + 0.5 * per, v));
        a.dist(b) <= 1e-5 * (1.0 + a.len())
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
            Surface::Screw { profile, axis_o, axis, rise, .. } => {
                let (su, cu) = u.sin_cos();
                let w = profile.eval(v) - *axis_o;
                *axis_o + w * cu + *axis * (axis.dot(w) * (1.0 - cu) + rise * u) + axis.cross(w) * su
            }
            _ => self.derivs2(u, v).p,
        }
    }

    /// Points on the grid `us x vs`, row-major (`i * vs.len() + j`); each is
    /// bit-identical to `eval(us[i], vs[j])` (splines share their basis work).
    pub fn eval_grid(&self, us: &[f64], vs: &[f64]) -> Vec<Vec3> {
        match self {
            Surface::BSpline(s) => s.eval_grid(us, vs),
            _ => us.iter().flat_map(|&u| vs.iter().map(move |&v| self.eval(u, v))).collect(),
        }
    }

    /// Point + first/second partials.
    pub fn derivs2(&self, u: f64, v: f64) -> SurfDerivs {        match self {
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
            Surface::Screw { profile, axis_o, axis, rise, .. } => {
                let c = profile.derivs(v, 2);
                let (su, cu) = u.sin_cos();
                let z = *axis;
                let rot = |w: Vec3| w * cu + z * (z.dot(w) * (1.0 - cu)) + z.cross(w) * su;
                let rot_u = |w: Vec3| w * (-su) + z * (z.dot(w) * su) + z.cross(w) * cu;
                let rot_uu = |w: Vec3| w * (-cu) + z * (z.dot(w) * cu) - z.cross(w) * su;
                let w = c[0] - *axis_o;
                SurfDerivs {
                    p: *axis_o + rot(w) + z * (rise * u),
                    du: rot_u(w) + z * *rise,
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
                // Analytic: O = B + d·n with n = W/w, W = Bu × Bv. The
                // normal's derivatives differentiate the cross-product field
                // (only base 2nd/3rd partials needed):
                //   n_u = (W_u − α W)/w, n_uu = (W_uu − 2αW_u + (2α² − β)W)/w
                // with α = w_u/w, β = w_uu/w; likewise for v (and the mixed
                // n_uv = (W_uv − αW_v − γW_u + (2αγ − β_uv)W)/w). Nested
                // offsets collapse: offset surfaces keep the base normals,
                // so offset(offset(B, d1), d2) = offset(B, d1 + d2).
                let (b0, d0) = offset_collapse(base);
                let d = *dist + d0;
                let b = b0.derivs2(u, v);
                let w2 = b.du.cross(b.dv).len2();
                if w2 <= 1e-300 {
                    // degenerate base (apex/pole): keep the finite-difference
                    // normal derivatives of the old fallback
                    return offset_fd(b0, u, v, d);
                }
                let t3 = b0.thirds(u, v);
                let w0 = b.du.cross(b.dv);
                let w = w2.sqrt();
                let n = w0 / w;
                let wu = b.duu.cross(b.dv) + b.du.cross(b.duv);
                let wv = b.duv.cross(b.dv) + b.du.cross(b.dvv);
                let (su, sv) = (w0.dot(wu) / w, w0.dot(wv) / w);
                let nu = (wu - n * su) / w;
                let nv = (wv - n * sv) / w;
                let wuu = t3.duuu.cross(b.dv) + b.duu.cross(b.duv) * 2.0 + b.du.cross(t3.duuv);
                let wuv = t3.duuv.cross(b.dv) + b.duu.cross(b.dvv) + b.du.cross(t3.duvv);
                let wvv = t3.duvv.cross(b.dv) + b.duv.cross(b.dvv) * 2.0 + b.du.cross(t3.dvvv);
                let suu = (wu.dot(wu) + w0.dot(wuu)) / w - su * su / w;
                let suv = (wu.dot(wv) + w0.dot(wuv)) / w - su * sv / w;
                let svv = (wv.dot(wv) + w0.dot(wvv)) / w - sv * sv / w;
                let (a, c) = (su / w, sv / w);
                let nuu = (wuu - wu * (2.0 * a) + w0 * (2.0 * a * a - suu / w)) / w;
                let nuv = (wuv - wu * c - wv * a + w0 * (2.0 * a * c - suv / w)) / w;
                let nvv = (wvv - wv * (2.0 * c) + w0 * (2.0 * c * c - svv / w)) / w;
                SurfDerivs {
                    p: b.p + n * d,
                    du: b.du + nu * d,
                    dv: b.dv + nv * d,
                    duu: b.duu + nuu * d,
                    duv: b.duv + nuv * d,
                    dvv: b.dvv + nvv * d,
                }
            }
        }
    }

    /// ∂³ partials. Exact for the analytic surfaces and B-splines; offsets
    /// return finite differences of their analytic second partials (only
    /// reached by direct calls — the offset evaluation itself collapses
    /// nested offsets and uses the base's exact thirds).
    pub fn thirds(&self, u: f64, v: f64) -> SurfThirds {
        match self {
            Surface::Plane { .. } => SurfThirds::default(),
            Surface::Cylinder { frame, r } => {
                // Buu = −r·radial, so Buuu = −r·tangential (∂u radial = tang)
                SurfThirds { duuu: tangential(frame, u) * -r, ..Default::default() }
            }
            Surface::Cone { frame, r, semi_angle } => {
                let (rad, tan) = (radial(frame, u), tangential(frame, u));
                let t = semi_angle.tan();
                SurfThirds { duuu: tan * -(r + v * t), duuv: rad * -t, ..Default::default() }
            }
            Surface::Sphere { frame, r } => {
                let (rad, tan) = (radial(frame, u), tangential(frame, u));
                let (sv, cv) = v.sin_cos();
                SurfThirds {
                    duuu: tan * -(r * cv),
                    duuv: rad * (r * sv),
                    duvv: tan * -(r * cv),
                    dvvv: rad * (r * sv) + frame.z * -(r * cv),
                }
            }
            Surface::Torus { frame, major, minor } => {
                let (rad, tan) = (radial(frame, u), tangential(frame, u));
                let (sv, cv) = v.sin_cos();
                SurfThirds {
                    duuu: tan * -(major + minor * cv),
                    duuv: rad * (minor * sv),
                    duvv: tan * -(minor * cv),
                    dvvv: rad * (minor * sv) + frame.z * -(minor * cv),
                }
            }
            Surface::Revolution { profile, axis_o, axis } => {
                let c = profile.derivs(v, 3);
                let (su, cu) = u.sin_cos();
                let z = *axis;
                // Rodrigues: R(u)w = w cos u + (z·w) z (1 - cos u) + (z×w) sin u
                let rot = |w: Vec3| w * cu + z * (z.dot(w) * (1.0 - cu)) + z.cross(w) * su;
                let rot_u = |w: Vec3| w * (-su) + z * (z.dot(w) * su) + z.cross(w) * cu;
                let rot_uu = |w: Vec3| w * (-cu) + z * (z.dot(w) * cu) - z.cross(w) * su;
                let w = c[0] - *axis_o;
                // ∂u³ R = −∂u R; the v chain just rotates the profile's
                // derivatives
                SurfThirds {
                    duuu: rot_u(w) * -1.0,
                    duuv: rot_uu(c[1]),
                    duvv: rot_u(c[2]),
                    dvvv: rot(c[3]),
                }
            }
            Surface::Screw { profile, axis_o, axis, .. } => {
                let c = profile.derivs(v, 3);
                let (su, cu) = u.sin_cos();
                let z = *axis;
                let rot = |w: Vec3| w * cu + z * (z.dot(w) * (1.0 - cu)) + z.cross(w) * su;
                let rot_u = |w: Vec3| w * (-su) + z * (z.dot(w) * su) + z.cross(w) * cu;
                let rot_uu = |w: Vec3| w * (-cu) + z * (z.dot(w) * cu) - z.cross(w) * su;
                let w = c[0] - *axis_o;
                SurfThirds { duuu: rot_u(w) * -1.0, duuv: rot_uu(c[1]), duvv: rot_u(c[2]), dvvv: rot(c[3]) }
            }
            Surface::Extrusion { profile, .. } => {
                let c = profile.derivs(u, 3);
                SurfThirds { duuu: c[3], ..Default::default() }
            }
            Surface::BSpline(s) => {
                let skl = s.derivs(u, v, 3);
                SurfThirds { duuu: skl[3][0], duuv: skl[2][1], duvv: skl[1][2], dvvv: skl[0][3] }
            }
            Surface::Offset { .. } => {
                let h = 1e-5;
                SurfThirds {
                    duuu: (self.derivs2(u + h, v).duu - self.derivs2(u - h, v).duu) / (2.0 * h),
                    duuv: (self.derivs2(u + h, v).duv - self.derivs2(u - h, v).duv) / (2.0 * h),
                    duvv: (self.derivs2(u, v + h).duv - self.derivs2(u, v - h).duv) / (2.0 * h),
                    dvvv: (self.derivs2(u, v + h).dvv - self.derivs2(u, v - h).dvv) / (2.0 * h),
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
            // offsets keep the base's normal field
            Surface::Offset { base, .. } => offset_collapse(base).0.normal(u, v),
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
            Surface::Screw { .. } => screw_foot(self, q),
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
                if let Some(r) = revolution_foot(profile, *axis_o, *axis, q) {
                    return r;
                }
                let (a, b) = finite_range(profile.range(), q, profile);
                let nu = 36;
                let nv = 48;
                let mut best = (INF, 0.0, a);
                // the profile points once per v (the same values `eval` rotates)
                let ws: Vec<Vec3> = (0..=nv).map(|j| profile.derivs(a + (b - a) * j as f64 / nv as f64, 2)[0] - *axis_o).collect();
                let z = *axis;
                for i in 0..nu {
                    let u = TAU * i as f64 / nu as f64;
                    let (su, cu) = u.sin_cos();
                    for (j, &w) in ws.iter().enumerate() {
                        let v = a + (b - a) * j as f64 / nv as f64;
                        // Rodrigues, as in `derivs2`
                        let p = *axis_o + (w * cu + z * (z.dot(w) * (1.0 - cu)) + z.cross(w) * su);
                        let d = p.dist(q);
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

    /// [`Self::closest_point`] by Newton from the parameter `hint` (cheap
    /// when the hint is near the foot, e.g. the previous station of a
    /// march); falls back to the global search when the Newton foot is not a
    /// stationary point of the distance.
    pub fn closest_point_near(&self, q: Vec3, hint: (f64, f64)) -> (f64, f64, Vec3) {
        if matches!(self, Surface::BSpline(_) | Surface::Revolution { .. } | Surface::Extrusion { .. } | Surface::Screw { .. }) {
            let dom = match self {
                Surface::Revolution { .. } => {
                    let (_, (v0, v1)) = self.domain();
                    ((-INF, INF), (v0, v1))
                }
                _ => self.domain(),
            };
            let (u, v) = newton_surface(|u, v| self.derivs2(u, v), q, hint, dom);
            let s = self.derivs2(u, v);
            let r = s.p - q;
            let (nu, nv) = (s.du.len().max(1e-300), s.dv.len().max(1e-300));
            // stationary only: a Newton foot pinned on the domain boundary may
            // be a local minimum, so that case takes the global search
            if (s.du.dot(r) / nu).abs() < 1e-9 && (s.dv.dot(r) / nv).abs() < 1e-9 {
                let u = if matches!(self, Surface::Revolution { .. }) { wrap_angle(u) } else { u };
                return (u, v, s.p);
            }
        }
        self.closest_point(q)
    }

    /// As [`closest_point_within`](Self::closest_point_within), many queries
    /// against one surface: `prep` is `closest_prep()` of the B-spline (None
    /// for other surfaces).
    pub fn closest_point_within_prep(&self, prep: Option<&crate::nurbs::bsurf::CpPrep>, q: Vec3, maxd: f64) -> Option<(f64, f64, Vec3)> {
        if let (Surface::BSpline(bs), Some(p)) = (self, prep) {
            let (u, v) = bs.closest_param_within_with(p, q, maxd)?;
            return Some((u, v, self.eval(u, v)));
        }
        self.closest_point_within(q, maxd)
    }

    /// [`closest_point`](Self::closest_point), `None` when the surface is
    /// farther than `maxd` from `q` (cheap for long B-spline faces).
    pub fn closest_point_within(&self, q: Vec3, maxd: f64) -> Option<(f64, f64, Vec3)> {
        if let Surface::BSpline(bs) = self {
            let (u, v) = bs.closest_param_within(q, maxd)?;
            return Some((u, v, self.eval(u, v)));
        }
        let r = self.closest_point(q);
        (r.2.dist(q) <= maxd).then_some(r)
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

/// Nested offsets collapse to one level over a non-offset base: offset
/// surfaces keep the base's normals, so offset(offset(B, d1), d2) =
/// offset(B, d1 + d2).
fn offset_collapse(base: &Surface) -> (&Surface, f64) {
    let mut cur = base;
    let mut d = 0.0;
    while let Surface::Offset { base: b, dist } = cur {
        d += *dist;
        cur = &**b;
    }
    (cur, d)
}

/// Finite-difference fallback for degenerate bases (apex/pole) where the
/// cross-product field vanishes; O(h²), the old behaviour.
fn offset_fd(base: &Surface, u: f64, v: f64, d: f64) -> SurfDerivs {
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
        p: b.p + n * d,
        du: b.du + nu * d,
        dv: b.dv + nv * d,
        duu: b.duu + nuu * d,
        duv: b.duv + nuv * d,
        dvv: b.dvv + nvv * d,
    }
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

/// Foot of `q` on a surface of revolution whose profile lies in one
/// half-plane through the axis (the usual case): the foot is in `q`'s own
/// meridian plane, so `u` is `q`'s angle and `v` is a 1D closest-point search
/// on the profile (scan + Newton). `None` when the profile is not such a
/// one-sided planar curve (the general search takes over).
fn revolution_foot(profile: &Curve, axis_o: Vec3, axis: Vec3, q: Vec3) -> Option<(f64, f64)> {
    let a = axis.try_normalize()?;
    let (p0, p1) = profile.range();
    if !(p0.is_finite() && p1.is_finite()) || p1 <= p0 {
        return None;
    }
    const N: usize = 160;
    // the 161 profile samples depend on the surface only: cached for a B-spline
    // profile (the foot is asked for thousands of times per surface)
    struct RevPrep {
        scale: f64,
        er: Vec3,
        et: Vec3,
        ras: Vec<f64>,
        zs: Vec<f64>,
        ok: bool,
    }
    thread_local! {
        static CACHE: std::cell::RefCell<Vec<(u64, std::rc::Rc<RevPrep>)>> = const { std::cell::RefCell::new(Vec::new()) };
    }
    let key = if let Curve::BSpline(bs) = profile {
        let mut h: u64 = 0xcbf29ce484222325;
        let mut mix = |x: u64| {
            h = (h ^ x).wrapping_mul(0x100000001b3);
            h ^= h >> 29;
        };
        mix(bs.degree as u64);
        for &k in &bs.knots {
            mix(k.to_bits());
        }
        for c in &bs.ctrl {
            mix(c.x.to_bits());
            mix(c.y.to_bits());
            mix(c.z.to_bits());
        }
        if let Some(w) = &bs.weights {
            for x in w {
                mix(x.to_bits());
            }
        }
        for v in [axis_o, axis] {
            mix(v.x.to_bits());
            mix(v.y.to_bits());
            mix(v.z.to_bits());
        }
        Some(h)
    } else {
        None
    };
    let cached = key.and_then(|k| CACHE.with(|c| c.borrow().iter().find(|e| e.0 == k).map(|e| e.1.clone())));
    let prep = match cached {
        Some(p) => p,
        None => {
            let mut perp = Vec::with_capacity(N + 1);
            let mut zs = Vec::with_capacity(N + 1);
            for k in 0..=N {
                let w = profile.eval(p0 + (p1 - p0) * k as f64 / N as f64) - axis_o;
                let z = w.dot(a);
                zs.push(z);
                perp.push(w - a * z);
            }
            let scale = perp.iter().map(|w| w.len()).fold(0.0f64, f64::max).max(zs.iter().fold(0.0f64, |m, z| m.max(z.abs())));
            let er = perp.iter().copied().max_by(|x, y| x.len().total_cmp(&y.len())).and_then(|e| e.try_normalize());
            let prep = match er {
                None => RevPrep { scale, er: Vec3::ZERO, et: Vec3::ZERO, ras: vec![], zs, ok: false },
                Some(er) => {
                    let et = a.cross(er);
                    let ras: Vec<f64> = perp.iter().map(|w| w.dot(er)).collect();
                    // one-sided and planar through the axis
                    let ok = !(ras.iter().any(|&r| r < -1e-9 * (1.0 + scale)) || perp.iter().any(|w| w.dot(et).abs() > 1e-7 * (1.0 + scale)));
                    RevPrep { scale, er, et, ras, zs, ok }
                }
            };
            let prep = std::rc::Rc::new(prep);
            if let Some(k) = key {
                CACHE.with(|c| {
                    let mut c = c.borrow_mut();
                    if c.len() >= 32 {
                        c.remove(0);
                    }
                    c.push((k, prep.clone()));
                });
            }
            prep
        }
    };
    if !prep.ok {
        return None;
    }
    let (scale, er, et, ras, zs) = (prep.scale, prep.er, prep.et, &prep.ras, &prep.zs);
    let qw = q - axis_o;
    let qz = qw.dot(a);
    let qp = qw - a * qz;
    let (rho, phi) = (qp.len(), if qp.len() > 1e-12 * (1.0 + scale) { qp.dot(et).atan2(qp.dot(er)) } else { 0.0 });
    let mut best = (f64::INFINITY, 0usize);
    for k in 0..=N {
        let d = (ras[k] - rho).powi(2) + (zs[k] - qz).powi(2);
        if d < best.0 {
            best = (d, k);
        }
    }
    let mut v = p0 + (p1 - p0) * best.1 as f64 / N as f64;
    for _ in 0..40 {
        let c = profile.derivs(v.clamp(p0, p1), 2);
        let (dr, dz) = ((c[0] - axis_o).dot(er) - rho, (c[0] - axis_o).dot(a) - qz);
        let (d1r, d1z) = (c[1].dot(er), c[1].dot(a));
        let (d2r, d2z) = (c[2].dot(er), c[2].dot(a));
        let f = dr * d1r + dz * d1z;
        let df = d1r * d1r + d1z * d1z + dr * d2r + dz * d2z;
        let step = if df.abs() > 1e-300 { -f / df } else { 0.0 };
        let nv = (v + step).clamp(p0, p1);
        let done = (nv - v).abs() < 1e-14 * (1.0 + v.abs());
        v = nv;
        if done {
            break;
        }
    }
    Some((wrap_angle(phi), v))
}

/// Closest point of a screw patch: the profile is sampled; for every sample the 1D problem in the
/// rotation `u` has its minima near `delta + 2 pi k` (the angular term) and near `Delta / rise` (the axial
/// term), so a few candidates per sample are scored by the exact squared distance; the best is polished by a
/// clamped 2D Newton.
fn screw_foot(s: &Surface, q: Vec3) -> (f64, f64) {
    let Surface::Screw { profile, axis_o, axis, rise, urange: (u0, u1), vrange: (v0, v1) } = s else { unreachable!() };
    let a = *axis;
    let e1 = crate::any_perp(a);
    let e2 = a.cross(e1);
    let qw = q - *axis_o;
    let (qz, qx, qy) = (qw.dot(a), qw.dot(e1), qw.dot(e2));
    let rho = qx.hypot(qy);
    let th_q = qy.atan2(qx);
    let n = if matches!(**profile, Curve::Line { .. }) { 16 } else { 48 };
    let mut best = (INF, *u0, *v0);
    let mut cand = |u: f64, r: f64, th: f64, z: f64, v: f64| {
        let u = u.clamp(*u0, *u1);
        let d = r * r + rho * rho - 2.0 * r * rho * (th + u - th_q).cos() + (z + rise * u - qz).powi(2);
        if d < best.0 {
            best = (d, u, v);
        }
    };
    for j in 0..=n {
        let v = v0 + (v1 - v0) * j as f64 / n as f64;
        let w = profile.eval(v) - *axis_o;
        let (z, x, y) = (w.dot(a), w.dot(e1), w.dot(e2));
        let (r, th) = (x.hypot(y), y.atan2(x));
        let delta = th_q - th;
        let k0 = if rise.abs() > 1e-12 { (((qz - z) / rise - delta) / TAU).round() } else { ((0.5 * (u0 + u1) - delta) / TAU).round() };
        for k in -1..=1 {
            cand(delta + TAU * (k0 + k as f64), r, th, z, v);
        }
        if rise.abs() > 1e-12 {
            cand((qz - z) / rise, r, th, z, v);
        }
        cand(*u0, r, th, z, v);
        cand(*u1, r, th, z, v);
    }
    newton_surface(|u, v| s.derivs2(u, v), q, (best.1, best.2), ((*u0, *u1), (*v0, *v1)))
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
            Surface::Screw {
                profile: Box::new(prof),
                axis_o: Vec3::new(0.2, -0.1, 0.0),
                axis: Vec3::Z,
                rise: 0.6,
                urange: (0.0, 6.0),
                vrange: (-1.0, 1.5),
            },
        ]
    }

    #[test]
    fn kinds() {
        let k: Vec<_> = all().iter().map(|s| s.kind()).collect();
        assert_eq!(k, ["plane", "cylinder", "cone", "sphere", "torus", "revolution", "extrusion", "offset", "spline"]);
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
                let tol = if s.kind() == "offset" { 1e-5 } else { 1e-6 };
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

    /// The foot of a point on a multi-turn screw patch is found on the right turn, and a point beyond the
    /// ends clamps to them.
    #[test]
    fn screw_closest_point_picks_the_turn() {
        let prof = Curve::circle_x(Vec3::new(5., 0., 0.), Vec3::Y, Vec3::X, 1.5);
        let s = Surface::Screw {
            profile: Box::new(prof),
            axis_o: Vec3::ZERO,
            axis: Vec3::Z,
            rise: 0.3,
            urange: (0.0, 6.0 * TAU),
            vrange: (0.0, std::f64::consts::PI),
        };
        for &(u, v) in &[(0.4, 0.5), (10.0, 2.0), (25.0, 1.0), (36.0, 2.5)] {
            let p = s.eval(u, v);
            let q = p + s.normal(u, v) * 0.05;
            let (cu, cv) = s.closest_param(q);
            assert!((cu - u).abs() < 1e-7 && (cv - v).abs() < 1e-7, "({u},{v}) -> ({cu},{cv})");
        }
        let past = s.eval(6.0 * TAU, 1.0) + Vec3::Z * 5.0;
        assert!((s.closest_param(past).0 - 6.0 * TAU).abs() < 1.0);
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

    /// Offsetting a quadric gives the same family with the radius adjusted —
    /// the analytic 2nd derivatives must match that surface exactly.
    #[test]
    fn offset_quadrics_exact() {
        let f = frame_from(Vec3::new(1., -2., 0.5), Vec3::new(0.3, 0.4, 1.0), Vec3::X);
        let (u, v) = (0.9, 0.35);
        // plane -> plane shifted along z
        let pl = Surface::Offset { base: Box::new(Surface::Plane { frame: f }), dist: 0.7 };
        assert!(vclose(pl.derivs2(u, v).p, Surface::Plane { frame: f }.eval(u, v) + f.z * 0.7, 1e-13));
        assert_eq!(pl.derivs2(u, v).duu, Vec3::ZERO);
        assert_eq!(pl.derivs2(u, v).duv, Vec3::ZERO);
        assert_eq!(pl.derivs2(u, v).dvv, Vec3::ZERO);
        // cylinder r -> r + d (same parameterisation)
        let cy = Surface::Offset { base: Box::new(Surface::Cylinder { frame: f, r: 2.0 }), dist: 0.7 };
        check_offset_eq(&cy, &Surface::Cylinder { frame: f, r: 2.7 }, u, v, 1e-12);
        // sphere R -> R + d
        let sp = Surface::Offset { base: Box::new(Surface::Sphere { frame: f, r: 3.0 }), dist: 0.7 };
        check_offset_eq(&sp, &Surface::Sphere { frame: f, r: 3.7 }, u, v, 1e-12);
        // cone r, α -> r + d/cos α, v shifted by −d·sin α (the offset slides
        // the generator)
        let k = Surface::Cone { frame: f, r: 2.0, semi_angle: 0.4 };
        let ko = Surface::Offset { base: Box::new(k.clone()), dist: 0.7 };
        let truth = Surface::Cone { frame: f, r: 2.0 + 0.7 / 0.4f64.cos(), semi_angle: 0.4 };
        let dv = -0.7 * 0.4f64.sin();
        let a = ko.derivs2(u, v);
        let b = truth.derivs2(u, v + dv);
        assert!(vclose(a.p, b.p, 1e-12), "cone p");
        assert!(vclose(a.du, b.du, 1e-12) && vclose(a.dv, b.dv, 1e-12), "cone 1st");
        assert!(vclose(a.duu, b.duu, 1e-12) && vclose(a.duv, b.duv, 1e-12) && vclose(a.dvv, b.dvv, 1e-12), "cone 2nd");
        // torus (R, r) -> (R, r + d)
        let to = Surface::Offset { base: Box::new(Surface::Torus { frame: f, major: 5.0, minor: 1.5 }), dist: 0.7 };
        check_offset_eq(&to, &Surface::Torus { frame: f, major: 5.0, minor: 2.2 }, u, v, 1e-12);
        // inward: sphere 3 − 0.7
        let sp2 = Surface::Offset { base: Box::new(Surface::Sphere { frame: f, r: 3.0 }), dist: -0.7 };
        check_offset_eq(&sp2, &Surface::Sphere { frame: f, r: 2.3 }, u, v, 1e-12);
        // nested offsets collapse to the summed distance
        let nested = Surface::Offset {
            base: Box::new(Surface::Offset { base: Box::new(Surface::Sphere { frame: f, r: 3.0 }), dist: 0.4 }),
            dist: 0.3,
        };
        check_offset_eq(&nested, &Surface::Sphere { frame: f, r: 3.7 }, u, v, 1e-12);
    }

    fn check_offset_eq(off: &Surface, truth: &Surface, u: f64, v: f64, tol: f64) {
        let a = off.derivs2(u, v);
        let b = truth.derivs2(u, v);
        assert!(vclose(a.p, b.p, tol), "{} p", truth.kind());
        assert!(vclose(a.du, b.du, tol) && vclose(a.dv, b.dv, tol), "{} 1st", truth.kind());
        assert!(vclose(a.duu, b.duu, tol), "{} duu", truth.kind());
        assert!(vclose(a.duv, b.duv, tol), "{} duv", truth.kind());
        assert!(vclose(a.dvv, b.dvv, tol), "{} dvv", truth.kind());
        assert!(vclose(off.normal(u, v), truth.normal(u, v), tol), "{} normal", truth.kind());
    }

    /// Oracle (kernel.wasm, `surfaceOffset` on an extruded circle R=10 + 20,
    /// then `curvatureAt`): the offset face F3.O at [12,10,0] reports
    /// n=[1,0,0], k1=1/12, k2=0, d1=[0,0,-1], d2=[0,1,0]; flipped (inward,
    /// dist 3) at [7,10,0]: k1=1/7. The analytic second derivatives must
    /// reproduce the offset cylinder's curvature exactly.
    #[test]
    fn offset_curvature_oracle() {
        let axis = frame_from(Vec3::ZERO, Vec3::Y, Vec3::Z);
        let off = Surface::Offset { base: Box::new(Surface::Cylinder { frame: axis, r: 10.0 }), dist: 2.0 };
        // [12,10,0]: u = π/2 (radial +x), v = 10 along the axis
        let k = crate::analysis::surface_curvature(&off.derivs2(FRAC_PI_2, 10.0), false);
        assert!(vclose(k.n, Vec3::X, 1e-14));
        assert!(close(k.k1, 1.0 / 12.0, 1e-14), "{}", k.k1);
        assert!(close(k.k2, 0.0, 1e-14));
        assert!(close(k.mean, 1.0 / 24.0, 1e-14) && close(k.gaussian, 0.0, 1e-14));
        assert!(close(k.d1.z.abs(), 1.0, 1e-14));
        assert!(vclose(k.d2, k.n.cross(k.d1), 1e-14));
        // flipped / inward: R=7, curvature 1/7 (oracle [7,10,0])
        let off2 = Surface::Offset { base: Box::new(Surface::Cylinder { frame: axis, r: 10.0 }), dist: -3.0 };
        let k2 = crate::analysis::surface_curvature(&off2.derivs2(FRAC_PI_2, 10.0), false);
        assert!(vclose(k2.n, Vec3::X, 1e-14));
        assert!(close(k2.k1, 1.0 / 7.0, 1e-14), "{}", k2.k1);
        assert!(close(k2.k2, 0.0, 1e-14));
    }

    /// Non-quadric bases: the analytic second partials agree with finite
    /// differences of the offset points (the old behaviour, now O(h²) only
    /// in the check, not in the implementation).
    #[test]
    fn revolution_foot_matches_the_sphere() {
        // half circle in the x-z plane (x >= 0) revolved about z: a sphere
        let pts: Vec<Vec3> = (0..=24).map(|i| {
            let t = -std::f64::consts::FRAC_PI_2 + std::f64::consts::PI * i as f64 / 24.0;
            Vec3::new(5.0 * t.cos(), 0.0, 5.0 * t.sin())
        }).collect();
        let circ = Curve::BSpline(crate::fit::interpolate(&pts, 3, crate::fit::Param::ChordLength).unwrap());
        let rev = Surface::Revolution { profile: Box::new(circ), axis_o: Vec3::ZERO, axis: Vec3::Z };
        let sph = Surface::Sphere { frame: crate::Frame::identity(), r: 5.0 };
        for q in [Vec3::new(8.0, 3.0, 2.0), Vec3::new(-1.0, 2.5, 0.7), Vec3::new(0.3, -2.0, -3.1), Vec3::new(2.0, 2.0, 7.0)] {
            let (_, _, p) = rev.closest_point(q);
            let (_, _, ps) = sph.closest_point(q);
            assert!(p.dist(ps) < 2e-4, "{q:?}: {p:?} vs {ps:?}");
        }
    }

    #[test]
    fn offset_nonquadric_fd() {
        let prof = Curve::circle_x(Vec3::new(5., 0., 0.), Vec3::Y, Vec3::X, 1.5);
        let ex = Surface::Extrusion {
            profile: Box::new(Curve::ellipse(Vec3::ZERO, Vec3::Z, Vec3::X, 3.0, 1.0)),
            dir: Vec3::new(0.2, 0.0, 2.0),
        };
        let cases = [
            (
                Surface::Offset {
                    base: Box::new(Surface::Revolution { profile: Box::new(prof), axis_o: Vec3::ZERO, axis: Vec3::Z }),
                    dist: 0.4,
                },
                [(0.7, 0.3), (2.0, -0.4), (4.1, 0.8)],
            ),
            (Surface::Offset { base: Box::new(ex.clone()), dist: 0.3 }, [(0.7, 0.3), (2.0, -0.4), (4.1, 0.8)]),
            // B-spline patch: stay inside its [0,1]² domain
            (
                Surface::Offset { base: Box::new(Surface::BSpline(Box::new(spline_patch()))), dist: -0.2 },
                [(0.3, 0.4), (0.7, 0.25), (0.5, 0.8)],
            ),
            // nested offsets collapse to dist 0.45 over the extrusion
            (Surface::Offset { base: Box::new(Surface::Offset { base: Box::new(ex), dist: 0.3 }), dist: 0.15 },
             [(0.7, 0.3), (2.0, -0.4), (4.1, 0.8)]),
        ];
        let h = 1e-4;
        for (s, pts) in &cases {
            for &(u, v) in pts {
                let d = s.derivs2(u, v);
                assert!(vclose(d.p, s.eval(u, v), 1e-13), "{} p", s.kind());
                let fuu = (s.eval(u + h, v) - d.p * 2.0 + s.eval(u - h, v)) / (h * h);
                let fvv = (s.eval(u, v + h) - d.p * 2.0 + s.eval(u, v - h)) / (h * h);
                let fuv = (s.eval(u + h, v + h) - s.eval(u + h, v - h) - s.eval(u - h, v + h)
                    + s.eval(u - h, v - h))
                    / (4.0 * h * h);
                assert!(vclose(d.duu, fuu, 1e-5), "{} duu {:?} vs {:?}", s.kind(), d.duu, fuu);
                assert!(vclose(d.dvv, fvv, 1e-5), "{} dvv", s.kind());
                assert!(vclose(d.duv, fuv, 1e-5), "{} duv", s.kind());
                // the normal field differentiates exactly: n_u from FD of the
                // analytic normal matches the implied one
                let nu_fd = (s.normal(u + h, v) - s.normal(u - h, v)) / (2.0 * h);
                assert!(vclose((d.du - base_du(s, u, v)) / offset_dist(s), nu_fd, 1e-6), "{} nu", s.kind());
            }
        }
    }

    fn offset_dist(s: &Surface) -> f64 {
        match s {
            Surface::Offset { base, dist } => {
                let (_, d0) = offset_collapse(base);
                *dist + d0
            }
            _ => 0.0,
        }
    }
    fn base_du(s: &Surface, u: f64, v: f64) -> Vec3 {
        match s {
            Surface::Offset { base, .. } => offset_collapse(base).0.derivs2(u, v).du,
            _ => Vec3::ZERO,
        }
    }

    fn spline_patch() -> crate::nurbs::bsurf::BSplineSurface {
        let (nu, nv) = (4, 4);
        let ctrl: Vec<Vec3> = (0..nu * nv)
            .map(|k| {
                let (i, j) = (k / nv, k % nv);
                Vec3::new(i as f64 * 1.5, j as f64 * 1.2, ((i * 3 + j * 5) % 7) as f64 * 0.35 - 1.0)
            })
            .collect();
        crate::nurbs::bsurf::BSplineSurface::new(
            3,
            3,
            crate::nurbs::basis::clamped_uniform(nu, 3),
            crate::nurbs::basis::clamped_uniform(nv, 3),
            nu,
            nv,
            ctrl,
            None,
        )
        .unwrap()
    }
}
