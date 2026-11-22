//! Procedural surfaces: a sweep (a profile carried along a path) and a
//! rolling-ball blend (the surface a ball of given radius sweeps while it rolls
//! in contact with two support surfaces), kept as their definitions and
//! converted to a B-spline on demand with a measured deviation bound.
//!
//! After solidcore's `surface/procedural_sweep` and `surface/procedural_blend`
//! (tangency-labs/solidcore, Apache-2.0, see THIRD_PARTY.md, and its
//! `docs/definitions-and-certificates.md` / `docs/procedural-sweep.md`): the
//! definition S answers every evaluation; the B-spline B serves export,
//! tessellation and the operations that need one form, and carries the bound
//! of its distance from S. Here the bound is measured on samples (the fit's
//! error at its samples and their midpoints, and a closest-point check on a
//! denser grid), not enclosed by interval arithmetic as upstream: a sampled
//! bound, named so.
//!
//! [`SweepSurface`]: S(u, v) = path(u) + M(u)·L(v), with L(v) the profile in the
//! moving frame M at the start of the path (so S(u0, ·) is the profile itself)
//! and M given by a [`SweepLaw`]: a pure translation, or the frame of the path
//! tangent and a fixed up direction (x = T, y = unit(up × T), z = x × y).
//! Exact to first order in u and to every order in v; the second u
//! derivatives are central differences of the exact first.
//!
//! [`RollingBall`]: at spine parameter u the ball centre c lies on both offset
//! surfaces O_i = S_i + d_i·n_i (d_i = side_i·r(u), n_i the unit Su × Sv of
//! support i, r(u) a linear radius law) and in the plane through spine(u)
//! square to the spine; the four foot parameters solve
//! `O1 − O2 = 0, (O1 − spine(u))·T(u) = 0` by Newton. The section across
//! (v in [0, 1]) is the circular arc about c from the foot on S1 to the foot on
//! S2, exact. v derivatives are closed form; u derivatives are central
//! differences of the exact station solve.

use crate::approx::{approximate_surface_fn, fn_surface_deviation};
use crate::curve::{push_vec3, Curve};
use crate::nurbs::bsurf::BSplineSurface;

use crate::surface::{SurfDerivs, Surface};
use crate::Vec3;

/// How the profile's frame moves along a sweep path.
#[derive(Debug, Clone, PartialEq)]
pub enum SweepLaw {
    /// The profile is translated (M = identity): a generalised extrusion.
    Translate,
    /// x = path tangent, y = unit(up × x), z = x × y.
    Up(Vec3),
}

/// A profile swept along a path (see the module notes).
#[derive(Debug, Clone, PartialEq)]
pub struct SweepSurface {
    pub profile: Curve,
    pub path: Curve,
    pub law: SweepLaw,
    /// Path parameter range (u).
    pub urange: (f64, f64),
    /// Profile parameter range (v).
    pub vrange: (f64, f64),
}

/// Columns of the moving frame and their u derivatives.
type FrameJet = ([Vec3; 3], [Vec3; 3]);

impl SweepSurface {
    /// Sweep `profile` (over `vrange`) along `path` (over `urange`). `Err` when a
    /// range is not finite and increasing, or the up direction is along the
    /// path tangent somewhere (sampled).
    pub fn new(profile: Curve, path: Curve, law: SweepLaw, urange: (f64, f64), vrange: (f64, f64)) -> Result<SweepSurface, String> {
        for (n, r) in [("path", urange), ("profile", vrange)] {
            if !(r.0.is_finite() && r.1.is_finite() && r.1 > r.0) {
                return Err(format!("sweep: {n} range [{}, {}] is not finite and increasing", r.0, r.1));
            }
        }
        if let SweepLaw::Up(up) = &law {
            if !(up.len() > 1e-12) {
                return Err("sweep: zero up direction".into());
            }
            for i in 0..=32 {
                let u = urange.0 + (urange.1 - urange.0) * i as f64 / 32.0;
                let t = path.deriv(u).normalize();
                if up.normalize().cross(t).len() < 1e-6 {
                    return Err(format!("sweep: the up direction is along the path tangent at u = {u}"));
                }
            }
        }
        if profile.nesting().max(path.nesting()) >= crate::procedural::MAX_NESTING {
            return Err("sweep: nested too deep".into());
        }
        let law = match law {
            SweepLaw::Up(u) => SweepLaw::Up(u.normalize()),
            l => l,
        };
        Ok(SweepSurface { profile, path, law, urange, vrange })
    }

    fn frame(&self, u: f64) -> FrameJet {
        match &self.law {
            SweepLaw::Translate => ([Vec3::X, Vec3::Y, Vec3::Z], [Vec3::ZERO; 3]),
            SweepLaw::Up(up) => {
                let p = self.path.derivs2_arr(u);
                let s = p[1].len().max(1e-300);
                let t = p[1] / s;
                let t1 = (p[2] - t * t.dot(p[2])) / s;
                let y0 = up.cross(t);
                let l = y0.len().max(1e-300);
                let y = y0 / l;
                let y0d = up.cross(t1);
                let y1 = (y0d - y * y.dot(y0d)) / l;
                let z = t.cross(y);
                let z1 = t1.cross(y) + t.cross(y1);
                ([t, y, z], [t1, y1, z1])
            }
        }
    }

    /// Profile point `q` in the start frame's coordinates.
    fn local(&self, q: Vec3, m0: &[Vec3; 3]) -> Vec3 {
        Vec3::new(q.dot(m0[0]), q.dot(m0[1]), q.dot(m0[2]))
    }

    fn first(&self, u: f64, v: f64) -> (Vec3, Vec3, Vec3, Vec3, Vec3) {
        let (m0, _) = self.frame(self.urange.0);
        let o0 = self.path.eval(self.urange.0);
        let c = self.profile.derivs2_arr(v);
        let (l, l1, l2) = (self.local(c[0] - o0, &m0), self.local(c[1], &m0), self.local(c[2], &m0));
        let (m, m1) = self.frame(u);
        let p = self.path.derivs1_arr(u);
        let ap = |m: &[Vec3; 3], x: Vec3| m[0] * x.x + m[1] * x.y + m[2] * x.z;
        (p[0] + ap(&m, l), p[1] + ap(&m1, l), ap(&m, l1), ap(&m, l2), ap(&m1, l1))
    }

    pub fn eval(&self, u: f64, v: f64) -> Vec3 {
        self.first(u, v).0
    }

    pub fn derivs2(&self, u: f64, v: f64) -> SurfDerivs {
        let (p, du, dv, dvv, duv) = self.first(u, v);
        let h = 1e-4 * (1.0 + u.abs()).max(self.urange.1 - self.urange.0);
        let (a, b) = (self.first(u + h, v).1, self.first(u - h, v).1);
        SurfDerivs { p, du, dv, duu: (a - b) / (2.0 * h), duv, dvv }
    }
}

/// The rolling-ball blend between two supports (see the module notes).
#[derive(Debug, Clone, PartialEq)]
pub struct RollingBall {
    pub s1: Surface,
    pub s2: Surface,
    /// +1: the ball on the side of support i's natural normal Su × Sv, −1: against it.
    pub side1: f64,
    pub side2: f64,
    /// Ball radius at the start and the end of `urange` (linear law between).
    pub radius: (f64, f64),
    /// The spine: the ball centre at u lies in the plane through spine(u)
    /// square to the spine's tangent.
    pub spine: Curve,
    pub urange: (f64, f64),
}

/// One cross-section of a rolling-ball blend.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Station {
    /// Ball centre.
    pub centre: Vec3,
    /// Foot parameters on the first and second support.
    pub foot1: (f64, f64),
    pub foot2: (f64, f64),
    /// Foot points.
    pub p1: Vec3,
    pub p2: Vec3,
    /// Largest residual of the defining equations at the solution.
    pub residual: f64,
}

/// Point of a support offset and its first partials: (O, O_a, O_b).
fn offset_jet(s: &Surface, a: f64, b: f64, d: f64) -> (Vec3, Vec3, Vec3, Vec3) {
    let k = s.derivs2(a, b);
    let w0 = k.du.cross(k.dv);
    let w = w0.len();
    if !(w > 1e-300) {
        let n = s.normal(a, b);
        return (k.p + n * d, k.du, k.dv, k.p);
    }
    let n = w0 / w;
    let wu = k.duu.cross(k.dv) + k.du.cross(k.duv);
    let wv = k.duv.cross(k.dv) + k.du.cross(k.dvv);
    let nu = (wu - n * n.dot(wu)) / w;
    let nv = (wv - n * n.dot(wv)) / w;
    (k.p + n * d, k.du + nu * d, k.dv + nv * d, k.p)
}

/// Gaussian elimination with partial pivoting; None when singular.
fn solve4(mut a: [[f64; 4]; 4], mut r: [f64; 4]) -> Option<[f64; 4]> {
    let scale = a.iter().flatten().fold(0.0f64, |m, x| m.max(x.abs()));
    if !(scale > 0.0) {
        return None;
    }
    for c in 0..4 {
        let p = (c..4).max_by(|&i, &j| a[i][c].abs().total_cmp(&a[j][c].abs()))?;
        if !(a[p][c].abs() > 1e-14 * scale) {
            return None;
        }
        a.swap(c, p);
        r.swap(c, p);
        for i in c + 1..4 {
            let f = a[i][c] / a[c][c];
            for j in c..4 {
                a[i][j] -= f * a[c][j];
            }
            r[i] -= f * r[c];
        }
    }
    let mut x = [0.0; 4];
    for i in (0..4).rev() {
        let mut s = r[i];
        for j in i + 1..4 {
            s -= a[i][j] * x[j];
        }
        x[i] = s / a[i][i];
    }
    Some(x)
}

impl RollingBall {
    /// The blend of `s1` and `s2` by a ball of radius `radius.0` at the start of
    /// `urange` to `radius.1` at its end, rolling along `spine`. `Err` when a
    /// radius is not positive, the range is not finite and increasing, a side is
    /// not ±1, or the station at the start or the end of the spine has no solution.
    pub fn new(s1: Surface, s2: Surface, side1: f64, side2: f64, radius: (f64, f64), spine: Curve, urange: (f64, f64)) -> Result<RollingBall, String> {
        if !(radius.0 > 0.0 && radius.1 > 0.0 && radius.0.is_finite() && radius.1.is_finite()) {
            return Err("rolling-ball blend: the radius must be positive".into());
        }
        if !(urange.0.is_finite() && urange.1.is_finite() && urange.1 > urange.0) {
            return Err("rolling-ball blend: the spine range is not finite and increasing".into());
        }
        if side1.abs() != 1.0 || side2.abs() != 1.0 {
            return Err("rolling-ball blend: sides are +1 or -1".into());
        }
        let n = crate::procedural::MAX_NESTING;
        if s1.nesting() >= n || s2.nesting() >= n || spine.nesting() >= n {
            return Err("rolling-ball blend: nested too deep".into());
        }
        let rb = RollingBall { s1, s2, side1, side2, radius, spine, urange };
        for u in [urange.0, 0.5 * (urange.0 + urange.1), urange.1] {
            let st = rb.station(u);
            let r = rb.radius_at(u);
            if !(st.residual <= 1e-9 * (1.0 + r + st.centre.len())) {
                return Err(format!("rolling-ball blend: no ball of radius {r} touches both supports at u = {u} (residual {:.3e})", st.residual));
            }
        }
        Ok(rb)
    }

    pub fn radius_at(&self, u: f64) -> f64 {
        let (u0, u1) = self.urange;
        let f = (u - u0) / (u1 - u0);
        self.radius.0 + (self.radius.1 - self.radius.0) * f
    }

    /// The cross-section at spine parameter `u`: Newton from the supports' feet
    /// of the spine point.
    pub fn station(&self, u: f64) -> Station {
        let sp = self.spine.eval(u);
        let f1 = self.s1.closest_param(sp);
        let f2 = self.s2.closest_param(sp);
        self.station_from(u, (f1, f2))
    }

    /// The cross-section at `u` by Newton from the foot parameters `seed`.
    pub fn station_from(&self, u: f64, seed: ((f64, f64), (f64, f64))) -> Station {
        let sp = self.spine.derivs1_arr(u);
        let t = sp[1].normalize();
        let r = self.radius_at(u);
        let (d1, d2) = (self.side1 * r, self.side2 * r);
        let (mut x1, mut x2) = seed;
        let (dom1, dom2) = (self.s1.domain(), self.s2.domain());
        let mut res = f64::INFINITY;
        for _ in 0..60 {
            let (o1, o1a, o1b, _) = offset_jet(&self.s1, x1.0, x1.1, d1);
            let (o2, o2a, o2b, _) = offset_jet(&self.s2, x2.0, x2.1, d2);
            let f = o1 - o2;
            let g = (o1 - sp[0]).dot(t);
            res = f.len().max(g.abs());
            let a = [
                [o1a.x, o1b.x, -o2a.x, -o2b.x],
                [o1a.y, o1b.y, -o2a.y, -o2b.y],
                [o1a.z, o1b.z, -o2a.z, -o2b.z],
                [o1a.dot(t), o1b.dot(t), 0.0, 0.0],
            ];
            let Some(dx) = solve4(a, [-f.x, -f.y, -f.z, -g]) else { break };
            let cl = |v: f64, r: (f64, f64)| v.clamp(r.0, r.1);
            let n1 = (cl(x1.0 + dx[0], dom1.0), cl(x1.1 + dx[1], dom1.1));
            let n2 = (cl(x2.0 + dx[2], dom2.0), cl(x2.1 + dx[3], dom2.1));
            let step = dx.iter().fold(0.0f64, |m, v| m.max(v.abs()));
            x1 = n1;
            x2 = n2;
            if step <= 1e-15 * (1.0 + x1.0.abs().max(x1.1.abs()).max(x2.0.abs()).max(x2.1.abs())) {
                break;
            }
        }
        let (o1, _, _, p1) = offset_jet(&self.s1, x1.0, x1.1, d1);
        let (o2, _, _, p2) = offset_jet(&self.s2, x2.0, x2.1, d2);
        let g = (o1 - sp[0]).dot(t);
        res = res.min((o1 - o2).len().max(g.abs()));
        Station { centre: (o1 + o2) * 0.5, foot1: x1, foot2: x2, p1, p2, residual: res }
    }

    /// (point, ∂/∂v, ∂²/∂v²) of the section arc of station `st` at `v`.
    fn arc(st: &Station, v: f64) -> (Vec3, Vec3, Vec3) {
        let c = st.centre;
        let (e1, e2) = (st.p1 - c, st.p2 - c);
        let th = e1.cross(e2).len().atan2(e1.dot(e2));
        if th.abs() < 1e-9 {
            let p = e1 + (e2 - e1) * v;
            return (c + p, e2 - e1, Vec3::ZERO);
        }
        let s = th.sin();
        let p = (e1 * ((1.0 - v) * th).sin() + e2 * (v * th).sin()) / s;
        let d = (e1 * (-((1.0 - v) * th).cos()) + e2 * (v * th).cos()) * (th / s);
        (c + p, d, -p * (th * th))
    }

    pub fn eval(&self, u: f64, v: f64) -> Vec3 {
        Self::arc(&self.station(u), v).0
    }

    pub fn derivs2(&self, u: f64, v: f64) -> SurfDerivs {
        let st = self.station(u);
        let (p, dv, dvv) = Self::arc(&st, v);
        let span = self.urange.1 - self.urange.0;
        let seed = (st.foot1, st.foot2);
        let at = |w: f64| Self::arc(&self.station_from(w, seed), v);
        let h = 1e-6 * span.max(1e-3);
        let (a, b) = (at(u + h), at(u - h));
        let h2 = 1e-4 * span.max(1e-3);
        let (a2, b2) = (at(u + h2).0, at(u - h2).0);
        SurfDerivs {
            p,
            du: (a.0 - b.0) / (2.0 * h),
            dv,
            duu: (a2 - p * 2.0 + b2) / (h2 * h2),
            duv: (a.1 - b.1) / (2.0 * h),
            dvv,
        }
    }
}

/// Exact content key of a procedural surface (false for the other kinds).
pub(crate) fn push_key(s: &Surface, out: &mut Vec<u64>) -> bool {
    match s {
        Surface::Sweep(w) => {
            out.push(26);
            w.profile.push_key(out);
            w.path.push_key(out);
            match &w.law {
                SweepLaw::Translate => out.push(0),
                SweepLaw::Up(u) => {
                    out.push(1);
                    push_vec3(out, *u);
                }
            }
            out.extend([w.urange.0.to_bits(), w.urange.1.to_bits(), w.vrange.0.to_bits(), w.vrange.1.to_bits()]);
        }
        Surface::Blend(b) => {
            out.push(27);
            b.s1.push_key(out);
            b.s2.push_key(out);
            out.extend([b.side1.to_bits(), b.side2.to_bits(), b.radius.0.to_bits(), b.radius.1.to_bits()]);
            b.spine.push_key(out);
            out.extend([b.urange.0.to_bits(), b.urange.1.to_bits()]);
        }
        _ => return false,
    }
    true
}

/// Default tolerance of the on-demand B-spline of a surface patch: 1e-6 of
/// its size (at least 1e-7).
pub fn default_tol(s: &Surface, ur: (f64, f64), vr: (f64, f64)) -> f64 {
    let mut lo = Vec3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY);
    let mut hi = -lo;
    for i in 0..=4 {
        for j in 0..=4 {
            let p = s.eval(ur.0 + (ur.1 - ur.0) * i as f64 / 4.0, vr.0 + (vr.1 - vr.0) * j as f64 / 4.0);
            lo = Vec3::new(lo.x.min(p.x), lo.y.min(p.y), lo.z.min(p.z));
            hi = Vec3::new(hi.x.max(p.x), hi.y.max(p.y), hi.z.max(p.z));
        }
    }
    (1e-6 * lo.dist(hi)).max(1e-7)
}

/// A bicubic B-spline of `s` over `ur × vr` within `tol` at the same parameters,
/// and its measured bound: the fit's error at its samples and their midpoints,
/// and an independent closest-point check of the definition against the fit
/// on a 48 × 48 grid, whichever is larger (a sampled bound). `Err` when the
/// fit does not reach `tol`.
pub fn certified_nurbs(s: &Surface, ur: (f64, f64), vr: (f64, f64), tol: f64) -> Result<(BSplineSurface, f64), String> {
    for r in [ur, vr] {
        if !(r.0.is_finite() && r.1.is_finite() && r.1 > r.0) {
            return Err(format!("certified_nurbs: [{}, {}] is not a finite increasing interval", r.0, r.1));
        }
    }
    let f = |u: f64, v: f64| s.eval(u, v);
    let (b, err) = approximate_surface_fn(&f, ur, vr, (3, 3), tol, &[], &[])?;
    let check = fn_surface_deviation(&f, ur, vr, &b, 48);
    let bound = err.max(check);
    if bound > tol {
        return Err(format!("certified_nurbs: measured deviation {bound:.3e} exceeds the tolerance {tol:.3e}"));
    }
    Ok((b, bound))
}

/// Closest parameters on a procedural surface: a grid scan of the domain and
/// Newton from the nearest sample.
pub(crate) fn closest(s: &Surface, q: Vec3) -> (f64, f64) {
    let ((u0, u1), (v0, v1)) = s.domain();
    let (nu, nv) = (16, 8);
    let mut best = (f64::INFINITY, u0, v0);
    for i in 0..=nu {
        let u = u0 + (u1 - u0) * i as f64 / nu as f64;
        for j in 0..=nv {
            let v = v0 + (v1 - v0) * j as f64 / nv as f64;
            let d = s.eval(u, v).dist(q);
            if d < best.0 {
                best = (d, u, v);
            }
        }
    }
    crate::surface::newton_surface(|u, v| s.derivs2(u, v), q, (best.1, best.2), ((u0, u1), (v0, v1)))
}

pub(crate) fn transform(s: &Surface, f: &dyn Fn(Vec3) -> Vec3, g: &dyn Fn(Vec3) -> Vec3) -> Option<Surface> {
    use crate::procedural::transform_keep_param as tk;
    let sc = g(Vec3::X).len();
    let det = g(Vec3::X).dot(g(Vec3::Y).cross(g(Vec3::Z)));
    Some(match s {
        Surface::Sweep(w) => Surface::Sweep(Box::new(SweepSurface {
            profile: tk(&w.profile, f, g, w.vrange),
            path: tk(&w.path, f, g, w.urange),
            law: match &w.law {
                SweepLaw::Translate => SweepLaw::Translate,
                SweepLaw::Up(u) => SweepLaw::Up(g(*u) / sc),
            },
            urange: w.urange,
            vrange: w.vrange,
        })),
        // a mirror turns the supports' natural normals (Su × Sv) against the mapped ones: the sides follow
        Surface::Blend(b) => Surface::Blend(Box::new(RollingBall {
            s1: b.s1.transform_dyn(f, g),
            s2: b.s2.transform_dyn(f, g),
            side1: b.side1 * det.signum(),
            side2: b.side2 * det.signum(),
            radius: (b.radius.0 * sc, b.radius.1 * sc),
            spine: tk(&b.spine, f, g, b.urange),
            urange: b.urange,
        })),
        _ => return None,
    })
}


#[cfg(test)]
mod tests {
    use super::*;
    use crate::testutil::*;
    use crate::{frame_from, Frame};
    use std::f64::consts::{FRAC_PI_2, TAU};

    fn plane(o: Vec3, n: Vec3) -> Surface {
        Surface::plane(o, n)
    }
    /// Ball of radius 1.5 in the corner of the planes z = 0 and x = 0, spine the y axis.
    fn corner_blend() -> Surface {
        let rb = RollingBall::new(plane(Vec3::ZERO, Vec3::Z), plane(Vec3::ZERO, Vec3::X), 1.0, 1.0, (1.5, 1.5), Curve::line(Vec3::ZERO, Vec3::Y), (-2.0, 3.0)).unwrap();
        Surface::Blend(Box::new(rb))
    }
    /// Ball of radius 1 on the plane z = 0 outside the cylinder of radius 5 about z: the fillet ring.
    fn ring_blend() -> Surface {
        let cyl = Surface::Cylinder { frame: Frame::identity(), r: 5.0 };
        let spine = Curve::circle(Vec3::ZERO, Vec3::Z, 5.0);
        let rb = RollingBall::new(plane(Vec3::ZERO, Vec3::Z), cyl, 1.0, 1.0, (1.0, 1.0), spine, (0.2, 2.5)).unwrap();
        Surface::Blend(Box::new(rb))
    }
    /// A small circle carried round a circle of radius 6 with the up law: a torus.
    fn torus_sweep() -> Surface {
        let path = Curve::circle_x(Vec3::ZERO, Vec3::Z, Vec3::X, 6.0);
        // the profile at the path start (6, 0, 0), in the plane square to the tangent (y)
        let prof = Curve::circle_x(Vec3::new(6., 0., 0.), Vec3::Y, Vec3::X, 1.5);
        Surface::Sweep(Box::new(SweepSurface::new(prof, path, SweepLaw::Up(Vec3::Z), (0.0, 4.0), (0.0, TAU)).unwrap()))
    }
    fn spline_sweep() -> Surface {
        let pts = vec![Vec3::new(0., 0., 0.), Vec3::new(1., 2., 1.), Vec3::new(3., 2.5, 2.), Vec3::new(5., 1., 4.)];
        let path = Curve::BSpline(crate::fit::interpolate(&pts, 3, crate::fit::Param::ChordLength).unwrap());
        let (a, b) = path.range();
        let prof = Curve::ellipse(Vec3::ZERO, path.deriv(a), Vec3::Z, 0.6, 0.3);
        Surface::Sweep(Box::new(SweepSurface::new(prof, path, SweepLaw::Up(Vec3::new(0.1, 0.0, 1.0)), (a, b), (0.0, TAU)).unwrap()))
    }
    fn all() -> Vec<Surface> {
        let line = Curve::line(Vec3::new(0., 0., 0.), Vec3::new(0., 0., 1.));
        let prof = Curve::circle(Vec3::ZERO, Vec3::Z, 2.0);
        let ext = Surface::Sweep(Box::new(SweepSurface::new(prof, line, SweepLaw::Translate, (0.0, 3.0), (0.0, TAU)).unwrap()));
        vec![corner_blend(), ring_blend(), torus_sweep(), spline_sweep(), ext]
    }
    fn inner(s: &Surface) -> ((f64, f64), (f64, f64)) {
        let ((u0, u1), (v0, v1)) = s.domain();
        let (du, dv) = (0.05 * (u1 - u0), 0.05 * (v1 - v0));
        ((u0 + du, u1 - du), (v0 + dv, v1 - dv))
    }

    #[test]
    fn corner_blend_is_a_quarter_cylinder() {
        let s = corner_blend();
        assert_eq!(s.kind(), "spline");
        assert!(s.is_procedural());
        for &u in &[-1.5, 0.0, 0.7, 2.5] {
            let c = Vec3::new(1.5, u, 1.5);
            assert!(vclose(s.eval(u, 0.0), Vec3::new(1.5, u, 0.0), 1e-12), "{:?}", s.eval(u, 0.0));
            assert!(vclose(s.eval(u, 1.0), Vec3::new(0.0, u, 1.5), 1e-12));
            for i in 0..=8 {
                let v = i as f64 / 8.0;
                let p = s.eval(u, v);
                assert!(close(p.dist(c), 1.5, 1e-12) && close(p.y, u, 1e-12));
                // the arc runs at constant angular speed π/2 per unit v
                assert!(close(s.derivs2(u, v).dv.len(), 1.5 * FRAC_PI_2, 1e-10));
            }
        }
        let Surface::Blend(b) = &s else { unreachable!() };
        let st = b.station(0.4);
        assert!(vclose(st.centre, Vec3::new(1.5, 0.4, 1.5), 1e-12) && st.residual < 1e-12);
        // no ball of radius 1 fits on the wrong side of the second plane: refused with the reason
        assert!(RollingBall::new(plane(Vec3::ZERO, Vec3::Z), plane(Vec3::ZERO, Vec3::Z), 1.0, -1.0, (1.0, 1.0), Curve::line(Vec3::ZERO, Vec3::Y), (0.0, 1.0)).is_err());
        assert!(RollingBall::new(plane(Vec3::ZERO, Vec3::Z), plane(Vec3::ZERO, Vec3::X), 1.0, 1.0, (0.0, 1.0), Curve::line(Vec3::ZERO, Vec3::Y), (0.0, 1.0)).is_err());
    }

    #[test]
    fn ring_blend_lies_on_its_torus() {
        let s = ring_blend();
        // centre circle radius 6 at height 1, tube radius 1
        for i in 0..=6 {
            let u = 0.2 + 2.3 * i as f64 / 6.0;
            for j in 0..=6 {
                let v = j as f64 / 6.0;
                let p = s.eval(u, v);
                let rho = (p.x * p.x + p.y * p.y).sqrt();
                assert!(close(((rho - 6.0).powi(2) + (p.z - 1.0).powi(2)).sqrt(), 1.0, 1e-11), "u={u} v={v} {p:?}");
                assert!(rho <= 6.0 + 1e-12 && p.z <= 1.0 + 1e-12);
            }
            // ends on the plane and on the cylinder
            assert!(s.eval(u, 0.0).z.abs() < 1e-12);
            let q = s.eval(u, 1.0);
            assert!(close((q.x * q.x + q.y * q.y).sqrt(), 5.0, 1e-12));
        }
    }

    #[test]
    fn torus_sweep_lies_on_its_torus() {
        let s = torus_sweep();
        let t = Surface::Torus { frame: Frame::identity(), major: 6.0, minor: 1.5 };
        for i in 0..=8 {
            for j in 0..=8 {
                let (u, v) = (4.0 * i as f64 / 8.0, TAU * j as f64 / 8.0);
                let p = s.eval(u, v);
                assert!(t.distance(p) < 1e-12, "{p:?}");
            }
        }
        // the start section is the profile itself
        let Surface::Sweep(w) = &s else { unreachable!() };
        for j in 0..8 {
            let v = TAU * j as f64 / 8.0;
            assert!(vclose(s.eval(0.0, v), w.profile.eval(v), 1e-14));
        }
        // a translation sweep of a circle along a line is the cylinder
        let ext = &all()[4];
        let cyl = Surface::Cylinder { frame: frame_from(Vec3::ZERO, Vec3::Z, crate::any_perp(Vec3::Z)), r: 2.0 };
        for &(u, v) in &[(0.0, 0.0), (1.0, 2.0), (2.5, 5.0)] {
            assert!(cyl.distance(ext.eval(u, v)) < 1e-13);
        }
    }

    #[test]
    fn derivatives_match_finite_differences() {
        for s in all() {
            let ((u0, u1), (v0, v1)) = inner(&s);
            for i in 0..=3 {
                for j in 0..=3 {
                    let (u, v) = (u0 + (u1 - u0) * i as f64 / 3.0, v0 + (v1 - v0) * j as f64 / 3.0);
                    let d = s.derivs2(u, v);
                    let (hu, hv) = (1e-4 * (u1 - u0), 1e-4 * (v1 - v0));
                    let fu = (s.eval(u + hu, v) - s.eval(u - hu, v)) / (2.0 * hu);
                    let fv = (s.eval(u, v + hv) - s.eval(u, v - hv)) / (2.0 * hv);
                    let fuu = (s.derivs2(u + hu, v).du - s.derivs2(u - hu, v).du) / (2.0 * hu);
                    let fuv = (s.derivs2(u, v + hv).du - s.derivs2(u, v - hv).du) / (2.0 * hv);
                    let fvv = (s.derivs2(u, v + hv).dv - s.derivs2(u, v - hv).dv) / (2.0 * hv);
                    let sc = 1.0 + d.du.len() + d.dv.len();
                    assert!((d.du - fu).len() < 1e-6 * sc, "du {u} {v}: {:?} {:?}", d.du, fu);
                    assert!((d.dv - fv).len() < 1e-6 * sc, "dv");
                    assert!((d.duu - fuu).len() < 1e-3 * sc, "duu {:?} {:?}", d.duu, fuu);
                    assert!((d.duv - fuv).len() < 1e-4 * sc, "duv {:?} {:?}", d.duv, fuv);
                    assert!((d.dvv - fvv).len() < 1e-5 * sc, "dvv");
                    assert!(vclose(d.p, s.eval(u, v), 1e-13));
                }
            }
        }
    }

    #[test]
    fn certified_nurbs_within_bound() {
        for s in all() {
            let (ur, vr) = s.domain();
            let tol = 1e-5;
            let (b, bound) = s.to_nurbs_within(ur, vr, tol).unwrap();
            assert!(bound <= tol);
            for i in 0..=13 {
                for j in 0..=11 {
                    let (u, v) = (ur.0 + (ur.1 - ur.0) * i as f64 / 13.0, vr.0 + (vr.1 - vr.0) * j as f64 / 11.0);
                    assert!(b.eval(u, v).dist(s.eval(u, v)) <= 2.0 * tol);
                }
            }
            // surface_to_nurbs takes the certified fit
            assert!(crate::convert::surface_to_nurbs(&s, ur, vr).is_ok());
        }
    }

    #[test]
    fn transform_keeps_the_parameterisation() {
        let rot = |p: Vec3| Vec3::new(p.y, -p.x, p.z);
        let mir = |p: Vec3| Vec3::new(-p.x, p.y, p.z);
        let f1 = |p: Vec3| rot(p) + Vec3::new(1., 2., 3.);
        let f2 = |p: Vec3| mir(p) + Vec3::new(0.5, 0., 0.);
        let maps: [(&dyn Fn(Vec3) -> Vec3, &dyn Fn(Vec3) -> Vec3); 2] = [(&f1, &rot), (&f2, &mir)];
        for (f, g) in maps {
            for s in all() {
                let m = s.transform_dyn(f, g);
                let ((u0, u1), (v0, v1)) = inner(&s);
                for &(a, b) in &[(0.0, 0.0), (0.3, 0.8), (1.0, 0.5)] {
                    let (u, v) = (u0 + (u1 - u0) * a, v0 + (v1 - v0) * b);
                    assert!(vclose(m.eval(u, v), f(s.eval(u, v)), 1e-9), "{:?} {:?}", m.eval(u, v), f(s.eval(u, v)));
                }
            }
        }
    }

    #[test]
    fn closest_param_recovers_points() {
        for s in all() {
            let ((u0, u1), (v0, v1)) = inner(&s);
            for &(a, b) in &[(0.2, 0.3), (0.5, 0.5), (0.9, 0.7)] {
                let p = s.eval(u0 + (u1 - u0) * a, v0 + (v1 - v0) * b);
                let (u, v) = s.closest_param(p);
                assert!(s.eval(u, v).dist(p) < 1e-8, "{:?}", s.eval(u, v));
            }
        }
    }

    #[test]
    fn keys_tell_kinds_apart() {
        let mut keys: Vec<Vec<u64>> = Vec::new();
        for s in all() {
            let mut k = Vec::new();
            s.push_key(&mut k);
            assert!(!keys.contains(&k));
            keys.push(k);
        }
        assert!(SweepSurface::new(Curve::line(Vec3::ZERO, Vec3::X), Curve::line(Vec3::ZERO, Vec3::Z), SweepLaw::Up(Vec3::Z), (0.0, 1.0), (0.0, 1.0)).is_err());
    }

    /// The length of an edge on a blend far from the origin used to bisect to depth 30 (2^30 panels, a hang): the speed of an
    /// iso-curve is a finite difference whose noise grows with the coordinates, so the 1e-12 error goal was never reached.
    /// It finishes within the panel budget, and the length is still the span's.
    #[test]
    fn iso_length_of_a_blend_far_from_the_origin_finishes() {
        for off in [0.0, 15.0, 4000.0, 1.0e6] {
            let o = Vec3::new(off, off, off);
            let rb = RollingBall::new(plane(o, Vec3::Z), plane(o, Vec3::X), 1.0, 1.0, (3.0, 3.0), Curve::line(o, Vec3::Y), (0.0, 25.0)).unwrap();
            let edge = Curve::iso(Surface::Blend(Box::new(rb)), false, 0.0);
            let len = edge.length_between(0.0, 25.0);
            assert!((len - 25.0).abs() <= 1e-6 * 25.0 + 1e-9 * off, "offset {off}: length {len}");
        }
    }
}
