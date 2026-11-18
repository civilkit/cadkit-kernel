//! Certified local solves for surface/surface intersection: interval arithmetic and one
//! Krawczyk step for the 4-unknown system `(u1, v1, u2, v2)` of two surfaces meeting, with a
//! guarantee-or-fail result ([`Certificate`]).
//!
//! Ported in idea from solidcore (Apache-2.0, `geom/krawczyk.rs`, `geom/certified.rs`, docs
//! `ssi.md` and `definitions-and-certificates.md`), rewritten onto this kernel's types; see
//! THIRD_PARTY.md. Not a copy of its type system: intervals are the small [`Iv`] below, every
//! operation widens its result by one ulp on each side (outward rounding), and only the
//! analytic surface kinds have an interval evaluator (plane, cylinder, cone, sphere,
//! torus). Anything else is `Unresolved(Unsupported)` and the caller keeps its uncertified
//! path.
//!
//! Krawczyk's operator for `F(x) = 0` over a box `Y` about the point `m`, with `C` an
//! approximate inverse of `F'(m)`:
//! `K(Y) = m - C F(m) + (I - C F'(Y)) (Y - m)`.
//! `K(Y)` strictly inside `Y` proves that `F` has exactly one root in `Y` and that `F'(Y)`
//! is nonsingular (Krawczyk 1969, Moore 1977).
//!
//! Two uses:
//! - [`certify_point`]: the point `S1(uv) = S2(uv)` and the plane through it normal to
//!   `dir` hold exactly one root in a box (a transversal, isolated solution of the marching
//!   corrector's own system). Proves the traced point is on both surfaces.
//! - [`certify_span`]: two traced points `a`, `b` lie on the same arc of the intersection.
//!   The right-hand side of the plane equation is taken as an interval covering the
//!   levels of both points, so for every level in between the system has one root in the
//!   same box; by continuity (the Jacobian is nonsingular over the box) the roots form one
//!   arc, and `a` and `b`, being roots of their own levels, are on it. A marcher that jumped
//!   to a neighbouring branch (two branches near a branch point) cannot be certified.

use super::ss::{MPt, Uv4};
use ck_geom::surface::Surface;
use ck_geom::Vec3;

/// A closed interval; every operation rounds outward by one ulp.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Iv {
    pub lo: f64,
    pub hi: f64,
}

#[inline]
fn dn(x: f64) -> f64 {
    x.next_down()
}
#[inline]
fn up(x: f64) -> f64 {
    x.next_up()
}

impl Iv {
    pub const ZERO: Iv = Iv { lo: 0.0, hi: 0.0 };
    #[inline]
    pub fn pt(x: f64) -> Iv {
        Iv { lo: x, hi: x }
    }
    #[inline]
    pub fn new(lo: f64, hi: f64) -> Iv {
        Iv { lo, hi }
    }
    #[inline]
    pub fn mid(self) -> f64 {
        0.5 * (self.lo + self.hi)
    }
    #[inline]
    pub fn mag(self) -> f64 {
        self.lo.abs().max(self.hi.abs())
    }
    #[inline]
    pub fn add(self, o: Iv) -> Iv {
        Iv { lo: dn(self.lo + o.lo), hi: up(self.hi + o.hi) }
    }
    #[inline]
    pub fn sub(self, o: Iv) -> Iv {
        Iv { lo: dn(self.lo - o.hi), hi: up(self.hi - o.lo) }
    }
    #[inline]
    pub fn mul(self, o: Iv) -> Iv {
        let p = [self.lo * o.lo, self.lo * o.hi, self.hi * o.lo, self.hi * o.hi];
        let lo = p.iter().cloned().fold(f64::INFINITY, f64::min);
        let hi = p.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
        Iv { lo: dn(lo), hi: up(hi) }
    }
    #[inline]
    pub fn scale(self, k: f64) -> Iv {
        self.mul(Iv::pt(k))
    }
    pub fn neg(self) -> Iv {
        Iv { lo: -self.hi, hi: -self.lo }
    }
    /// sin over the interval (libm's error is under an ulp; two ulps of margin).
    pub fn sin(self) -> Iv {
        self.trig(std::f64::consts::FRAC_PI_2, f64::sin)
    }
    pub fn cos(self) -> Iv {
        self.trig(0.0, f64::cos)
    }
    /// `f` is sin or cos; `peak` is a point where `f` has its maximum 1 (period 2 pi).
    fn trig(self, peak: f64, f: fn(f64) -> f64) -> Iv {
        use std::f64::consts::{PI, TAU};
        if !(self.lo.is_finite() && self.hi.is_finite()) || self.hi - self.lo >= TAU {
            return Iv { lo: -1.0, hi: 1.0 };
        }
        let (a, b) = (f(self.lo), f(self.hi));
        let (mut lo, mut hi) = (a.min(b), a.max(b));
        // a maximum at peak + 2 pi k, a minimum at peak + pi + 2 pi k, inside [lo, hi]?
        let has = |at: f64| {
            let k = ((self.lo - at) / TAU).ceil();
            at + k * TAU <= self.hi
        };
        if has(peak) {
            hi = 1.0;
        }
        if has(peak + PI) {
            lo = -1.0;
        }
        Iv { lo: dn(dn(lo)).max(-1.0), hi: up(up(hi)).min(1.0) }
    }
}

type V3 = [Iv; 3];

fn v3(v: Vec3) -> V3 {
    [Iv::pt(v.x), Iv::pt(v.y), Iv::pt(v.z)]
}
fn vadd(a: V3, b: V3) -> V3 {
    [a[0].add(b[0]), a[1].add(b[1]), a[2].add(b[2])]
}
fn vscale(a: Vec3, k: Iv) -> V3 {
    [k.scale(a.x), k.scale(a.y), k.scale(a.z)]
}

/// Point and first partials of a surface over a parameter box.
#[derive(Clone, Copy, Debug)]
pub struct IDerivs {
    pub p: V3,
    pub du: V3,
    pub dv: V3,
}

/// Interval evaluation of the analytic surface kinds (`None` for the others).
pub fn ideriv(s: &Surface, u: Iv, v: Iv) -> Option<IDerivs> {
    let rad = |f: &ck_geom::Frame| vadd(vscale(f.x, u.cos()), vscale(f.y, u.sin()));
    let tan = |f: &ck_geom::Frame| vadd(vscale(f.x, u.sin().neg()), vscale(f.y, u.cos()));
    let mulv = |a: V3, k: Iv| [a[0].mul(k), a[1].mul(k), a[2].mul(k)];
    Some(match s {
        Surface::Plane { frame } => IDerivs {
            p: vadd(vadd(v3(frame.o), vscale(frame.x, u)), vscale(frame.y, v)),
            du: v3(frame.x),
            dv: v3(frame.y),
        },
        Surface::Cylinder { frame, r } => IDerivs {
            p: vadd(vadd(v3(frame.o), mulv(rad(frame), Iv::pt(*r))), vscale(frame.z, v)),
            du: mulv(tan(frame), Iv::pt(*r)),
            dv: v3(frame.z),
        },
        Surface::Cone { frame, r, semi_angle } => {
            let t = semi_angle.tan();
            let rho = Iv::pt(*r).add(v.scale(t));
            IDerivs {
                p: vadd(vadd(v3(frame.o), mulv(rad(frame), rho)), vscale(frame.z, v)),
                du: mulv(tan(frame), rho),
                dv: vadd(mulv(rad(frame), Iv::pt(t)), v3(frame.z)),
            }
        }
        Surface::Sphere { frame, r } => {
            let (sv, cv) = (v.sin(), v.cos());
            IDerivs {
                p: vadd(vadd(v3(frame.o), mulv(rad(frame), cv.scale(*r))), vscale(frame.z, sv.scale(*r))),
                du: mulv(tan(frame), cv.scale(*r)),
                dv: vadd(mulv(rad(frame), sv.scale(-*r)), vscale(frame.z, cv.scale(*r))),
            }
        }
        Surface::Torus { frame, major, minor } => {
            let (sv, cv) = (v.sin(), v.cos());
            let rho = Iv::pt(*major).add(cv.scale(*minor));
            IDerivs {
                p: vadd(vadd(v3(frame.o), mulv(rad(frame), rho)), vscale(frame.z, sv.scale(*minor))),
                du: mulv(tan(frame), rho),
                dv: vadd(mulv(rad(frame), sv.scale(-*minor)), vscale(frame.z, cv.scale(*minor))),
            }
        }
        _ => return None,
    })
}

/// Why a certificate could not be given. Never a statement that the contact is absent.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Unresolved {
    /// A surface kind without an interval evaluator (free-form, extrusion, offset, ...).
    Unsupported,
    /// The Jacobian at the centre is singular: a tangent contact, a branch point, a pole.
    Singular,
    /// Krawczyk's image did not fall inside the box within the step budget (the box
    /// meets a singular Jacobian: near-tangent, or the points are not on one arc).
    NoContraction,
    /// Inputs not finite or degenerate.
    Degenerate,
}

/// The result of a certified solve: a proof, or the reason there is none.
#[derive(Clone, Debug, PartialEq)]
pub enum Certificate<T> {
    Certified(T),
    Unresolved(Unresolved),
}

impl<T> Certificate<T> {
    pub fn is_certified(&self) -> bool {
        matches!(self, Certificate::Certified(_))
    }
}

/// A box proved to hold exactly one root.
#[derive(Clone, Debug, PartialEq)]
pub struct CertifiedPoint {
    /// Parameters (u1, v1, u2, v2) of the root's box centre.
    pub uv: Uv4,
    /// Half widths of the proved box in each parameter.
    pub radius: [f64; 4],
    /// Model-space point (mean of the two surfaces at the centre).
    pub p: Vec3,
}

const KRAWCZYK_STEPS: usize = 8;

fn inverse4(m: [[f64; 4]; 4]) -> Option<[[f64; 4]; 4]> {
    let mut a = [[0.0f64; 8]; 4];
    for i in 0..4 {
        a[i][..4].copy_from_slice(&m[i]);
        a[i][4 + i] = 1.0;
    }
    for c in 0..4 {
        let mut piv = c;
        for r in c + 1..4 {
            if a[r][c].abs() > a[piv][c].abs() {
                piv = r;
            }
        }
        if !(a[piv][c].abs() > 1e-13) {
            return None;
        }
        a.swap(c, piv);
        let d = a[c][c];
        for k in 0..8 {
            a[c][k] /= d;
        }
        for r in 0..4 {
            if r != c {
                let f = a[r][c];
                if f != 0.0 {
                    for k in 0..8 {
                        a[r][k] -= f * a[c][k];
                    }
                }
            }
        }
    }
    let mut out = [[0.0; 4]; 4];
    for i in 0..4 {
        for j in 0..4 {
            out[i][j] = a[i][4 + j];
        }
    }
    out.iter().flatten().all(|x| x.is_finite()).then_some(out)
}

/// `F` (rows 0-2 `S1 - S2`, row 3 `(S1 - m) . t - lam`) over a box.
fn f_box(s1: &Surface, s2: &Surface, x: &[Iv; 4], m: Vec3, t: Vec3, lam: Iv) -> Option<[Iv; 4]> {
    let d1 = ideriv(s1, x[0], x[1])?;
    let d2 = ideriv(s2, x[2], x[3])?;
    let mut f = [Iv::ZERO; 4];
    for i in 0..3 {
        f[i] = d1.p[i].sub(d2.p[i]);
    }
    let mv = [m.x, m.y, m.z];
    let tv = [t.x, t.y, t.z];
    let mut g = Iv::ZERO;
    for i in 0..3 {
        g = g.add(d1.p[i].sub(Iv::pt(mv[i])).scale(tv[i]));
    }
    f[3] = g.sub(lam);
    Some(f)
}

/// `F'` over a box.
fn j_box(s1: &Surface, s2: &Surface, x: &[Iv; 4], t: Vec3) -> Option<[[Iv; 4]; 4]> {
    let d1 = ideriv(s1, x[0], x[1])?;
    let d2 = ideriv(s2, x[2], x[3])?;
    let tv = [t.x, t.y, t.z];
    let mut j = [[Iv::ZERO; 4]; 4];
    for i in 0..3 {
        j[i] = [d1.du[i], d1.dv[i], d2.du[i].neg(), d2.dv[i].neg()];
    }
    let dot = |a: V3| (0..3).fold(Iv::ZERO, |s, i| s.add(a[i].scale(tv[i])));
    j[3] = [dot(d1.du), dot(d1.dv), Iv::ZERO, Iv::ZERO];
    Some(j)
}

/// Krawczyk's test about `mid` for the system plane-through-`m`-normal-to-`t` at levels
/// `lam`, starting from a box of half widths at least `min_rad`.
fn krawczyk(s1: &Surface, s2: &Surface, mid: [f64; 4], m: Vec3, t: Vec3, lam: Iv, min_rad: [f64; 4]) -> Certificate<([Iv; 4], [f64; 4])> {
    if !(mid.iter().all(|x| x.is_finite()) && m.is_finite() && t.is_finite()) {
        return Certificate::Unresolved(Unresolved::Degenerate);
    }
    let xm: [Iv; 4] = mid.map(Iv::pt);
    let Some(jm) = j_box(s1, s2, &xm, t) else { return Certificate::Unresolved(Unresolved::Unsupported) };
    let jp = jm.map(|r| r.map(|e| e.mid()));
    let Some(c) = inverse4(jp) else { return Certificate::Unresolved(Unresolved::Singular) };
    let Some(f) = f_box(s1, s2, &xm, m, t, lam) else { return Certificate::Unresolved(Unresolved::Unsupported) };
    // C F(m)
    let mut cf = [Iv::ZERO; 4];
    for i in 0..4 {
        for k in 0..4 {
            cf[i] = cf[i].add(f[k].scale(c[i][k]));
        }
    }
    let mut rad = [0.0; 4];
    for i in 0..4 {
        rad[i] = (2.0 * cf[i].mag() + 1e-12 * (1.0 + mid[i].abs())).max(min_rad[i]);
    }
    for _ in 0..KRAWCZYK_STEPS {
        let y: [Iv; 4] = std::array::from_fn(|i| Iv::new(mid[i] - rad[i], mid[i] + rad[i]));
        if !rad.iter().all(|r| r.is_finite()) {
            return Certificate::Unresolved(Unresolved::NoContraction);
        }
        let Some(jb) = j_box(s1, s2, &y, t) else { return Certificate::Unresolved(Unresolved::Unsupported) };
        // I - C J(Y)
        let mut cm = [[Iv::ZERO; 4]; 4];
        for i in 0..4 {
            for j in 0..4 {
                let mut e = Iv::pt(if i == j { 1.0 } else { 0.0 });
                for k in 0..4 {
                    e = e.sub(jb[k][j].scale(c[i][k]));
                }
                cm[i][j] = e;
            }
        }
        let mut k_img = [Iv::ZERO; 4];
        for i in 0..4 {
            let mut e = Iv::pt(mid[i]).sub(cf[i]);
            for j in 0..4 {
                e = e.add(cm[i][j].mul(y[j].sub(Iv::pt(mid[j]))));
            }
            k_img[i] = e;
        }
        if (0..4).all(|i| k_img[i].lo > y[i].lo && k_img[i].hi < y[i].hi) {
            let out: [Iv; 4] = std::array::from_fn(|i| Iv::new(k_img[i].lo.max(y[i].lo), k_img[i].hi.min(y[i].hi)));
            return Certificate::Certified((out, rad));
        }
        for i in 0..4 {
            let reach = (k_img[i].lo - mid[i]).abs().max((k_img[i].hi - mid[i]).abs());
            rad[i] = 1.5 * reach.max(rad[i]) + 1e-12 * (1.0 + mid[i].abs());
        }
    }
    Certificate::Unresolved(Unresolved::NoContraction)
}

/// Prove that `uv` (an approximate root of `S1 = S2` with the plane through `S1(uv)` normal to
/// `dir`) has a unique exact root nearby: the point is on both surfaces, isolated and
/// transversal. `Unresolved(Singular | NoContraction)` at a tangent contact.
pub fn certify_point(s1: &Surface, s2: &Surface, uv: Uv4, dir: Vec3) -> Certificate<CertifiedPoint> {
    let Some(t) = dir.try_normalize() else { return Certificate::Unresolved(Unresolved::Degenerate) };
    let m = s1.eval(uv[0], uv[1]);
    match krawczyk(s1, s2, uv, m, t, Iv::ZERO, [0.0; 4]) {
        Certificate::Certified((b, rad)) => {
            let ctr: Uv4 = std::array::from_fn(|i| b[i].mid());
            let p = (s1.eval(ctr[0], ctr[1]) + s2.eval(ctr[2], ctr[3])) * 0.5;
            Certificate::Certified(CertifiedPoint { uv: ctr, radius: std::array::from_fn(|i| 0.5 * (b[i].hi - b[i].lo).min(rad[i])), p })
        }
        Certificate::Unresolved(r) => Certificate::Unresolved(r),
    }
}

/// `x` moved by whole periods so that it is within half a period of `near`.
fn unwrap_near(x: f64, near: f64, period: Option<f64>) -> f64 {
    match period {
        Some(p) => x - p * ((x - near) / p).round(),
        None => x,
    }
}

/// Prove that two points of the intersection lie on one arc: for every plane level between
/// them (planes normal to `t`) the system has one root in a common parameter box, so no
/// other branch of the intersection passes between them. `t` is the marching direction.
pub fn certify_span(s1: &Surface, s2: &Surface, a: &MPt, b: &MPt, t: Vec3) -> Certificate<()> {
    let Some(t) = t.try_normalize() else { return Certificate::Unresolved(Unresolved::Degenerate) };
    let (p1, p2) = (s1.periods(), s2.periods());
    let pers = [p1.0, p1.1, p2.0, p2.1];
    let bu: Uv4 = std::array::from_fn(|i| unwrap_near(b.uv[i], a.uv[i], pers[i]));
    let mid: [f64; 4] = std::array::from_fn(|i| 0.5 * (a.uv[i] + bu[i]));
    let m = (a.p + b.p) * 0.5;
    let la = (a.p - m).dot(t);
    let lb = (b.p - m).dot(t);
    let slack = 1e-9 * (1.0 + la.abs().max(lb.abs()));
    let lam = Iv::new(la.min(lb) - slack, la.max(lb) + slack);
    let min_rad: [f64; 4] = std::array::from_fn(|i| 0.5 * (a.uv[i] - bu[i]).abs() * 1.05 + 1e-9);
    match krawczyk(s1, s2, mid, m, t, lam, min_rad) {
        Certificate::Certified(_) => Certificate::Certified(()),
        Certificate::Unresolved(r) => Certificate::Unresolved(r),
    }
}

/// Newton-refine a predicted point onto both surfaces (the marcher's corrector) and prove it.
/// Returns the refined point with its certificate. A refinement that does not converge is
/// `Unresolved(NoContraction)`.
pub fn refine_certified(s1: &Surface, s2: &Surface, pred: Vec3, dir: Vec3, uv: Uv4, tol: f64) -> Option<(MPt, Certificate<CertifiedPoint>)> {
    let (p, uv) = super::ss::correct_uv(s1, s2, pred, dir, uv, tol)?;
    let cert = certify_point(s1, s2, uv, dir);
    Some((MPt { p, uv }, cert))
}

/// Certify a polyline of intersection points (a traced curve): the share of consecutive
/// pairs proved to lie on one arc. Parameters come from projecting each point on both
/// surfaces. Returns (certified spans, unresolved spans, total spans).
pub fn verify_polyline(s1: &Surface, s2: &Surface, pts: &[Vec3]) -> (usize, usize, usize) {
    let mp: Vec<MPt> = pts
        .iter()
        .map(|&p| {
            let (a, b) = s1.closest_param(p);
            let (c, d) = s2.closest_param(p);
            MPt { p, uv: [a, b, c, d] }
        })
        .collect();
    let (mut ok, mut bad) = (0, 0);
    for w in mp.windows(2) {
        let Some(t) = (w[1].p - w[0].p).try_normalize() else { continue };
        if certify_span(s1, s2, &w[0], &w[1], t).is_certified() {
            ok += 1;
        } else {
            bad += 1;
        }
    }
    (ok, bad, ok + bad)
}

#[cfg(test)]
mod tests {
    use super::*;
    use ck_geom::{frame_from, Frame};

    fn fr(o: Vec3, z: Vec3, x: Vec3) -> Frame {
        frame_from(o, z, x)
    }

    #[test]
    fn interval_trig_encloses() {
        for k in 0..2000 {
            let a = -7.0 + 0.0071 * k as f64;
            let b = a + 0.3 + 0.002 * (k % 50) as f64;
            let (s, c) = (Iv::new(a, b).sin(), Iv::new(a, b).cos());
            for j in 0..=20 {
                let x = (a + (b - a) * j as f64 / 20.0).min(b);
                assert!(s.lo <= x.sin() && x.sin() <= s.hi && c.lo <= x.cos() && x.cos() <= c.hi);
            }
        }
    }

    #[test]
    fn transversal_point_certified_tangent_not() {
        // perpendicular cylinders r2, r1: a transversal point
        let a = Surface::Cylinder { frame: fr(Vec3::ZERO, Vec3::Z, Vec3::X), r: 2.0 };
        let b = Surface::Cylinder { frame: fr(Vec3::ZERO, Vec3::X, Vec3::Y), r: 1.0 };
        // point (sqrt(3), 1, 0)?  x^2+y^2=4 and y^2+z^2=1 -> y=1, z=0, x=sqrt3
        let p = Vec3::new(3f64.sqrt(), 1.0, 0.0);
        let (u1, v1) = a.closest_param(p);
        let (u2, v2) = b.closest_param(p);
        let dir = a.normal(u1, v1).cross(b.normal(u2, v2));
        assert!(certify_point(&a, &b, [u1, v1, u2, v2], dir).is_certified());
        // the branch point of the same pair at offset 1: (0, 2, 0) is a tangent contact
        let b2 = Surface::Cylinder { frame: fr(Vec3::new(0.0, 1.0, 0.0), Vec3::X, Vec3::Z), r: 1.0 };
        let p = Vec3::new(0.0, 2.0, 0.0);
        let (u1, v1) = a.closest_param(p);
        let (u2, v2) = b2.closest_param(p);
        let r = certify_point(&a, &b2, [u1, v1, u2, v2], Vec3::X);
        assert!(!r.is_certified(), "{r:?}");
    }

    #[test]
    fn span_across_the_neck_is_not_certified() {
        // cylinders r2 and r1.999 perpendicular: the two loops pass 0.126 apart at the neck
        let a = Surface::Cylinder { frame: fr(Vec3::ZERO, Vec3::Z, Vec3::X), r: 2.0 };
        let b = Surface::Cylinder { frame: fr(Vec3::ZERO, Vec3::X, Vec3::Y), r: 1.999 };
        let mk = |x: f64, y: f64| {
            // z from y^2+z^2 = r^2 with z=0, x from x^2+y^2=4
            let _ = y;
            let p = Vec3::new(x, (4.0 - x * x).sqrt(), 0.0);
            let (u1, v1) = a.closest_param(p);
            let (u2, v2) = b.closest_param(p);
            MPt { p, uv: [u1, v1, u2, v2] }
        };
        // (0.0632, 1.9990, 0) is on both: use exact solution x^2 = 4 - 1.999^2
        let x0 = (4.0f64 - 1.999 * 1.999).sqrt();
        let (pa, pb) = (mk(x0, 0.0), mk(-x0, 0.0));
        let dir = Vec3::new(1.0, 0.0, 0.0);
        let r = certify_span(&a, &b, &pa, &pb, dir);
        assert!(!r.is_certified(), "{r:?}");
    }
}
