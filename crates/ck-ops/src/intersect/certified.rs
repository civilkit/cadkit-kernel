//! Certified local solves for surface/surface intersection: interval arithmetic and one
//! Krawczyk step for the 4-unknown system `(u1, v1, u2, v2)` of two surfaces meeting, with a
//! guarantee-or-fail result ([`Certificate`]).
//!
//! Ported in idea from solidcore (Apache-2.0, `geom/krawczyk.rs`, `geom/certified.rs`, docs
//! `ssi.md` and `definitions-and-certificates.md`), rewritten onto this kernel's types; see
//! THIRD_PARTY.md. Not a copy of its type system: intervals are the small [`Iv`] below, every
//! operation widens its result by one ulp on each side (outward rounding). Interval evaluators
//! cover the analytic kinds, B-splines, extrusions, revolutions, screws and offsets (of the
//! analytic kinds in their exact form, of planes, B-splines and swept surfaces through the
//! normal's derivatives). Anything else is `Unresolved(Unsupported)` and the caller keeps its
//! uncertified path.
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
use ck_geom::curve::Curve;
use ck_geom::nurbs::bspline::BSplineCurve;
use ck_geom::nurbs::bsurf::BSplineSurface;
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
    /// The smallest interval holding both.
    #[inline]
    pub fn hull(self, o: Iv) -> Iv {
        Iv { lo: self.lo.min(o.lo), hi: self.hi.max(o.hi) }
    }
    /// `None` when the divisor contains zero.
    #[inline]
    pub fn div(self, o: Iv) -> Option<Iv> {
        if !(o.lo > 0.0 || o.hi < 0.0) {
            return None;
        }
        let p = [self.lo / o.lo, self.lo / o.hi, self.hi / o.lo, self.hi / o.hi];
        let lo = p.iter().cloned().fold(f64::INFINITY, f64::min);
        let hi = p.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
        Some(Iv { lo: dn(lo), hi: up(hi) })
    }
    /// Square root (IEEE sqrt is correctly rounded); `None` for a negative interval.
    pub fn sqrt(self) -> Option<Iv> {
        if !(self.hi >= 0.0) {
            return None;
        }
        Some(Iv { lo: dn(self.lo.max(0.0).sqrt()).max(0.0), hi: up(self.hi.sqrt()) })
    }
    #[inline]
    fn finite(self) -> bool {
        self.lo.is_finite() && self.hi.is_finite()
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
        Surface::BSpline(b) => bspline_ideriv(b, u, v, false)?.0,
        Surface::Extrusion { profile, dir } => {
            let (c, dc) = curve_ideriv(profile, u)?;
            IDerivs { p: vadd(c, vscale(*dir, v)), du: dc, dv: v3(*dir) }
        }
        Surface::Revolution { profile, axis_o, axis } => {
            let (c, dc) = curve_ideriv(profile, v)?;
            let w = vsub(c, v3(*axis_o));
            let (su, cu) = (u.sin(), u.cos());
            IDerivs { p: vadd(v3(*axis_o), rot(*axis, w, su, cu)), du: rot_u(*axis, w, su, cu), dv: rot(*axis, dc, su, cu) }
        }
        Surface::Screw { profile, axis_o, axis, rise, .. } => {
            let (c, dc) = curve_ideriv(profile, v)?;
            let w = vsub(c, v3(*axis_o));
            let (su, cu) = (u.sin(), u.cos());
            IDerivs {
                p: vadd(vadd(v3(*axis_o), rot(*axis, w, su, cu)), vscale(*axis, u.scale(*rise))),
                du: vadd(rot_u(*axis, w, su, cu), v3(*axis * *rise)),
                dv: rot(*axis, dc, su, cu),
            }
        }
        Surface::Offset { .. } => offset_ideriv(s, u, v)?,
        // procedural kinds have no interval evaluator: Unsupported, the march stays uncertified
        Surface::Sweep(_) | Surface::Blend(_) => return None,
    })
    .filter(|d| [d.p, d.du, d.dv].iter().flatten().all(|e| e.finite()))
}

fn vsub(a: V3, b: V3) -> V3 {
    [a[0].sub(b[0]), a[1].sub(b[1]), a[2].sub(b[2])]
}
fn vmul(a: V3, k: Iv) -> V3 {
    [a[0].mul(k), a[1].mul(k), a[2].mul(k)]
}
fn vdot(a: V3, b: V3) -> Iv {
    a[0].mul(b[0]).add(a[1].mul(b[1])).add(a[2].mul(b[2]))
}
fn vcross(a: V3, b: V3) -> V3 {
    [a[1].mul(b[2]).sub(a[2].mul(b[1])), a[2].mul(b[0]).sub(a[0].mul(b[2])), a[0].mul(b[1]).sub(a[1].mul(b[0]))]
}
fn vhull(a: V3, b: V3) -> V3 {
    [a[0].hull(b[0]), a[1].hull(b[1]), a[2].hull(b[2])]
}
fn vdiv(a: V3, k: Iv) -> Option<V3> {
    Some([a[0].div(k)?, a[1].div(k)?, a[2].div(k)?])
}

/// Rodrigues rotation about unit `z` by the angle whose sine / cosine are `su`, `cu`, split into the axial
/// part (fixed) and the rest: `R w = (z.w) z + (w - (z.w) z) cos + (z x w) sin` (as the kernel's
/// `w cos + z (z.w)(1 - cos) + (z x w) sin`).
fn rot(z: Vec3, w: V3, su: Iv, cu: Iv) -> V3 {
    let zw = vdot(v3(z), w);
    let ax = vscale(z, zw);
    vadd(vadd(ax, vmul(vsub(w, ax), cu)), vmul(vcross(v3(z), w), su))
}
/// d/du of [`rot`]: `-(w - (z.w) z) sin + (z x w) cos`.
fn rot_u(z: Vec3, w: V3, su: Iv, cu: Iv) -> V3 {
    let zw = vdot(v3(z), w);
    let perp = vsub(w, vscale(z, zw));
    vadd(vmul(perp, su.neg()), vmul(vcross(v3(z), w), cu))
}

/// Point and first derivative of a curve over a parameter interval.
fn curve_ideriv(c: &Curve, t: Iv) -> Option<(V3, V3)> {
    let (st, ct) = (t.sin(), t.cos());
    let r = match c {
        Curve::Line { o, d } => (vadd(v3(*o), vscale(*d, t)), v3(*d)),
        Curve::Circle { frame, r } => (
            vadd(v3(frame.o), vadd(vscale(frame.x, ct.scale(*r)), vscale(frame.y, st.scale(*r)))),
            vadd(vscale(frame.x, st.scale(-*r)), vscale(frame.y, ct.scale(*r))),
        ),
        Curve::Ellipse { frame, a, b } => (
            vadd(v3(frame.o), vadd(vscale(frame.x, ct.scale(*a)), vscale(frame.y, st.scale(*b)))),
            vadd(vscale(frame.x, st.scale(-*a)), vscale(frame.y, ct.scale(*b))),
        ),
        Curve::Conic { frame, p, e } => {
            // rho = p / (1 + e cos t), rho' = p e sin t / (1 + e cos t)^2
            let den = Iv::pt(1.0).add(ct.scale(*e));
            let rho = Iv::pt(*p).div(den)?;
            let drho = st.scale(p * e).div(den.mul(den))?;
            let radial = vadd(vscale(frame.x, ct), vscale(frame.y, st));
            let tang = vadd(vscale(frame.x, st.neg()), vscale(frame.y, ct));
            (vadd(v3(frame.o), vmul(radial, rho)), vadd(vmul(tang, rho), vmul(radial, drho)))
        }
        Curve::Sinusoid { frame, amp, freq, phase } => {
            let arg = t.scale(*freq).add(Iv::pt(*phase));
            (
                vadd(vadd(v3(frame.o), vscale(frame.x, t)), vscale(frame.y, arg.sin().scale(*amp))),
                vadd(v3(frame.x), vscale(frame.y, arg.cos().scale(amp * freq))),
            )
        }
        Curve::BSpline(b) => bspline_curve_ideriv(b, t)?,
        // procedural kinds have no interval evaluator: Unsupported, the march stays uncertified
        Curve::Composite(_) | Curve::Offset { .. } | Curve::Projected { .. } | Curve::OnSurface { .. } | Curve::Iso { .. } => return None,
    };
    [r.0, r.1].iter().flatten().all(|e| e.finite()).then_some(r)
}

// ---------------------------------------------------------------------------------------------
// B-splines over a parameter box. Each knot span is a polynomial (in homogeneous coordinates for
// rational ones); its Bezier control points over a sub-interval [a, b] are blossom values
// f(a, .., a, b, .., b), computed by de Boor's algorithm with interval arithmetic. The Bezier hull
// (after projection, for positive weights) holds the values over [a, b]; the derivative is the
// spline of the differenced control net, restricted the same way (no division by b - a, so the
// enclosure stays tight for tiny boxes). The kernel clamps spline parameters to the domain: a box
// reaching past it is clamped, and the derivative along that direction also takes the value 0.

/// Homogeneous point `(w (P - c), w)` (polynomial: `w = 1`).
type H4 = [Iv; 4];

fn hlerp(a: &H4, b: &H4, al: Iv) -> H4 {
    std::array::from_fn(|i| a[i].add(al.mul(b[i].sub(a[i]))))
}

/// Largest degree whose de Boor triangle is kept on the stack (higher degrees use a heap copy).
const STACK_DEG: usize = 15;

/// One de Boor step `r` of the degree-`q` piece of span `k` at the blossom argument `x`, in place on `w[0..=q]`.
#[inline]
fn deboor_step(w: &mut [H4], q: usize, k: usize, t: &[f64], r: usize, x: Iv) -> Option<()> {
    for j in (r..=q).rev() {
        let (t0, t1) = (t[j + k - q], t[j + 1 + k - r]);
        let al = x.sub(Iv::pt(t0)).div(Iv::pt(t1).sub(Iv::pt(t0)))?;
        w[j] = hlerp(&w[j - 1], &w[j], al);
    }
    Some(())
}

/// The blossom `f(xs[0], .., xs[q-1])` of the degree-`q` piece of span `k` (knots `t`), from its local
/// control points `d[0..=q]` (global indices `k - q ..= k`).
fn blossom(d: &[H4], q: usize, k: usize, t: &[f64], xs: &[Iv]) -> Option<H4> {
    if q <= STACK_DEG {
        let mut w = [[Iv::ZERO; 4]; STACK_DEG + 1];
        w[..=q].copy_from_slice(&d[..=q]);
        for r in 1..=q {
            deboor_step(&mut w, q, k, t, r, xs[r - 1])?;
        }
        return Some(w[q]);
    }
    let mut w = d.to_vec();
    for r in 1..=q {
        deboor_step(&mut w, q, k, t, r, xs[r - 1])?;
    }
    Some(w[q])
}

/// Bezier control points of the piece over `x` (one point when `x` is a point).
///
/// Point `i` is the blossom `f(lo, .., lo, hi, .., hi)` with `q - i` arguments `lo`: the de Boor steps of its
/// leading `lo` arguments are those of every point with at least as many, so the all-`lo` chain is run once
/// and each point continues from its state (the same operations in the same order as one blossom per point).
fn restrict(d: &[H4], q: usize, k: usize, t: &[f64], x: Iv) -> Option<Vec<H4>> {
    if q == 0 {
        return Some(vec![d[0]]);
    }
    if x.lo == x.hi {
        return Some(vec![blossom(d, q, k, t, &vec![x; q])?]);
    }
    let (lo, hi) = (Iv::pt(x.lo), Iv::pt(x.hi));
    if q > STACK_DEG {
        return (0..=q)
            .map(|i| {
                let xs: Vec<Iv> = (0..q).map(|r| if r < q - i { lo } else { hi }).collect();
                blossom(d, q, k, t, &xs)
            })
            .collect();
    }
    let mut out = vec![[Iv::ZERO; 4]; q + 1];
    // `chain` = state after `s` steps at `lo`; point `q - s` continues it with `q - s` steps at `hi`
    let mut chain = [[Iv::ZERO; 4]; STACK_DEG + 1];
    chain[..=q].copy_from_slice(&d[..=q]);
    for s in 0..=q {
        if s > 0 {
            deboor_step(&mut chain, q, k, t, s, lo)?;
        }
        let mut w = chain;
        for r in s + 1..=q {
            deboor_step(&mut w, q, k, t, r, hi)?;
        }
        out[q - s] = w[q];
    }
    Some(out)
}

/// The control points of the derivative of the degree-`q` piece of span `k` (degree `q - 1`; zero for `q = 0`).
fn diff(d: &[H4], q: usize, k: usize, t: &[f64]) -> Option<Vec<H4>> {
    if q == 0 {
        return Some(vec![[Iv::ZERO; 4]]);
    }
    (0..q)
        .map(|i| {
            let g = k - q + 1 + i;
            let f = Iv::pt(q as f64).div(Iv::pt(t[g + q]).sub(Iv::pt(t[g])))?;
            Some(std::array::from_fn(|c| d[i + 1][c].sub(d[i][c]).mul(f)))
        })
        .collect()
}

thread_local! {
    static EXTEND: std::cell::Cell<bool> = const { std::cell::Cell::new(false) };
}

/// While alive, the spline evaluators continue each end span's polynomial past the domain instead of
/// clamping (the edge-span proofs of [`check_curves`]; everything else sees the kernel's clamped spline).
struct Extend(bool);
impl Extend {
    fn on() -> Extend {
        Extend(EXTEND.with(|c| c.replace(true)))
    }
}
impl Drop for Extend {
    fn drop(&mut self) {
        EXTEND.with(|c| c.set(self.0));
    }
}

/// The interval clamped to the domain `[t[p], t[n + 1]]` and cut at the knots inside it: (piece, span)
/// pairs, and whether the interval reached outside the domain. Under [`Extend`] the parts past the domain
/// are pieces of the first and last spans (their polynomials continued), and the interval is not reported
/// as outside.
fn pieces(t: &[f64], p: usize, n: usize, x: Iv) -> Option<(Vec<(Iv, usize)>, bool)> {
    if !x.finite() || x.lo > x.hi {
        return None;
    }
    let (d0, d1) = (t[p], t[n + 1]);
    let outside = x.lo < d0 || x.hi > d1;
    let (lo, hi) = (x.lo.clamp(d0, d1), x.hi.clamp(d0, d1));
    let span_of = |a: f64| (p..=n).rev().find(|&k| t[k] <= a && t[k] < t[k + 1]).unwrap_or(p);
    let extend = outside && EXTEND.with(|c| c.get());
    let mut out: Vec<(Iv, usize)> = Vec::new();
    if extend && x.lo < d0 {
        out.push((Iv::new(x.lo, x.hi.min(d0)), span_of(d0)));
    }
    if !extend || (x.hi > d0 && x.lo < d1) {
        let mut cuts = vec![lo];
        for &kn in &t[p + 1..=n] {
            if kn > lo && kn < hi && kn > *cuts.last().unwrap() {
                cuts.push(kn);
            }
        }
        if cuts.len() > 32 {
            return None;
        }
        cuts.push(hi);
        out.extend(cuts.windows(2).map(|w| (Iv::new(w[0], w[1]), span_of(w[0]))));
    }
    if extend && x.hi > d1 {
        out.push((Iv::new(x.lo.max(d1), x.hi), span_of(d1)));
    }
    Some((out, outside && !extend))
}

/// A tensor-product net `net[i][j]` (degree `qu` x `qv`, spans `ku`, `kv`) restricted to `u x v`.
fn restrict2(net: &[Vec<H4>], qu: usize, ku: usize, tu: &[f64], u: Iv, qv: usize, kv: usize, tv: &[f64], v: Iv) -> Option<Vec<H4>> {
    let rows: Vec<Vec<H4>> = net.iter().map(|r| restrict(r, qv, kv, tv, v)).collect::<Option<_>>()?;
    let mut out = Vec::new();
    for j in 0..rows[0].len() {
        let col: Vec<H4> = rows.iter().map(|r| r[j]).collect();
        out.extend(restrict(&col, qu, ku, tu, u)?);
    }
    Some(out)
}

fn diff_u(net: &[Vec<H4>], q: usize, k: usize, t: &[f64]) -> Option<Vec<Vec<H4>>> {
    let cols: Vec<Vec<H4>> = (0..net[0].len()).map(|j| diff(&net.iter().map(|r| r[j]).collect::<Vec<_>>(), q, k, t)).collect::<Option<_>>()?;
    Some((0..cols[0].len()).map(|i| cols.iter().map(|c| c[i]).collect()).collect())
}
fn diff_v(net: &[Vec<H4>], q: usize, k: usize, t: &[f64]) -> Option<Vec<Vec<H4>>> {
    net.iter().map(|r| diff(r, q, k, t)).collect()
}

/// Hulls of the homogeneous parts of a set of control points: (xyz, w).
fn hhull(pts: &[H4]) -> (V3, Iv) {
    let mut a = [pts[0][0], pts[0][1], pts[0][2]];
    let mut w = pts[0][3];
    for p in &pts[1..] {
        a = vhull(a, [p[0], p[1], p[2]]);
        w = w.hull(p[3]);
    }
    (a, w)
}

/// Projected hull (rational convex hull property; every weight positive) of a set of control points.
fn proj_hull(pts: &[H4]) -> Option<V3> {
    let mut out: Option<V3> = None;
    for p in pts {
        if !(p[3].lo > 0.0) {
            return None;
        }
        let q = vdiv([p[0], p[1], p[2]], p[3])?;
        out = Some(out.map_or(q, |o| vhull(o, q)));
    }
    out
}

fn hull_opt(a: Option<V3>, b: V3) -> V3 {
    a.map_or(b, |a| vhull(a, b))
}

/// Point and first partials of a B-spline surface over `u x v`, plus the second partials
/// `(uu, uv, vv)` when `second`.
fn bspline_ideriv(s: &BSplineSurface, u: Iv, v: Iv, second: bool) -> Option<(IDerivs, Option<[V3; 3]>)> {
    let (pu, pv) = (s.deg_u, s.deg_v);
    let (tu, tv) = (&s.knots_u, &s.knots_v);
    let (pcu, out_u) = pieces(tu, pu, s.nu - 1, u)?;
    let (pcv, out_v) = pieces(tv, pv, s.nv - 1, v)?;
    if pcu.len() * pcv.len() > 64 {
        return None;
    }
    let c = s.cp(pcu[0].1, pcv[0].1);
    let (mut hp, mut hu, mut hv) = (None, None, None);
    let mut h2: [Option<V3>; 3] = [None; 3];
    for &(ui, ku) in &pcu {
        for &(vi, kv) in &pcv {
            let net: Vec<Vec<H4>> = (0..=pu)
                .map(|i| {
                    (0..=pv)
                        .map(|j| {
                            let (gi, gj) = (ku - pu + i, kv - pv + j);
                            let (p, w) = (s.cp(gi, gj), s.weight(gi, gj));
                            let wi = Iv::pt(w);
                            [Iv::pt(p.x).sub(Iv::pt(c.x)).mul(wi), Iv::pt(p.y).sub(Iv::pt(c.y)).mul(wi), Iv::pt(p.z).sub(Iv::pt(c.z)).mul(wi), wi]
                        })
                        .collect()
                })
                .collect();
            let qu = diff_u(&net, pu, ku, tu)?;
            let qv = diff_v(&net, pv, kv, tv)?;
            let (qu_d, qv_d) = (pu.saturating_sub(1), pv.saturating_sub(1));
            let b0 = restrict2(&net, pu, ku, tu, ui, pv, kv, tv, vi)?;
            let bu = restrict2(&qu, qu_d, ku, tu, ui, pv, kv, tv, vi)?;
            let bv = restrict2(&qv, pu, ku, tu, ui, qv_d, kv, tv, vi)?;
            let pt = proj_hull(&b0)?;
            let (_, w) = hhull(&b0);
            let (au, wu) = hhull(&bu);
            let (av, wv) = hhull(&bv);
            // P = A / W: P_u = (A_u - P W_u) / W
            let du = vdiv(vsub(au, vmul(pt, wu)), w)?;
            let dv = vdiv(vsub(av, vmul(pt, wv)), w)?;
            if second {
                let quu = diff_u(&qu, qu_d, ku, tu)?;
                let quv = diff_v(&qu, pv, kv, tv)?;
                let qvv = diff_v(&qv, qv_d, kv, tv)?;
                let buu = restrict2(&quu, pu.saturating_sub(2), ku, tu, ui, pv, kv, tv, vi)?;
                let buv = restrict2(&quv, qu_d, ku, tu, ui, qv_d, kv, tv, vi)?;
                let bvv = restrict2(&qvv, pu, ku, tu, ui, pv.saturating_sub(2), kv, tv, vi)?;
                let (auu, wuu) = hhull(&buu);
                let (auv, wuv) = hhull(&buv);
                let (avv, wvv) = hhull(&bvv);
                // P_uu = (A_uu - 2 P_u W_u - P W_uu) / W, P_uv = (A_uv - P_u W_v - P_v W_u - P W_uv) / W
                let puu = vdiv(vsub(vsub(auu, vmul(du, wu.scale(2.0))), vmul(pt, wuu)), w)?;
                let puv = vdiv(vsub(vsub(vsub(auv, vmul(du, wv)), vmul(dv, wu)), vmul(pt, wuv)), w)?;
                let pvv = vdiv(vsub(vsub(avv, vmul(dv, wv.scale(2.0))), vmul(pt, wvv)), w)?;
                h2 = [Some(hull_opt(h2[0], puu)), Some(hull_opt(h2[1], puv)), Some(hull_opt(h2[2], pvv))];
            }
            hp = Some(hull_opt(hp, pt));
            hu = Some(hull_opt(hu, du));
            hv = Some(hull_opt(hv, dv));
        }
    }
    let zero = [Iv::ZERO; 3];
    let (mut du, mut dv) = (hu?, hv?);
    if out_u {
        du = vhull(du, zero);
    }
    if out_v {
        dv = vhull(dv, zero);
    }
    let mut sec = None;
    if second {
        let [mut uu, mut uv, mut vv] = [h2[0]?, h2[1]?, h2[2]?];
        if out_u {
            uu = vhull(uu, zero);
            uv = vhull(uv, zero);
        }
        if out_v {
            vv = vhull(vv, zero);
            uv = vhull(uv, zero);
        }
        sec = Some([uu, uv, vv]);
    }
    Some((IDerivs { p: vadd(v3(c), hp?), du, dv }, sec))
}

/// Point and derivative of a B-spline curve over `t`.
fn bspline_curve_ideriv(b: &BSplineCurve, t: Iv) -> Option<(V3, V3)> {
    bspline_curve_ideriv2(b, t, false).map(|(p, d, _)| (p, d))
}

/// Point, first and (when `second`) second derivative of a B-spline curve over `t`.
fn bspline_curve_ideriv2(b: &BSplineCurve, t: Iv, second: bool) -> Option<(V3, V3, Option<V3>)> {
    let p = b.degree;
    let (pcs, out) = pieces(&b.knots, p, b.ctrl.len() - 1, t)?;
    let c = b.ctrl[pcs[0].1];
    let (mut hp, mut hd, mut h2) = (None, None, None);
    for &(x, k) in &pcs {
        let d: Vec<H4> = (0..=p)
            .map(|i| {
                let g = k - p + i;
                let (q, wi) = (b.ctrl[g], Iv::pt(b.weight(g)));
                [Iv::pt(q.x).sub(Iv::pt(c.x)).mul(wi), Iv::pt(q.y).sub(Iv::pt(c.y)).mul(wi), Iv::pt(q.z).sub(Iv::pt(c.z)).mul(wi), wi]
            })
            .collect();
        let dd = diff(&d, p, k, &b.knots)?;
        let b0 = restrict(&d, p, k, &b.knots, x)?;
        let b1 = restrict(&dd, p.saturating_sub(1), k, &b.knots, x)?;
        let pt = proj_hull(&b0)?;
        let (_, w) = hhull(&b0);
        let (a1, w1) = hhull(&b1);
        let der = vdiv(vsub(a1, vmul(pt, w1)), w)?;
        if second {
            // P'' = (A'' - 2 P' W' - P W'') / W
            let d2 = diff(&dd, p.saturating_sub(1), k, &b.knots)?;
            let b2 = restrict(&d2, p.saturating_sub(2), k, &b.knots, x)?;
            let (a2, w2) = hhull(&b2);
            let sec = vdiv(vsub(vsub(a2, vmul(der, w1.scale(2.0))), vmul(pt, w2)), w)?;
            h2 = Some(hull_opt(h2, sec));
        }
        hp = Some(hull_opt(hp, pt));
        hd = Some(hull_opt(hd, der));
    }
    let mut der = hd?;
    if out {
        der = vhull(der, [Iv::ZERO; 3]);
        h2 = h2.map(|h| vhull(h, [Iv::ZERO; 3]));
    }
    Some((vadd(v3(c), hp?), der, h2))
}

/// Point, first and second derivative of a curve over a parameter interval (the profile kinds a swept
/// offset base can have here; `None` for the others).
fn curve_ideriv2(c: &Curve, t: Iv) -> Option<(V3, V3, V3)> {
    let (st, ct) = (t.sin(), t.cos());
    let r = match c {
        Curve::Line { o, d } => (vadd(v3(*o), vscale(*d, t)), v3(*d), [Iv::ZERO; 3]),
        Curve::Circle { frame, r } => (
            vadd(v3(frame.o), vadd(vscale(frame.x, ct.scale(*r)), vscale(frame.y, st.scale(*r)))),
            vadd(vscale(frame.x, st.scale(-*r)), vscale(frame.y, ct.scale(*r))),
            vadd(vscale(frame.x, ct.scale(-*r)), vscale(frame.y, st.scale(-*r))),
        ),
        Curve::Ellipse { frame, a, b } => (
            vadd(v3(frame.o), vadd(vscale(frame.x, ct.scale(*a)), vscale(frame.y, st.scale(*b)))),
            vadd(vscale(frame.x, st.scale(-*a)), vscale(frame.y, ct.scale(*b))),
            vadd(vscale(frame.x, ct.scale(-*a)), vscale(frame.y, st.scale(-*b))),
        ),
        Curve::Sinusoid { frame, amp, freq, phase } => {
            let arg = t.scale(*freq).add(Iv::pt(*phase));
            (
                vadd(vadd(v3(frame.o), vscale(frame.x, t)), vscale(frame.y, arg.sin().scale(*amp))),
                vadd(v3(frame.x), vscale(frame.y, arg.cos().scale(amp * freq))),
                vscale(frame.y, arg.sin().scale(-amp * freq * freq)),
            )
        }
        Curve::BSpline(b) => {
            let (p, d, s2) = bspline_curve_ideriv2(b, t, true)?;
            (p, d, s2?)
        }
        _ => return None,
    };
    [r.0, r.1, r.2].iter().flatten().all(|e| e.finite()).then_some(r)
}

/// d^2/du^2 of [`rot`]: `-(w - (z.w) z) cos - (z x w) sin`.
fn rot_uu(z: Vec3, w: V3, su: Iv, cu: Iv) -> V3 {
    let zw = vdot(v3(z), w);
    let perp = vsub(w, vscale(z, zw));
    vsub(vmul(perp, cu.neg()), vmul(vcross(v3(z), w), su))
}

/// Point, first and second partials for the surfaces a generic offset can sit on here (plane, B-spline,
/// extrusion, revolution and screw of the profiles [`curve_ideriv2`] takes).
fn ideriv2(s: &Surface, u: Iv, v: Iv) -> Option<(IDerivs, [V3; 3])> {
    let z3 = [Iv::ZERO; 3];
    match s {
        Surface::Plane { .. } => Some((ideriv(s, u, v)?, [z3; 3])),
        Surface::BSpline(b) => {
            let (d, sec) = bspline_ideriv(b, u, v, true)?;
            Some((d, sec?))
        }
        Surface::Extrusion { profile, dir } => {
            let (c, dc, ddc) = curve_ideriv2(profile, u)?;
            Some((IDerivs { p: vadd(c, vscale(*dir, v)), du: dc, dv: v3(*dir) }, [ddc, z3, z3]))
        }
        Surface::Revolution { profile, axis_o, axis } => {
            let (c, dc, ddc) = curve_ideriv2(profile, v)?;
            let w = vsub(c, v3(*axis_o));
            let (su, cu) = (u.sin(), u.cos());
            let d = IDerivs { p: vadd(v3(*axis_o), rot(*axis, w, su, cu)), du: rot_u(*axis, w, su, cu), dv: rot(*axis, dc, su, cu) };
            Some((d, [rot_uu(*axis, w, su, cu), rot_u(*axis, dc, su, cu), rot(*axis, ddc, su, cu)]))
        }
        Surface::Screw { profile, axis_o, axis, rise, .. } => {
            let (c, dc, ddc) = curve_ideriv2(profile, v)?;
            let w = vsub(c, v3(*axis_o));
            let (su, cu) = (u.sin(), u.cos());
            let d = IDerivs {
                p: vadd(vadd(v3(*axis_o), rot(*axis, w, su, cu)), vscale(*axis, u.scale(*rise))),
                du: vadd(rot_u(*axis, w, su, cu), v3(*axis * *rise)),
                dv: rot(*axis, dc, su, cu),
            };
            Some((d, [rot_uu(*axis, w, su, cu), rot_u(*axis, dc, su, cu), rot(*axis, ddc, su, cu)]))
        }
        _ => None,
    }
}

/// The sign (+1 / -1) of an interval that does not contain zero.
fn sign_of(x: Iv) -> Option<f64> {
    if x.lo > 0.0 {
        Some(1.0)
    } else if x.hi < 0.0 {
        Some(-1.0)
    } else {
        None
    }
}

/// Offsets of cylinders, cones, spheres and tori are again surfaces of the same kind in the same
/// parameters. The offset normal is the kernel's `W / |W|` (`W = B_u x B_v`), oriented by the sign of the
/// factor in front of the geometric normal (`None` where that sign is not fixed over the box: an apex, a
/// pole, the self-crossing of a spindle torus):
/// - cylinder: `W = r rad`: radius `r + s d`;
/// - sphere: `W = r^2 cos v (rad cos v + z sin v)`: radius `r + d` (`cos v > 0`);
/// - torus: `W = rho minor (rad cos v + z sin v)`, `rho = major + minor cos v`: minor radius `minor + s d`;
/// - cone: `W = rho (rad - z tan a)`, `rho = r + v tan a`: radial `rho + s d cos a`, axial `v - s d sin a`
///   (the cone of radius `r + s d / cos a` with `v` shifted by `-s d sin a`).
fn analytic_offset(s: &Surface, dist: Iv, u: Iv, v: Iv) -> Option<IDerivs> {
    let rad = |f: &ck_geom::Frame| vadd(vscale(f.x, u.cos()), vscale(f.y, u.sin()));
    let tan = |f: &ck_geom::Frame| vadd(vscale(f.x, u.sin().neg()), vscale(f.y, u.cos()));
    Some(match s {
        Surface::Cylinder { frame, r } => {
            let rho = Iv::pt(*r).add(dist.scale(sign_of(Iv::pt(*r))?));
            IDerivs { p: vadd(vadd(v3(frame.o), vmul(rad(frame), rho)), vscale(frame.z, v)), du: vmul(tan(frame), rho), dv: v3(frame.z) }
        }
        Surface::Sphere { frame, r } => {
            let (sv, cv) = (v.sin(), v.cos());
            if !(cv.lo > 0.0 && *r != 0.0) {
                return None;
            }
            let rr = Iv::pt(*r).add(dist);
            IDerivs {
                p: vadd(vadd(v3(frame.o), vmul(rad(frame), cv.mul(rr))), vscale(frame.z, sv.mul(rr))),
                du: vmul(tan(frame), cv.mul(rr)),
                dv: vadd(vmul(rad(frame), sv.mul(rr).neg()), vscale(frame.z, cv.mul(rr))),
            }
        }
        Surface::Torus { frame, major, minor } => {
            let (sv, cv) = (v.sin(), v.cos());
            let g = sign_of(Iv::pt(*major).add(cv.scale(*minor)).scale(*minor))?;
            let mr = Iv::pt(*minor).add(dist.scale(g));
            let rho = Iv::pt(*major).add(cv.mul(mr));
            IDerivs {
                p: vadd(vadd(v3(frame.o), vmul(rad(frame), rho)), vscale(frame.z, sv.mul(mr))),
                du: vmul(tan(frame), rho),
                dv: vadd(vmul(rad(frame), sv.mul(mr).neg()), vscale(frame.z, cv.mul(mr))),
            }
        }
        Surface::Cone { frame, r, semi_angle } => {
            let t = semi_angle.tan();
            let rho = Iv::pt(*r).add(v.scale(t));
            let sd = dist.scale(sign_of(rho)?);
            let a = Iv::pt(*semi_angle);
            let rho_o = rho.add(sd.mul(a.cos()));
            let ax = v.sub(sd.mul(a.sin()));
            IDerivs {
                p: vadd(vadd(v3(frame.o), vmul(rad(frame), rho_o)), vscale(frame.z, ax)),
                du: vmul(tan(frame), rho_o),
                dv: vadd(vmul(rad(frame), Iv::pt(t)), v3(frame.z)),
            }
        }
        _ => return None,
    })
}

/// `O = B + d n`, `n = W / |W|` with `W = B_u x B_v`; `n_u = (W_u - n (n . W_u)) / |W|` (the kernel's
/// offset, nested offsets collapsed). Offsets of cylinders, cones, spheres and tori use their exact
/// analytic form ([`analytic_offset`]). `None` where `|W|` can vanish (a degenerate base).
fn offset_ideriv(s: &Surface, u: Iv, v: Iv) -> Option<IDerivs> {
    let mut cur = s;
    // the kernel sums nested distances in its own order: an interval sum holds every rounding of it
    let mut k = Iv::ZERO;
    while let Surface::Offset { base, dist } = cur {
        k = k.add(Iv::pt(*dist));
        cur = &**base;
    }
    if let Surface::Cylinder { .. } | Surface::Cone { .. } | Surface::Sphere { .. } | Surface::Torus { .. } = cur {
        let r = analytic_offset(cur, k, u, v)?;
        return [r.p, r.du, r.dv].iter().flatten().all(|e| e.finite()).then_some(r);
    }
    let (b, [buu, buv, bvv]) = ideriv2(cur, u, v)?;
    let w = vcross(b.du, b.dv);
    let wl = vdot(w, w).sqrt()?;
    if !(wl.lo > 0.0) {
        return None;
    }
    let n = vdiv(w, wl)?;
    let wu = vadd(vcross(buu, b.dv), vcross(b.du, buv));
    let wv = vadd(vcross(buv, b.dv), vcross(b.du, bvv));
    let nu = vdiv(vsub(wu, vmul(n, vdot(n, wu))), wl)?;
    let nv = vdiv(vsub(wv, vmul(n, vdot(n, wv))), wl)?;
    let r = IDerivs { p: vadd(b.p, vmul(n, k)), du: vadd(b.du, vmul(nu, k)), dv: vadd(b.dv, vmul(nv, k)) };
    [r.p, r.du, r.dv].iter().flatten().all(|e| e.finite()).then_some(r)
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
fn f_of(d1: &IDerivs, d2: &IDerivs, m: Vec3, t: Vec3, lam: Iv) -> [Iv; 4] {
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
    f
}

fn j_of(d1: &IDerivs, d2: &IDerivs, t: Vec3) -> [[Iv; 4]; 4] {
    let tv = [t.x, t.y, t.z];
    let mut j = [[Iv::ZERO; 4]; 4];
    for i in 0..3 {
        j[i] = [d1.du[i], d1.dv[i], d2.du[i].neg(), d2.dv[i].neg()];
    }
    let dot = |a: V3| (0..3).fold(Iv::ZERO, |s, i| s.add(a[i].scale(tv[i])));
    j[3] = [dot(d1.du), dot(d1.dv), Iv::ZERO, Iv::ZERO];
    j
}

fn j_box(s1: &Surface, s2: &Surface, x: &[Iv; 4], t: Vec3) -> Option<[[Iv; 4]; 4]> {
    let d1 = ideriv(s1, x[0], x[1])?;
    let d2 = ideriv(s2, x[2], x[3])?;
    Some(j_of(&d1, &d2, t))
}

/// Krawczyk's test about `mid` for the system plane-through-`m`-normal-to-`t` at levels
/// `lam`, starting from a box of half widths at least `min_rad`.
fn krawczyk(s1: &Surface, s2: &Surface, mid: [f64; 4], m: Vec3, t: Vec3, lam: Iv, min_rad: [f64; 4]) -> Certificate<([Iv; 4], [f64; 4])> {
    if !(mid.iter().all(|x| x.is_finite()) && m.is_finite() && t.is_finite()) {
        return Certificate::Unresolved(Unresolved::Degenerate);
    }
    let xm: [Iv; 4] = mid.map(Iv::pt);
    // one evaluation at the point box serves both J and F (it was evaluated twice, once for each)
    let (Some(d1), Some(d2)) = (ideriv(s1, xm[0], xm[1]), ideriv(s2, xm[2], xm[3])) else { return Certificate::Unresolved(Unresolved::Unsupported) };
    let jm = j_of(&d1, &d2, t);
    let jp = jm.map(|r| r.map(|e| e.mid()));
    let Some(c) = inverse4(jp) else { return Certificate::Unresolved(Unresolved::Singular) };
    let f = f_of(&d1, &d2, m, t, lam);
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
        // epsilon-inflation of the coordinates whose image left the box, to just past the image; the others keep
        // their width (growing every coordinate grows the Jacobian's spread over the box with it, and a thin
        // coordinate across a wide one, the sagitta of a span through the curve's vertex, never catches up)
        for i in 0..4 {
            if !(k_img[i].lo > y[i].lo && k_img[i].hi < y[i].hi) {
                let reach = (k_img[i].lo - mid[i]).abs().max((k_img[i].hi - mid[i]).abs());
                rad[i] = (1.1 * reach).max(1.5 * rad[i]) + 1e-12 * (1.0 + mid[i].abs());
            }
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
/// The box is centred on the arc (the root of the middle level), and a span whose box does
/// not contract (a shallow crossing near the curve's vertex: the arc's bend divided by the
/// normals' sine is large against the step) is proved in halves, see [`span_boxes`].
pub fn certify_span(s1: &Surface, s2: &Surface, a: &MPt, b: &MPt, t: Vec3) -> Certificate<()> {
    let Some(t) = t.try_normalize() else { return Certificate::Unresolved(Unresolved::Degenerate) };
    let (p1, p2) = (s1.periods(), s2.periods());
    let pers = [p1.0, p1.1, p2.0, p2.1];
    let bu: Uv4 = std::array::from_fn(|i| unwrap_near(b.uv[i], a.uv[i], pers[i]));
    let m = (a.p + b.p) * 0.5;
    let la = (a.p - m).dot(t);
    let lb = (b.p - m).dot(t);
    match span_boxes(s1, s2, a.uv, bu, la, lb, m, t, SPAN_SPLITS, None) {
        Certificate::Certified(_) => Certificate::Certified(()),
        Certificate::Unresolved(r) => Certificate::Unresolved(r),
    }
}

/// [`certify_span`] for a span at the edge of a spline's domain, whose proof box reaches past it (where the
/// kernel's clamped spline stops moving, the Jacobian over the box holds singular matrices and nothing
/// contracts). The boxes are proved on the spline's polynomial continuation ([`Extend`]): one arc of the
/// continued surfaces' intersection runs through the box, and the true intersection inside the box (and
/// the domain) is part of it. That arc stays inside the domain between `a` and `b` (as far as the traced
/// end points themselves are inside it) when each parameter whose box leaves the domain is monotone along
/// the arc ([`arc_monotone`]); a box where that is not proved fails.
fn certify_span_edge(s1: &Surface, s2: &Surface, a: &MPt, b: &MPt, t: Vec3) -> Certificate<()> {
    let Some(t) = t.try_normalize() else { return Certificate::Unresolved(Unresolved::Degenerate) };
    let (p1, p2) = (s1.periods(), s2.periods());
    let pers = [p1.0, p1.1, p2.0, p2.1];
    let (d1, d2) = (s1.domain(), s2.domain());
    let dom = [d1.0, d1.1, d2.0, d2.1];
    let bu: Uv4 = std::array::from_fn(|i| unwrap_near(b.uv[i], a.uv[i], pers[i]));
    let m = (a.p + b.p) * 0.5;
    let (la, lb) = ((a.p - m).dot(t), (b.p - m).dot(t));
    let _ext = Extend::on();
    let leaf = |y: &[Iv; 4], mid: [f64; 4]| {
        let need: [bool; 4] = std::array::from_fn(|i| pers[i].is_none() && (y[i].lo < dom[i].0 || y[i].hi > dom[i].1));
        !need.iter().any(|&n| n) || arc_monotone(s1, s2, y, mid, t, need)
    };
    match span_boxes(s1, s2, a.uv, bu, la, lb, m, t, SPAN_SPLITS, Some(&leaf)) {
        Certificate::Certified(_) => Certificate::Certified(()),
        Certificate::Unresolved(r) => Certificate::Unresolved(r),
    }
}

/// Proves that the parameters `need` are monotone along the arc through the box `y` (the root set of the
/// span system over its levels, whose Jacobian `J` is nonsingular over `y`): the arc's tangent `dx/dlam`
/// solves `J(x) z = e4`, and an interval vector `Z` with `C e4 + (I - C J(y)) Z` inside `Z` holds every such
/// `z` (Krawczyk's test of the linear system); a component of `Z` that excludes zero has a fixed sign.
fn arc_monotone(s1: &Surface, s2: &Surface, y: &[Iv; 4], mid: [f64; 4], t: Vec3, need: [bool; 4]) -> bool {
    let xm: [Iv; 4] = mid.map(Iv::pt);
    let Some(jm) = j_box(s1, s2, &xm, t) else { return false };
    let Some(c) = inverse4(jm.map(|r| r.map(|e| e.mid()))) else { return false };
    let Some(jb) = j_box(s1, s2, y, t) else { return false };
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
    // C e4 is the last column of C
    let ce: [f64; 4] = std::array::from_fn(|i| c[i][3]);
    let mut rad: [f64; 4] = std::array::from_fn(|i| 0.1 * ce[i].abs() + 1e-300);
    let scale = ce.iter().fold(0.0f64, |s, x| s.max(x.abs()));
    for r in rad.iter_mut() {
        *r = r.max(1e-3 * scale);
    }
    for _ in 0..KRAWCZYK_STEPS {
        let z: [Iv; 4] = std::array::from_fn(|i| Iv::new(ce[i] - rad[i], ce[i] + rad[i]));
        let img: [Iv; 4] = std::array::from_fn(|i| (0..4).fold(Iv::pt(ce[i]), |s, j| s.add(cm[i][j].mul(z[j]))));
        if (0..4).all(|i| img[i].lo > z[i].lo && img[i].hi < z[i].hi) {
            return (0..4).all(|i| !need[i] || img[i].lo > 0.0 || img[i].hi < 0.0);
        }
        for i in 0..4 {
            let reach = (img[i].lo - ce[i]).abs().max((img[i].hi - ce[i]).abs());
            rad[i] = (1.1 * reach).max(1.5 * rad[i]);
        }
    }
    false
}

/// How many times a span that does not contract in one box is halved (up to `2^SPAN_SPLITS` boxes).
const SPAN_SPLITS: usize = 3;

/// `F` and `F'` at a point (floating point), for the Newton steps that put a box centre on the arc.
fn f_j_point(s1: &Surface, s2: &Surface, x: &[f64; 4], m: Vec3, t: Vec3, lam: f64) -> ([f64; 4], [[f64; 4]; 4]) {
    let d1 = s1.derivs1(x[0], x[1]);
    let d2 = s2.derivs1(x[2], x[3]);
    let r = d1.p - d2.p;
    let f = [r.x, r.y, r.z, (d1.p - m).dot(t) - lam];
    let c = |v: Vec3| [v.x, v.y, v.z];
    let (a, b, e, g) = (c(d1.du), c(d1.dv), c(d2.du), c(d2.dv));
    let mut j = [[0.0; 4]; 4];
    for i in 0..3 {
        j[i] = [a[i], b[i], -e[i], -g[i]];
    }
    j[3] = [d1.du.dot(t), d1.dv.dot(t), 0.0, 0.0];
    (f, j)
}

/// The root of the span system at level `lam` near `x` (Newton), or `None`.
fn newton_level(s1: &Surface, s2: &Surface, x: [f64; 4], m: Vec3, t: Vec3, lam: f64) -> Option<[f64; 4]> {
    let mut x = x;
    let mut conv = false;
    for _ in 0..8 {
        let (f, j) = f_j_point(s1, s2, &x, m, t, lam);
        let c = inverse4(j)?;
        let dx: [f64; 4] = std::array::from_fn(|i| (0..4).map(|k| c[i][k] * f[k]).sum());
        for i in 0..4 {
            x[i] -= dx[i];
        }
        if dx.iter().zip(&x).all(|(d, x)| d.abs() <= 1e-13 * (1.0 + x.abs())) {
            conv = true;
            break;
        }
    }
    (conv && x.iter().all(|v| v.is_finite())).then_some(x)
}

fn box_within(a: &[Iv; 4], b: &[Iv; 4]) -> bool {
    (0..4).all(|i| a[i].lo >= b[i].lo && a[i].hi <= b[i].hi)
}

/// Proves the arc between the levels `la` and `lb` (end parameters `xa`, `xb`) in one box centred on the
/// arc's middle-level root, or, when that box does not contract, in two halves joined at that root.
/// Each half proves its levels have one root in its box; the middle level's root, certified in a small
/// box inside both halves' boxes, is then the same point of both arcs, so the halves are one arc.
/// Returns the boxes that hold the arc's two ends. `leaf`, when given, must also accept every box that
/// proves a piece of the arc (with its centre).
#[allow(clippy::too_many_arguments)]
fn span_boxes(
    s1: &Surface,
    s2: &Surface,
    xa: [f64; 4],
    xb: [f64; 4],
    la: f64,
    lb: f64,
    m: Vec3,
    t: Vec3,
    depth: usize,
    leaf: Option<&dyn Fn(&[Iv; 4], [f64; 4]) -> bool>,
) -> Certificate<([Iv; 4], [Iv; 4])> {
    let lc = 0.5 * (la + lb);
    let avg: [f64; 4] = std::array::from_fn(|i| 0.5 * (xa[i] + xb[i]));
    // the chord's midpoint is off the arc by the sagitta, which the box's margin would have to absorb (through
    // the curve's vertex the sagitta is all of a coordinate's spread: compare with the largest step)
    let reach = (0..4).map(|i| (xa[i] - xb[i]).abs()).fold(0.0, f64::max);
    let on_arc = newton_level(s1, s2, avg, m, t, lc).filter(|x| (0..4).all(|i| (x[i] - avg[i]).abs() <= reach + 1e-9 * (1.0 + avg[i].abs())));
    let mid = on_arc.unwrap_or(avg);
    let min_rad: [f64; 4] = std::array::from_fn(|i| 1.05 * (xa[i] - mid[i]).abs().max((xb[i] - mid[i]).abs()) + 1e-9);
    let slack = 1e-9 * (1.0 + la.abs().max(lb.abs()));
    let lam = Iv::new(la.min(lb) - slack, la.max(lb) + slack);
    match krawczyk(s1, s2, mid, m, t, lam, min_rad) {
        Certificate::Certified((_, rad)) => {
            let y: [Iv; 4] = std::array::from_fn(|i| Iv::new(mid[i] - rad[i], mid[i] + rad[i]));
            if leaf.is_some_and(|f| !f(&y, mid)) {
                return Certificate::Unresolved(Unresolved::NoContraction);
            }
            Certificate::Certified((y, y))
        }
        Certificate::Unresolved(Unresolved::NoContraction) if depth > 0 && on_arc.is_some() => {
            let (l0, l1) = match span_boxes(s1, s2, xa, mid, la, lc, m, t, depth - 1, leaf) {
                Certificate::Certified(b) => b,
                r => return r,
            };
            let (r0, r1) = match span_boxes(s1, s2, mid, xb, lc, lb, m, t, depth - 1, leaf) {
                Certificate::Certified(b) => b,
                r => return r,
            };
            match krawczyk(s1, s2, mid, m, t, Iv::pt(lc), [0.0; 4]) {
                Certificate::Certified((j, _)) if box_within(&j, &l1) && box_within(&j, &r0) => Certificate::Certified((l0, r1)),
                _ => Certificate::Unresolved(Unresolved::NoContraction),
            }
        }
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

/// A span of a traced curve (points `span` and `span + 1`, wrapping for a closed curve) that is not
/// proved to lie on one arc of the intersection.
#[derive(Clone, Debug, PartialEq)]
pub struct SpanIssue {
    pub curve: usize,
    pub span: usize,
    pub reason: Unresolved,
    /// Points inserted on the arc (at levels between the ends) made every sub-span provable.
    pub repaired: bool,
}

/// The certified check of the marched curves of an intersection ([`check_curves`]).
#[derive(Clone, Debug, Default, PartialEq)]
pub struct SsCheck {
    /// Spans of the marched curves (exact analytic curves are not marched and not counted).
    pub spans: usize,
    /// Spans proved as traced.
    pub certified: usize,
    /// The spans that were not, and whether inserting points repaired them.
    pub issues: Vec<SpanIssue>,
    /// Per marched curve, its points with the repair points inserted (as traced where nothing was repaired).
    pub polylines: Vec<Vec<Vec3>>,
}

impl SsCheck {
    /// Spans neither proved as traced nor repaired.
    pub fn unresolved(&self) -> usize {
        self.issues.iter().filter(|i| !i.repaired).count()
    }
}

/// Up to this many points are inserted into a span to repair it (in rounds of 1, 3, 7).
const REPAIR_ROUNDS: usize = 3;

/// Diagnostics / validation: prove that adjacent points of every marched curve of `res` lie on one arc
/// of the true intersection of `s1` and `s2` ([`certify_span`], and at a spline's edge
/// [`certify_span_edge`]); where a span is not proved, insert points of the arc (Newton at levels between
/// the ends, on the plane family normal to the chord) and prove the sub-spans. Reports the spans and the
/// repaired polylines; the intersection itself is not changed. A span left unresolved is not a proof of a
/// hop (a tangent contact or a pole also stays unresolved), only the absence of a guarantee.
pub fn check_curves(s1: &Surface, s2: &Surface, res: &super::ss::SsResult) -> SsCheck {
    let mut out = SsCheck::default();
    let (p1, p2) = (s1.periods(), s2.periods());
    let pers = [p1.0, p1.1, p2.0, p2.1];
    let mut ci = 0;
    for c in res.curves() {
        let super::ss::SsCurve::Fitted { pts, closed, .. } = c else { continue };
        let mp: Vec<MPt> = pts
            .iter()
            .map(|&p| {
                let (a, b) = s1.closest_param(p);
                let (c, d) = s2.closest_param(p);
                MPt { p, uv: [a, b, c, d] }
            })
            .collect();
        let n = mp.len();
        let nspans = if *closed && n > 2 { n } else { n.saturating_sub(1) };
        let mut poly: Vec<Vec3> = mp.first().map(|m| m.p).into_iter().collect();
        for k in 0..nspans {
            let (a, b) = (&mp[k], &mp[(k + 1) % n]);
            out.spans += 1;
            let Some(t) = (b.p - a.p).try_normalize() else {
                // coincident points: nothing between them
                out.certified += 1;
                poly.push(b.p);
                continue;
            };
            match prove_span(s1, s2, a, b, t) {
                Certificate::Certified(()) => {
                    out.certified += 1;
                    poly.push(b.p);
                }
                Certificate::Unresolved(reason) => {
                    let fix = repair_span(s1, s2, a, b, t, &pers);
                    out.issues.push(SpanIssue { curve: ci, span: k, reason, repaired: fix.is_some() });
                    if let Some(inner) = fix {
                        poly.extend(inner);
                    }
                    poly.push(b.p);
                }
            }
        }
        out.polylines.push(poly);
        ci += 1;
    }
    out
}

/// The span check of [`check_curves`]: [`certify_span`], and where that fails, the edge-span proof of
/// [`certify_span_edge`] (the first failure's reason is the one reported).
fn prove_span(s1: &Surface, s2: &Surface, a: &MPt, b: &MPt, t: Vec3) -> Certificate<()> {
    match certify_span(s1, s2, a, b, t) {
        Certificate::Unresolved(r) => match certify_span_edge(s1, s2, a, b, t) {
            Certificate::Certified(()) => Certificate::Certified(()),
            _ => Certificate::Unresolved(r),
        },
        ok => ok,
    }
}

/// Points of the arc between `a` and `b` that make every sub-span provable, or `None`.
fn repair_span(s1: &Surface, s2: &Surface, a: &MPt, b: &MPt, t: Vec3, pers: &[Option<f64>; 4]) -> Option<Vec<Vec3>> {
    let bu: Uv4 = std::array::from_fn(|i| unwrap_near(b.uv[i], a.uv[i], pers[i]));
    let m = (a.p + b.p) * 0.5;
    let (la, lb) = ((a.p - m).dot(t), (b.p - m).dot(t));
    'round: for r in 1..=REPAIR_ROUNDS {
        let k = (1usize << r) - 1;
        let mut chain = vec![*a];
        let mut x = a.uv;
        for j in 1..=k {
            let s = j as f64 / (k + 1) as f64;
            let guess: [f64; 4] = std::array::from_fn(|i| x[i] + (bu[i] - x[i]) * (1.0 / (k + 2 - j) as f64));
            let Some(y) = newton_level(s1, s2, guess, m, t, la + (lb - la) * s) else { continue 'round };
            let p = (s1.eval(y[0], y[1]) + s2.eval(y[2], y[3])) * 0.5;
            chain.push(MPt { p, uv: y });
            x = y;
        }
        chain.push(MPt { p: b.p, uv: bu });
        for w in chain.windows(2) {
            let Some(tt) = (w[1].p - w[0].p).try_normalize() else { continue 'round };
            if !prove_span(s1, s2, &w[0], &w[1], tt).is_certified() {
                continue 'round;
            }
        }
        return Some(chain[1..chain.len() - 1].iter().map(|m| m.p).collect());
    }
    None
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

#[cfg(test)]
mod enclosure_tests {
    //! The interval evaluators against dense point sampling: every sampled value and partial lies in the
    //! enclosure of a box holding the sample, and enclosures shrink with the box.
    use super::*;
    use ck_geom::{frame_from, Frame};

    struct Rng(u64);
    impl Rng {
        fn f(&mut self) -> f64 {
            self.0 = self.0.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
            (self.0 >> 11) as f64 / (1u64 << 53) as f64
        }
        fn r(&mut self, a: f64, b: f64) -> f64 {
            a + (b - a) * self.f()
        }
    }

    /// Clamped knot vector of `n` control points, degree `p`, over [a, b], with random interior knots
    /// (some repeated, up to multiplicity p).
    fn knots(rng: &mut Rng, n: usize, p: usize, a: f64, b: f64) -> Vec<f64> {
        let m = n - p - 1;
        let mut inner: Vec<f64> = Vec::new();
        while inner.len() < m {
            let k = rng.r(a + 0.05 * (b - a), b - 0.05 * (b - a));
            let rep = 1 + (rng.f() * p as f64) as usize;
            for _ in 0..rep.min(m - inner.len()) {
                inner.push(k);
            }
        }
        inner.sort_by(f64::total_cmp);
        let mut t = vec![a; p + 1];
        t.extend(inner);
        t.extend(vec![b; p + 1]);
        t
    }

    /// `restrict` as one blossom per Bezier point (the form before the shared `lo` chain).
    fn restrict_per_point(d: &[H4], q: usize, k: usize, t: &[f64], x: Iv) -> Option<Vec<H4>> {
        (0..=q)
            .map(|i| {
                let xs: Vec<Iv> = (0..q).map(|r| Iv::pt(if r < q - i { x.lo } else { x.hi })).collect();
                let mut w = d.to_vec();
                for r in 1..=q {
                    for j in (r..=q).rev() {
                        let (t0, t1) = (t[j + k - q], t[j + 1 + k - r]);
                        let al = xs[r - 1].sub(Iv::pt(t0)).div(Iv::pt(t1).sub(Iv::pt(t0)))?;
                        w[j] = hlerp(&w[j - 1], &w[j], al);
                    }
                }
                Some(w[q])
            })
            .collect()
    }

    #[test]
    fn restrict_is_bitwise_one_blossom_per_point() {
        let mut rng = Rng(7);
        for _ in 0..400 {
            let p = 1 + (rng.f() * 5.0) as usize;
            let n = p + 1 + (rng.f() * 5.0) as usize;
            let (a0, a1) = (rng.r(-2.0, 0.0), rng.r(0.5, 3.0));
            let t = knots(&mut rng, n, p, a0, a1);
            let k = (p..n).filter(|&k| t[k] < t[k + 1]).nth((rng.f() * 3.0) as usize % (n - p)).unwrap_or(p);
            let d: Vec<H4> = (0..=p).map(|_| std::array::from_fn(|c| { let a = rng.r(-5.0, 5.0); if c == 3 { Iv::pt(rng.r(0.3, 3.0)) } else { Iv::new(a, a + rng.r(0.0, 1e-6)) } })).collect();
            let (a, b) = (t[k], t[k + 1]);
            let lo = rng.r(a, b);
            let x = Iv::new(lo, rng.r(lo, b));
            let (new, old) = (restrict(&d, p, k, &t, x), restrict_per_point(&d, p, k, &t, x));
            let bits = |v: &Option<Vec<H4>>| v.as_ref().map(|v| v.iter().flat_map(|h| h.iter().flat_map(|i| [i.lo.to_bits(), i.hi.to_bits()])).collect::<Vec<_>>());
            assert_eq!(bits(&new), bits(&old));
        }
    }

    pub(super) fn rand_surface(rng: &mut Rng, pu: usize, pv: usize, rational: bool) -> BSplineSurface {
        let (nu, nv) = (pu + 1 + (rng.f() * 4.0) as usize, pv + 1 + (rng.f() * 4.0) as usize);
        let (u0, v0) = (rng.r(-2.0, 1.0), rng.r(-2.0, 1.0));
        let (u1, v1) = (u0 + rng.r(0.5, 3.0), v0 + rng.r(0.5, 3.0));
        let ku = knots(rng, nu, pu, u0, u1);
        let kv = knots(rng, nv, pv, v0, v1);
        let off = Vec3::new(rng.r(-50.0, 50.0), rng.r(-50.0, 50.0), rng.r(-50.0, 50.0));
        let ctrl: Vec<Vec3> = (0..nu * nv)
            .map(|k| off + Vec3::new((k / nv) as f64 + rng.r(-0.6, 0.6), (k % nv) as f64 + rng.r(-0.6, 0.6), rng.r(-1.5, 1.5)))
            .collect();
        let w = rational.then(|| (0..nu * nv).map(|_| rng.r(0.3, 3.0)).collect());
        BSplineSurface::new(pu, pv, ku, kv, nu, nv, ctrl, w).unwrap()
    }

    fn inside(e: &V3, p: Vec3, slack: f64) -> bool {
        let q = [p.x, p.y, p.z];
        (0..3).all(|i| {
            let s = slack * (1.0 + q[i].abs());
            e[i].lo - s <= q[i] && q[i] <= e[i].hi + s
        })
    }

    fn width(e: &V3) -> f64 {
        e.iter().map(|x| x.hi - x.lo).fold(0.0, f64::max)
    }

    /// Checks a surface's enclosures on random boxes (some reaching past the domain) against samples
    /// (point values everywhere, partials inside the domain).
    fn check_surface(s: &Surface, rng: &mut Rng, (u0, u1): (f64, f64), (v0, v1): (f64, f64), boxes: usize) {
        let (du_, dv_) = (u1 - u0, v1 - v0);
        for _ in 0..boxes {
            let wu = du_ * [1e-9, 1e-4, 0.01, 0.1, 0.5][(rng.f() * 5.0) as usize];
            let wv = dv_ * [1e-9, 1e-4, 0.01, 0.1, 0.5][(rng.f() * 5.0) as usize];
            let (a, c) = (rng.r(u0 - 0.1 * du_, u1 + 0.1 * du_ - wu), rng.r(v0 - 0.1 * dv_, v1 + 0.1 * dv_ - wv));
            let (ui, vi) = (Iv::new(a, a + wu), Iv::new(c, c + wv));
            let Some(e) = ideriv(s, ui, vi) else { panic!("no enclosure for {ui:?} {vi:?}") };
            for i in 0..=8 {
                for j in 0..=8 {
                    let (u, v) = ((a + wu * i as f64 / 8.0).min(a + wu), (c + wv * j as f64 / 8.0).min(c + wv));
                    let d = s.derivs1(u, v);
                    assert!(inside(&e.p, d.p, 1e-12), "point {:?} not in {:?} at ({u}, {v})", d.p, e.p);
                    if u >= u0 && u <= u1 && v >= v0 && v <= v1 {
                        assert!(inside(&e.du, d.du, 1e-10), "du {:?} not in {:?} at ({u}, {v}) box {ui:?} {vi:?}", d.du, e.du);
                        assert!(inside(&e.dv, d.dv, 1e-10), "dv {:?} not in {:?} at ({u}, {v}) box {ui:?} {vi:?}", d.dv, e.dv);
                    }
                }
            }
        }
    }

    #[test]
    fn bspline_surfaces_enclose_samples() {
        let mut rng = Rng(17);
        for pu in 1..=5 {
            for pv in 1..=5 {
                for rational in [false, true] {
                    for _ in 0..3 {
                        let b = rand_surface(&mut rng, pu, pv, rational);
                        let dom = b.domain();
                        check_surface(&Surface::BSpline(Box::new(b)), &mut rng, dom.0, dom.1, 12);
                    }
                }
            }
        }
    }

    #[test]
    fn bspline_enclosures_shrink_with_the_box() {
        // first-order tightness (what Krawczyk needs): the spread of the enclosure over a box of width w is
        // about |S_u| w, and that of the partials O(w), not the size of the control net
        let mut rng = Rng(5);
        for p in 1..=5 {
            for rational in [false, true] {
                let b = rand_surface(&mut rng, p, p, rational);
                let ((u0, u1), (v0, v1)) = b.domain();
                let (u, v) = (0.5 * (u0 + u1) + 1e-3, 0.5 * (v0 + v1) - 1e-3);
                let s = Surface::BSpline(Box::new(b));
                let d = s.derivs1(u, v);
                let g = d.du.len().max(d.dv.len());
                for w in [1e-3, 1e-5, 1e-7] {
                    let e = ideriv(&s, Iv::new(u - w, u + w), Iv::new(v - w, v + w)).unwrap();
                    assert!(width(&e.p) < 8.0 * g * w + 1e-9, "p={p} r={rational} w={w}: point spread {} vs {}", width(&e.p), g * w);
                    assert!(width(&e.du) < 1e4 * g * w + 1e-8 && width(&e.dv) < 1e4 * g * w + 1e-8, "p={p} r={rational} w={w}: partial spread {} {}", width(&e.du), width(&e.dv));
                }
                let e = ideriv(&s, Iv::pt(u), Iv::pt(v)).unwrap();
                assert!(width(&e.p) < 1e-11 * (1.0 + d.p.len()) && width(&e.du) < 1e-10 * (1.0 + g), "point box: {} {}", width(&e.p), width(&e.du));
            }
        }
    }

    fn rand_curve(rng: &mut Rng, p: usize, rational: bool) -> BSplineCurve {
        let n = p + 1 + (rng.f() * 4.0) as usize;
        let a = rng.r(-1.0, 1.0);
        let b = a + rng.r(0.5, 2.0);
        let t = knots(rng, n, p, a, b);
        let ctrl: Vec<Vec3> = (0..n).map(|i| Vec3::new(2.0 + rng.r(-0.5, 0.5) + 0.3 * i as f64, rng.r(-0.4, 0.4), i as f64 + rng.r(-0.3, 0.3))).collect();
        if rational {
            BSplineCurve::new_rational(p, t, ctrl, (0..n).map(|_| rng.r(0.4, 2.5)).collect()).unwrap()
        } else {
            BSplineCurve::new(p, t, ctrl).unwrap()
        }
    }

    fn fr(o: Vec3, z: Vec3, x: Vec3) -> Frame {
        frame_from(o, z, x)
    }

    #[test]
    fn swept_surfaces_enclose_samples() {
        let mut rng = Rng(99);
        let axis = Vec3::new(0.2, -0.3, 1.0).normalize();
        let o = Vec3::new(0.5, -1.0, 0.3);
        for p in 1..=5 {
            for rational in [false, true] {
                let c = rand_curve(&mut rng, p, rational);
                let dom = c.domain();
                let prof = Box::new(Curve::BSpline(c));
                let ex = Surface::Extrusion { profile: prof.clone(), dir: Vec3::new(0.3, 0.4, 2.0) };
                check_surface(&ex, &mut rng, dom, (-1.0, 2.0), 15);
                let rv = Surface::Revolution { profile: prof.clone(), axis_o: o, axis };
                check_surface(&rv, &mut rng, (-1.0, 7.0), dom, 15);
                let sc = Surface::Screw { profile: prof, axis_o: o, axis, rise: 0.37, urange: (0.0, 12.0), vrange: dom };
                check_surface(&sc, &mut rng, (-1.0, 13.0), dom, 15);
            }
        }
        let f = fr(Vec3::new(1.0, 2.0, 3.0), Vec3::new(1.0, 1.0, 0.2).normalize(), Vec3::Z);
        let analytic = [
            Curve::Line { o: Vec3::new(2.0, 0.0, -1.0), d: Vec3::new(0.3, 0.1, 1.0) },
            Curve::Circle { frame: f, r: 1.3 },
            Curve::Ellipse { frame: f, a: 2.0, b: 0.7 },
            Curve::Conic { frame: f, p: 1.2, e: 0.6 },
            Curve::Sinusoid { frame: f, amp: 0.4, freq: 2.3, phase: 0.2 },
        ];
        for c in analytic {
            let prof = Box::new(c);
            check_surface(&Surface::Extrusion { profile: prof.clone(), dir: Vec3::new(0.0, 0.5, 1.5) }, &mut rng, (-2.0, 2.0), (-1.0, 1.0), 15);
            check_surface(&Surface::Revolution { profile: prof, axis_o: o, axis }, &mut rng, (0.0, 6.3), (-1.5, 1.5), 15);
        }
    }

    #[test]
    fn offsets_enclose_samples() {
        let mut rng = Rng(3);
        let mut checked = 0;
        for p in 2..=4 {
            for rational in [false, true] {
                let b = rand_surface(&mut rng, p, p, rational);
                let ((u0, u1), (v0, v1)) = b.domain();
                // inside the domain (past it the kernel's offset normal is the clamped one)
                let inset = |a: f64, b: f64| (a + 0.05 * (b - a), b - 0.05 * (b - a));
                let s = Surface::Offset { base: Box::new(Surface::Offset { base: Box::new(Surface::BSpline(Box::new(b))), dist: 0.1 }), dist: 0.05 };
                let (ur, vr) = (inset(u0, u1), inset(v0, v1));
                for _ in 0..30 {
                    let w = (u1 - u0) * [1e-6, 1e-3, 0.02][(rng.f() * 3.0) as usize];
                    let (a, c) = (rng.r(ur.0, ur.1 - w), rng.r(vr.0, vr.1 - w));
                    let Some(e) = ideriv(&s, Iv::new(a, a + w), Iv::new(c, c + w)) else { continue };
                    checked += 1;
                    for i in 0..=6 {
                        for j in 0..=6 {
                            let (u, v) = (a + w * i as f64 / 6.0, c + w * j as f64 / 6.0);
                            let d = s.derivs2(u, v);
                            assert!(inside(&e.p, d.p, 1e-11) && inside(&e.du, d.du, 1e-8) && inside(&e.dv, d.dv, 1e-8), "offset at ({u}, {v}): {d:?} vs {e:?}");
                        }
                    }
                }
            }
        }
        assert!(checked > 100, "{checked}");
        let pl = Surface::Offset { base: Box::new(Surface::Plane { frame: fr(Vec3::new(1.0, 2.0, 3.0), Vec3::new(0.1, 0.2, 1.0).normalize(), Vec3::X) }), dist: 0.7 };
        check_surface(&pl, &mut rng, (-3.0, 3.0), (-3.0, 3.0), 10);
    }

    #[test]
    fn extended_spline_encloses_its_continuation() {
        // the bicubic Bezier patch (u, v, u^3 + v^2) over [0, 1]^2, and the cubic (t, t^3, 0): past the domain
        // the extended evaluator holds the polynomial (the clamped one would hold the edge values)
        let a = [0.0, 0.0, 0.0, 1.0];
        let b = [0.0, 0.0, 1.0 / 3.0, 1.0];
        let ctrl: Vec<Vec3> = (0..16).map(|k| Vec3::new((k / 4) as f64 / 3.0, (k % 4) as f64 / 3.0, a[k / 4] + b[k % 4])).collect();
        let kn = vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0];
        let s = Surface::BSpline(Box::new(BSplineSurface::new(3, 3, kn.clone(), kn.clone(), 4, 4, ctrl, None).unwrap()));
        let c = BSplineCurve::new(3, kn, (0..4).map(|i| Vec3::new(i as f64 / 3.0, a[i], 0.0)).collect()).unwrap();
        let mut rng = Rng(8);
        let _ext = Extend::on();
        for _ in 0..200 {
            let w = [1e-6, 1e-3, 0.1, 0.4][(rng.f() * 4.0) as usize];
            let (u0, v0) = (rng.r(-0.3, 1.3 - w), rng.r(-0.3, 1.3 - w));
            let e = ideriv(&s, Iv::new(u0, u0 + w), Iv::new(v0, v0 + w)).unwrap();
            let (p, d) = bspline_curve_ideriv(&c, Iv::new(u0, u0 + w)).unwrap();
            for i in 0..=6 {
                for j in 0..=6 {
                    let (u, v) = ((u0 + w * i as f64 / 6.0).min(u0 + w), (v0 + w * j as f64 / 6.0).min(v0 + w));
                    assert!(inside(&e.p, Vec3::new(u, v, u * u * u + v * v), 1e-12), "p at ({u}, {v}): {:?}", e.p);
                    assert!(inside(&e.du, Vec3::new(1.0, 0.0, 3.0 * u * u), 1e-12) && inside(&e.dv, Vec3::new(0.0, 1.0, 2.0 * v), 1e-12), "partials at ({u}, {v})");
                    assert!(inside(&p, Vec3::new(u, u * u * u, 0.0), 1e-12) && inside(&d, Vec3::new(1.0, 3.0 * u * u, 0.0), 1e-12), "curve at {u}");
                }
            }
        }
        drop(_ext);
        // and the guard restores the clamped evaluator: past the edge the partial may be zero again
        let e = ideriv(&s, Iv::new(1.1, 1.2), Iv::new(0.5, 0.6)).unwrap();
        assert!(e.du[0].lo <= 0.0);
    }

    /// Boxes inside `ur x vr` (an offset's normal past a spline's domain is the clamped one): every sample's
    /// point and partials (the kernel's `derivs2`) lie in the enclosure. Returns how many boxes had one.
    fn check_inside(s: &Surface, rng: &mut Rng, ur: (f64, f64), vr: (f64, f64), boxes: usize) -> usize {
        let mut checked = 0;
        for _ in 0..boxes {
            let wu = (ur.1 - ur.0) * [1e-9, 1e-5, 1e-3, 0.02, 0.2][(rng.f() * 5.0) as usize];
            let wv = (vr.1 - vr.0) * [1e-9, 1e-5, 1e-3, 0.02, 0.2][(rng.f() * 5.0) as usize];
            let (a, c) = (rng.r(ur.0, ur.1 - wu), rng.r(vr.0, vr.1 - wv));
            let Some(e) = ideriv(s, Iv::new(a, a + wu), Iv::new(c, c + wv)) else { continue };
            checked += 1;
            for i in 0..=6 {
                for j in 0..=6 {
                    let (u, v) = ((a + wu * i as f64 / 6.0).min(a + wu), (c + wv * j as f64 / 6.0).min(c + wv));
                    let d = s.derivs2(u, v);
                    assert!(
                        inside(&e.p, d.p, 1e-11) && inside(&e.du, d.du, 1e-8) && inside(&e.dv, d.dv, 1e-8),
                        "{} at ({u}, {v}): {d:?} vs {e:?}",
                        s.kind()
                    );
                }
            }
        }
        checked
    }

    #[test]
    fn analytic_offsets_enclose_samples() {
        // offsets of cylinders, cones, spheres and tori in their exact analytic form, outward, inward, past
        // the axis (a cylinder offset by more than its radius) and nested
        let mut rng = Rng(41);
        let f = fr(Vec3::new(1.0, -2.0, 0.5), Vec3::new(0.3, 0.4, 1.0).normalize(), Vec3::X);
        let off = |b: Surface, d: f64| Surface::Offset { base: Box::new(b), dist: d };
        let bases: Vec<(Surface, (f64, f64))> = vec![
            (Surface::Cylinder { frame: f, r: 2.0 }, (-3.0, 3.0)),
            (Surface::Cone { frame: f, r: 2.0, semi_angle: 0.4 }, (-3.0, 3.0)),
            (Surface::Cone { frame: f, r: 1.0, semi_angle: -0.3 }, (-2.0, 2.0)),
            (Surface::Sphere { frame: f, r: 3.0 }, (-1.3, 1.3)),
            (Surface::Torus { frame: f, major: 5.0, minor: 1.5 }, (-1.0, 7.0)),
        ];
        for (b, vr) in bases {
            for d in [0.7, -0.7, -2.5, 1e-3] {
                let s = off(b.clone(), d);
                let n = check_inside(&s, &mut rng, (-1.0, 7.0), vr, 40);
                assert_eq!(n, 40, "{} {d}: boxes without an enclosure", b.kind());
                let nested = off(off(b.clone(), 0.5 * d), 0.25 * d);
                assert_eq!(check_inside(&nested, &mut rng, (-1.0, 7.0), vr, 20), 20, "nested {} {d}", b.kind());
            }
        }
        // tight: a small box's enclosure is about as wide as the surface moves over it
        let s = off(Surface::Torus { frame: f, major: 5.0, minor: 1.5 }, 0.7);
        let e = ideriv(&s, Iv::new(0.3, 0.3 + 1e-6), Iv::new(1.1, 1.1 + 1e-6)).unwrap();
        assert!(width(&e.p) < 1e-5 && width(&e.du) < 1e-5, "{} {}", width(&e.p), width(&e.du));
        // a cone offset over a box holding the apex has no fixed normal orientation
        let k = off(Surface::Cone { frame: f, r: 1.0, semi_angle: 0.4 }, 0.2);
        let apex = -1.0 / 0.4f64.tan();
        assert!(ideriv(&k, Iv::new(0.0, 1.0), Iv::new(apex - 0.1, apex + 0.1)).is_none());
    }

    #[test]
    fn swept_offsets_enclose_samples() {
        // offsets of extrusions, revolutions and screws of B-spline and analytic profiles
        let mut rng = Rng(77);
        let axis = Vec3::new(0.2, -0.3, 1.0).normalize();
        let o = Vec3::new(0.5, -1.0, 0.3);
        let off = |b: Surface, d: f64| Surface::Offset { base: Box::new(b), dist: d };
        let inset = |(a, b): (f64, f64)| (a + 0.05 * (b - a), b - 0.05 * (b - a));
        let mut checked = 0;
        for p in 1..=5 {
            for rational in [false, true] {
                let c = rand_curve(&mut rng, p, rational);
                let dom = inset(c.domain());
                let prof = Box::new(Curve::BSpline(c));
                let ex = off(Surface::Extrusion { profile: prof.clone(), dir: Vec3::new(0.3, 0.4, 2.0) }, 0.15);
                checked += check_inside(&ex, &mut rng, dom, (-1.0, 2.0), 20);
                let rv = off(Surface::Revolution { profile: prof.clone(), axis_o: o, axis }, -0.2);
                checked += check_inside(&rv, &mut rng, (-1.0, 7.0), dom, 20);
                let sc = off(Surface::Screw { profile: prof, axis_o: o, axis, rise: 0.37, urange: (0.0, 12.0), vrange: dom }, 0.1);
                checked += check_inside(&sc, &mut rng, (0.0, 12.0), dom, 20);
            }
        }
        // degree 1 profiles have no curvature: the polyline's kinks are knots, which the boxes cut at
        assert!(checked > 450, "{checked}");
        let f = fr(Vec3::new(1.0, 2.0, 3.0), Vec3::new(1.0, 1.0, 0.2).normalize(), Vec3::Z);
        let analytic = [
            Curve::Line { o: Vec3::new(2.0, 0.0, -1.0), d: Vec3::new(0.3, 0.1, 1.0) },
            Curve::Circle { frame: f, r: 1.3 },
            Curve::Ellipse { frame: f, a: 2.0, b: 0.7 },
            Curve::Sinusoid { frame: f, amp: 0.4, freq: 2.3, phase: 0.2 },
        ];
        for c in analytic {
            let prof = Box::new(c);
            let ex = off(Surface::Extrusion { profile: prof.clone(), dir: Vec3::new(0.0, 0.5, 1.5) }, 0.3);
            // the widest boxes (a fifth of the range) can lose the normal to the wrapping of the cross product
            let n = check_inside(&ex, &mut rng, (-2.0, 2.0), (-1.0, 1.0), 20);
            assert!(n >= 15, "extrusion of {prof:?}: {n}");
            let rv = off(Surface::Revolution { profile: prof.clone(), axis_o: o, axis }, 0.25);
            let n = check_inside(&rv, &mut rng, (0.0, 6.3), (-1.5, 1.5), 20);
            assert!(n >= 12, "revolution of {prof:?}: {n}");
        }
    }
}
