//! N-sided patch filling: a closed loop of N >= 3 boundary curves is filled with N four-sided
//! patches round a centre point.
//!
//! The scheme follows the central split of the solidcore n-sided fill (`fill_n_sided_hole`,
//! Apache-2.0, see THIRD_PARTY.md), rewritten onto our types:
//!
//! 1. each boundary curve is split at its parameter midpoint `M_i`;
//! 2. a centre `C` and a centre normal are estimated (from the cross-boundary tangents when
//!    given, else from the loop), and a cubic inner curve runs from each `M_i` to `C`, leaving
//!    along the cross direction and arriving in the centre's tangent plane;
//! 3. patch `k` is the quadrilateral `V_k, M_k, C, M_(k-1)`: sides are the first half of curve
//!    `k`, the second half of curve `k-1` (reversed) and the inner curves `k` and `k-1`;
//! 4. every patch is a bicubically blended Coons patch (Hermite blending of the four boundary
//!    curves and their cross-boundary derivatives, minus the corner Hermite patch).  The cross
//!    derivatives across an inner curve lie in one tangent-plane field `n(t)` shared by the two
//!    patches, so the seam is G1.  The corner twists are made compatible by adding small bumps
//!    to the derivative fields (the solidcore fill approximates the derivatives instead).
//!
//! Each patch is then sampled on a uniform grid and interpolated by a bicubic B-spline surface;
//! the grid is doubled until the boundary curves are interpolated within the tolerance.  Both
//! patches of a seam sample the same points at the same parameters, so the seams agree exactly
//! (G0).  Where cross-boundary tangents are supplied the patch leaves the boundary tangent to
//! them (G1, up to a small angle near the corners where the twist bumps act).
//!
//! Orientation: the curves run end to start round the loop; the patch normals point to the
//! side the loop turns counter-clockwise about (`n = sum (M_i - C) x (M_(i+1) - C)`).
//! Cross-boundary tangent curves point from the boundary into the hole.

use crate::fit::grid_surface;
use crate::nurbs::bspline::BSplineCurve;
use crate::nurbs::bsurf::BSplineSurface;
use crate::Vec3;

/// The fewest sides of a loop.
pub const MIN_SIDES: usize = 3;
/// The most sides of a loop.
pub const MAX_SIDES: usize = 64;

/// One boundary curve of the loop and its optional cross-boundary derivative.
#[derive(Debug, Clone)]
pub struct NSidedSide {
    /// The boundary curve; the end of side `i` is the start of side `i + 1`.
    pub curve: BSplineCurve,
    /// The derivative across the boundary, pointing into the hole, as a curve over the same
    /// parameter range as `curve` (its parameter is mapped by fraction). Its magnitude is the
    /// derivative with respect to a parameter that runs 0 to 1 over the patch, so a length of the
    /// order of the distance to the loop's centre gives an even patch.
    pub cross: Option<BSplineCurve>,
}

impl NSidedSide {
    pub fn new(curve: BSplineCurve) -> Self {
        NSidedSide { curve, cross: None }
    }
    pub fn with_cross(curve: BSplineCurve, cross: BSplineCurve) -> Self {
        NSidedSide { curve, cross: Some(cross) }
    }
}

/// Why a loop has no patch.
#[derive(Debug, Clone, PartialEq)]
pub enum NSidedError {
    /// The input is unusable (side count, non-finite numbers, tolerance, gaps between curves, ...).
    Input(String),
    /// The loop's shape does not allow a central split (degenerate or folded).
    Geometry(String),
    /// The patches cannot interpolate the boundary within the tolerance.
    Tolerance { deviation: f64, tol: f64 },
}

impl std::fmt::Display for NSidedError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            NSidedError::Input(s) => write!(f, "n-sided patch input: {s}"),
            NSidedError::Geometry(s) => write!(f, "n-sided patch geometry: {s}"),
            NSidedError::Tolerance { deviation, tol } => {
                write!(f, "n-sided patch deviates {deviation:.3e} from the boundary (tolerance {tol:.3e})")
            }
        }
    }
}

impl std::error::Error for NSidedError {}

/// The patches of a fill and what was measured on them.
#[derive(Debug, Clone)]
pub struct NSidedPatch {
    /// Patch `k` has its corner `(0, 0)` at the start of side `k`: `u` runs along the first half
    /// of side `k` to its midpoint, `v` along the second half of side `k - 1` reversed.
    /// `(1, 1)` is the centre of all of them.
    pub patches: Vec<BSplineSurface>,
    /// The centre point.
    pub centre: Vec3,
    /// The largest distance of a boundary half from the patch side that stands for it.
    pub boundary_dev: f64,
    /// The largest gap between the two patches along an inner seam.
    pub seam_gap: f64,
    /// The largest angle (radians) between the normals of the two patches along an inner seam.
    pub seam_angle: f64,
}

fn input(s: impl Into<String>) -> NSidedError {
    NSidedError::Input(s.into())
}
fn geom(s: impl Into<String>) -> NSidedError {
    NSidedError::Geometry(s.into())
}

// ---- small vector helpers ----

fn unit(v: Vec3) -> Option<Vec3> {
    let l = v.len();
    if l.is_finite() && l > 1e-300 {
        Some(v * (1.0 / l))
    } else {
        None
    }
}

/// `v` without its component along the unit vector `n`.
fn rej(v: Vec3, n: Vec3) -> Vec3 {
    v - n * v.dot(n)
}

fn h00(s: f64) -> f64 {
    (2.0 * s - 3.0) * s * s + 1.0
}
fn h01(s: f64) -> f64 {
    (3.0 - 2.0 * s) * s * s
}
fn h10(s: f64) -> f64 {
    ((s - 2.0) * s + 1.0) * s
}
fn h11(s: f64) -> f64 {
    (s - 1.0) * s * s
}
/// Value 0 at both ends, slope 1 at 0 and 0 at 1; narrow, so the fix stays near the corner.
fn b0(s: f64) -> f64 {
    s * (1.0 - s).powi(5)
}
/// Value 0 at both ends, slope 0 at 0 and 1 at 1.
fn b1(s: f64) -> f64 {
    -(1.0 - s) * s.powi(5)
}
/// 1 at 0 and 0 at 1, narrow.
fn w5(s: f64) -> f64 {
    (1.0 - s).powi(5)
}

type Fld<'a> = &'a dyn Fn(f64) -> Vec3;

const FD: f64 = 1e-3;

/// The derivative at 0 and at 1 of a field on [0, 1], by one-sided three-point differences.
fn d0(f: Fld) -> Vec3 {
    (f(0.0) * -3.0 + f(FD) * 4.0 - f(2.0 * FD)) * (0.5 / FD)
}
fn d1(f: Fld) -> Vec3 {
    (f(1.0) * 3.0 - f(1.0 - FD) * 4.0 + f(1.0 - 2.0 * FD)) * (0.5 / FD)
}

/// `f` with its values at 0 and 1 replaced by `e0` and `e1` (the change dies away within a fifth of the side).
fn forced(f: Fld, e0: Vec3, e1: Vec3, t: f64) -> Vec3 {
    f(t) + (e0 - f(0.0)) * w5(t) + (e1 - f(1.0)) * w5(1.0 - t)
}

/// `f` with its slopes at 0 and 1 replaced by `w0` and `w1` (when given), the end values left alone.
fn twisted(f: Fld, fd0: Vec3, fd1: Vec3, w0: Vec3, w1: Vec3, t: f64) -> Vec3 {
    f(t) + (w0 - fd0) * b0(t) + (w1 - fd1) * b1(t)
}

struct Geo<'a> {
    sides: &'a [NSidedSide],
    n: usize,
    /// parameter range of each curve and its half width
    a: Vec<f64>,
    b: Vec<f64>,
    m: Vec<f64>,
    h: Vec<f64>,
    centre: Vec3,
    nn: Vec3,
    inner: Vec<BSplineCurve>,
    /// the unit normal at the start of each inner curve
    n0: Vec<Vec3>,
}

impl<'a> Geo<'a> {
    fn pt(&self, i: usize, t: f64) -> Vec3 {
        self.sides[i].curve.eval(t)
    }
    fn tan(&self, i: usize, t: f64) -> Vec3 {
        self.sides[i].curve.derivs(t, 1)[1]
    }
    /// The cross derivative of side `i` at curve parameter `t`.
    fn cross_at(&self, i: usize, t: f64) -> Option<Vec3> {
        let d = self.sides[i].cross.as_ref()?;
        let (d0, d1) = d.domain();
        let f = (t - self.a[i]) / (self.b[i] - self.a[i]);
        Some(d.eval(d0 + f * (d1 - d0)))
    }
    /// First half of side `i` on [0, 1] (value and derivative).
    fn l(&self, i: usize, u: f64) -> (Vec3, Vec3) {
        let t = self.a[i] + u * self.h[i];
        (self.pt(i, t), self.tan(i, t) * self.h[i])
    }
    /// Second half of side `i`, reversed, on [0, 1] (value and derivative).
    fn r(&self, i: usize, v: f64) -> (Vec3, Vec3) {
        let t = self.b[i] - v * self.h[i];
        (self.pt(i, t), self.tan(i, t) * -self.h[i])
    }
    fn inn(&self, i: usize, t: f64) -> (Vec3, Vec3) {
        let d = self.inner[i].derivs(t, 1);
        (d[0], d[1])
    }

    /// Tangent-plane field of seam `i`: the vector `(1 - t) e0 + t e1` projected into the plane
    /// that contains the seam's tangent and turns its normal from `n0` to the centre normal.
    fn seam_field(&self, i: usize, t: f64, e0: Vec3, e1: Vec3) -> Vec3 {
        let (_, tg) = self.inn(i, t);
        let nl = self.n0[i] * (1.0 - t) + self.nn * t;
        let tu = unit(tg).unwrap_or(Vec3::ZERO);
        let n = unit(rej(nl, tu)).unwrap_or(self.nn);
        let v = e0 * (1.0 - t) + e1 * t;
        rej(v, n)
    }

    fn build(sides: &'a [NSidedSide], tol: f64) -> Result<Geo<'a>, NSidedError> {
        let n = sides.len();
        let mut a = vec![];
        let mut b = vec![];
        let mut m = vec![];
        let mut h = vec![];
        let mut scale: f64 = 0.0;
        for s in sides {
            let (t0, t1) = s.curve.domain();
            if !(t1 > t0) || !t0.is_finite() || !t1.is_finite() {
                return Err(input("a boundary curve has an empty or non-finite parameter range"));
            }
            if s.curve.ctrl.iter().any(|p| !p.is_finite()) {
                return Err(input("a boundary curve has a non-finite control point"));
            }
            if let Some(d) = &s.cross {
                if d.ctrl.iter().any(|p| !p.is_finite()) {
                    return Err(input("a cross-boundary curve has a non-finite control point"));
                }
                if !(d.domain().1 > d.domain().0) {
                    return Err(input("a cross-boundary curve has an empty parameter range"));
                }
            }
            a.push(t0);
            b.push(t1);
            m.push(0.5 * (t0 + t1));
            h.push(0.5 * (t1 - t0));
            let len = s.curve.length();
            if !(len > 1e-12) || !len.is_finite() {
                return Err(input("a boundary curve has no length"));
            }
            scale = scale.max(len);
        }
        let gap_tol = tol.max(1e-9 * scale);
        for i in 0..n {
            let j = (i + 1) % n;
            let gap = sides[i].curve.end().dist(sides[j].curve.start());
            if !(gap <= gap_tol) {
                return Err(input(format!("side {i} does not end where side {j} starts (gap {gap:.3e})")));
            }
        }
        let mut g = Geo {
            sides,
            n,
            a,
            b,
            m,
            h,
            centre: Vec3::ZERO,
            nn: Vec3::ZERO,
            inner: vec![],
            n0: vec![],
        };
        let mid: Vec<Vec3> = (0..n).map(|i| g.pt(i, g.m[i])).collect();
        let lens: Vec<f64> = sides.iter().map(|s| s.curve.length()).collect();
        let cg = mid.iter().fold(Vec3::ZERO, |s, &p| s + p) * (1.0 / n as f64);
        // inward direction at each midpoint, scaled by a quarter of the neighbours' length
        let mut dmid = vec![];
        for i in 0..n {
            let tg = unit(g.tan(i, g.m[i])).ok_or_else(|| geom("a boundary curve has no tangent at its middle"))?;
            let dir = match g.cross_at(i, g.m[i]) {
                Some(d) => {
                    let dn = unit(d).ok_or_else(|| input("a cross-boundary curve vanishes at the middle of its side"))?;
                    if dn.cross(tg).len() < 0.05 {
                        return Err(input("a cross-boundary tangent runs along its boundary curve"));
                    }
                    dn
                }
                None => unit(rej(cg - mid[i], tg)).ok_or_else(|| geom("the loop has no inward direction at the middle of a side"))?,
            };
            let la = 0.25 * (lens[(i + n - 1) % n] + lens[(i + 1) % n]);
            dmid.push(dir * la);
        }
        let mut centre = Vec3::ZERO;
        for i in 0..n {
            centre = centre + (mid[i] + dmid[i] * 0.5);
        }
        g.centre = centre * (1.0 / n as f64);
        let mut nsum = Vec3::ZERO;
        for i in 0..n {
            nsum = nsum + (mid[i] - g.centre).cross(mid[(i + 1) % n] - g.centre);
        }
        g.nn = unit(nsum * (1.0 / n as f64)).ok_or_else(|| geom("the loop has no orientation (it is degenerate)"))?;
        for i in 0..n {
            let mi = mid[i];
            let t0 = unit(dmid[i]).unwrap();
            let a_pt = mi - g.nn * (mi - g.centre).dot(g.nn);
            let t3 = unit(g.centre - a_pt).ok_or_else(|| geom("a side's midpoint lies on the centre axis"))?;
            let f = (g.centre - mi).len() / 3.0;
            if !(f > 1e-12) {
                return Err(geom("a side's midpoint is at the centre"));
            }
            let c = BSplineCurve::bezier(vec![mi, mi + t0 * f, g.centre - t3 * f, g.centre]);
            g.inner.push(c);
        }
        for i in 0..n {
            let (_, lp1) = g.l(i, 1.0);
            let (_, ip0) = g.inn(i, 0.0);
            let nrm = unit(lp1.cross(ip0)).ok_or_else(|| geom("an inner curve leaves its boundary along the tangent"))?;
            if nrm.dot(g.nn) < -0.5 {
                return Err(geom("the loop folds back on itself (a boundary normal opposes the centre normal)"));
            }
            g.n0.push(nrm);
        }
        Ok(g)
    }

    /// Patch `k` sampled on `ns x ns` points and interpolated.
    fn patch(&self, k: usize, ns: usize) -> Result<BSplineSurface, NSidedError> {
        let n = self.n;
        let p = (k + n - 1) % n;
        let ts: Vec<f64> = (0..ns).map(|j| j as f64 / (ns - 1) as f64).collect();

        let (_, lp0) = self.l(k, 0.0);
        let (_, lp1) = self.l(k, 1.0);
        let (_, rp0) = self.r(p, 0.0);
        let (_, rp1) = self.r(p, 1.0);
        let (_, ik0) = self.inn(k, 0.0);
        let (_, ik1) = self.inn(k, 1.0);
        let (_, ip0) = self.inn(p, 0.0);
        let (_, ip1) = self.inn(p, 1.0);

        // cross-derivative fields before the twist bumps
        let bd_base = |u: f64| -> Vec3 {
            match self.cross_at(k, self.a[k] + u * self.h[k]) {
                Some(d) => d,
                None => self.inn(p, u).0 - self.l(k, u).0,
            }
        };
        let ld_base = |v: f64| -> Vec3 {
            match self.cross_at(p, self.b[p] - v * self.h[p]) {
                Some(d) => d,
                None => self.inn(k, v).0 - self.r(p, v).0,
            }
        };
        let bd_f = |u: f64| forced(&bd_base, rp0, ik0, u);
        let ld_f = |v: f64| forced(&ld_base, lp0, ip0, v);
        let rd_f = |v: f64| self.seam_field(k, v, lp1, ip1);
        let td_f = |u: f64| self.seam_field(p, u, rp1, ik1);

        let w00 = (d0(&bd_f) + d0(&ld_f)) * 0.5;
        let w10 = d0(&rd_f);
        let w01 = d0(&td_f);
        let w11 = (d1(&rd_f) + d1(&td_f)) * 0.5;

        let (bd_d0, bd_d1) = (d0(&bd_f), d1(&bd_f));
        let (ld_d0, ld_d1) = (d0(&ld_f), d1(&ld_f));
        let (rd_d0, rd_d1) = (d0(&rd_f), d1(&rd_f));
        let (td_d0, td_d1) = (d0(&td_f), d1(&td_f));
        let bd = |u: f64| twisted(&bd_f, bd_d0, bd_d1, w00, w10, u);
        let ld = |v: f64| twisted(&ld_f, ld_d0, ld_d1, w00, w01, v);
        let rd = |v: f64| twisted(&rd_f, rd_d0, rd_d1, w10, w11, v);
        let td = |u: f64| twisted(&td_f, td_d0, td_d1, w01, w11, u);

        // sampled sides and fields
        let bb: Vec<Vec3> = ts.iter().map(|&u| self.l(k, u).0).collect();
        let tt: Vec<Vec3> = ts.iter().map(|&u| self.inn(p, u).0).collect();
        let lf: Vec<Vec3> = ts.iter().map(|&v| self.r(p, v).0).collect();
        let rt: Vec<Vec3> = ts.iter().map(|&v| self.inn(k, v).0).collect();
        let bdv: Vec<Vec3> = ts.iter().map(|&u| bd(u)).collect();
        let tdv: Vec<Vec3> = ts.iter().map(|&u| td(u)).collect();
        let ldv: Vec<Vec3> = ts.iter().map(|&v| ld(v)).collect();
        let rdv: Vec<Vec3> = ts.iter().map(|&v| rd(v)).collect();

        // corner data: position, S_u, S_v, S_uv for (a, b) = (0,0), (1,0), (0,1), (1,1)
        let pos = [bb[0], bb[ns - 1], tt[0], tt[ns - 1]];
        let su = [lp0, lp1, ip0, ip1];
        let sv = [rp0, ik0, rp1, ik1];
        let suv = [w00, w10, w01, w11];

        let mut grid = vec![vec![Vec3::ZERO; ns]; ns];
        for (i, &u) in ts.iter().enumerate() {
            let hu = [h00(u), h01(u), h10(u), h11(u)];
            for (j, &v) in ts.iter().enumerate() {
                let hv = [h00(v), h01(v), h10(v), h11(v)];
                let pv = bb[i] * hv[0] + tt[i] * hv[1] + bdv[i] * hv[2] + tdv[i] * hv[3];
                let pu = lf[j] * hu[0] + rt[j] * hu[1] + ldv[j] * hu[2] + rdv[j] * hu[3];
                let mut pt = Vec3::ZERO;
                for c in 0..4 {
                    let (ca, cb) = (c & 1, c >> 1);
                    let (hpa, hda) = (hu[ca], hu[2 + ca]);
                    let (hpb, hdb) = (hv[cb], hv[2 + cb]);
                    pt = pt + pos[c] * (hpa * hpb) + su[c] * (hda * hpb) + sv[c] * (hpa * hdb) + suv[c] * (hda * hdb);
                }
                let s = pu + pv - pt;
                if !s.is_finite() {
                    return Err(geom("the patch is not finite"));
                }
                grid[i][j] = s;
            }
        }
        grid_surface(&grid, &ts, &ts, 3, 3).map_err(|e| geom(format!("patch fit failed: {e}")))
    }
}

fn angle(a: Vec3, b: Vec3) -> f64 {
    let (la, lb) = (a.len(), b.len());
    if !(la > 0.0 && lb > 0.0) {
        return 0.0;
    }
    (a.dot(b) / (la * lb)).clamp(-1.0, 1.0).acos()
}

/// Fill the closed loop of `sides` with `sides.len()` patches (module docs).
///
/// `tol` bounds the distance of every boundary curve from the patch sides that stand for it; the
/// fill refines its sampling until that holds and fails with [`NSidedError::Tolerance`] when it
/// cannot.
///
/// # Errors
///
/// Fewer than [`MIN_SIDES`] or more than [`MAX_SIDES`] sides, a non-finite or empty curve, `tol` not
/// finite and positive, curves that do not meet end to start (within `tol`), a cross tangent along
/// its boundary, or a loop that is degenerate or folds back so no central split exists.
pub fn n_sided_patch(sides: &[NSidedSide], tol: f64) -> Result<NSidedPatch, NSidedError> {
    let n = sides.len();
    if n < MIN_SIDES || n > MAX_SIDES {
        return Err(input(format!("{n} sides (need {MIN_SIDES} to {MAX_SIDES})")));
    }
    if !(tol.is_finite() && tol > 0.0) {
        return Err(input("the tolerance must be finite and positive"));
    }
    let g = Geo::build(sides, tol)?;
    let mut worst = f64::INFINITY;
    for &ns in &[17usize, 33, 65, 129] {
        let patches: Vec<BSplineSurface> = (0..n).map(|k| g.patch(k, ns)).collect::<Result<_, _>>()?;
        let mut dev: f64 = 0.0;
        let mut gap: f64 = 0.0;
        let mut ang: f64 = 0.0;
        let mids: Vec<f64> = (0..ns - 1).map(|j| (j as f64 + 0.5) / (ns - 1) as f64).collect();
        for k in 0..n {
            let p = (k + n - 1) % n;
            let s = &patches[k];
            for &t in &mids {
                dev = dev.max(s.eval(t, 0.0).dist(g.l(k, t).0));
                dev = dev.max(s.eval(0.0, t).dist(g.r(p, t).0));
            }
            // seam k is side u = 1 of patch k and side v = 1 of patch k + 1
            let nx = &patches[(k + 1) % n];
            for (j, &t) in mids.iter().enumerate() {
                gap = gap.max(s.eval(1.0, t).dist(nx.eval(t, 1.0)));
                if j > 0 && j + 1 < mids.len() {
                    ang = ang.max(angle(s.normal(1.0, t), nx.normal(t, 1.0)));
                }
            }
            // every patch normal stays on the side of the centre normal
            for &u in &mids {
                for &v in &mids {
                    let nv = s.normal(u, v);
                    if nv.len() > 0.0 && nv.dot(g.nn) < 0.0 {
                        return Err(geom("the patch folds over (its normal opposes the centre normal)"));
                    }
                }
            }
        }
        worst = dev;
        if dev <= tol {
            return Ok(NSidedPatch { patches, centre: g.centre, boundary_dev: dev, seam_gap: gap, seam_angle: ang });
        }
    }
    Err(NSidedError::Tolerance { deviation: worst, tol })
}

/// A cross-boundary tangent curve for `curve` from the normals of the surface it lies on:
/// at each parameter the direction `normal x tangent` (unit length times `scale`), the sign chosen
/// at the middle of the curve so it points away from `away` (a point on the neighbour's side;
/// for a hole to fill, a point on the neighbouring surface's interior), or toward `toward`.
///
/// `normal(t)` is the neighbour's surface normal at the curve point of parameter `t`. `toward` is
/// `Some(point)` to make the tangent point to that point, `None` to take the direction exactly as
/// `normal x tangent` gives it.
pub fn cross_from_normals(
    curve: &BSplineCurve,
    normal: &dyn Fn(f64) -> Vec3,
    scale: f64,
    toward: Option<Vec3>,
) -> Result<BSplineCurve, NSidedError> {
    let (t0, t1) = curve.domain();
    if !(t1 > t0) || !(scale.is_finite() && scale > 0.0) {
        return Err(input("a curve without a parameter range, or a scale not finite and positive"));
    }
    let dir_at = |t: f64| -> Option<Vec3> { unit(normal(t).cross(curve.derivs(t, 1)[1])) };
    let tm = 0.5 * (t0 + t1);
    let sign = match toward {
        Some(q) => {
            let d = dir_at(tm).ok_or_else(|| input("no cross direction at the middle of the curve"))?;
            if d.dot(q - curve.eval(tm)) < 0.0 {
                -1.0
            } else {
                1.0
            }
        }
        None => 1.0,
    };
    let np = 33usize;
    let us: Vec<f64> = (0..np).map(|j| t0 + (t1 - t0) * j as f64 / (np - 1) as f64).collect();
    let mut pts = Vec::with_capacity(np);
    let mut prev: Option<Vec3> = None;
    for &t in &us {
        let mut d = dir_at(t).ok_or_else(|| input("the normal is parallel to the curve (or vanishes)"))? * sign;
        if let Some(p) = prev {
            // keep the sense continuous when the normals flip along the curve
            if d.dot(p) < 0.0 {
                d = -d;
            }
        }
        prev = Some(d);
        pts.push(d * scale);
    }
    let knots = crate::fit::scaled_knots(&us, 3);
    crate::fit::interpolate_with(&pts, 3, &us, knots).map_err(|e| input(format!("cross curve fit failed: {e}")))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::convert::segment;

    fn polygon(n: usize, r: f64, z: impl Fn(f64) -> f64) -> Vec<Vec3> {
        (0..n)
            .map(|i| {
                let th = std::f64::consts::TAU * i as f64 / n as f64;
                Vec3::new(r * th.cos(), r * th.sin(), z(th))
            })
            .collect()
    }

    fn line_sides(pts: &[Vec3]) -> Vec<NSidedSide> {
        let n = pts.len();
        (0..n).map(|i| NSidedSide::new(segment(pts[i], pts[(i + 1) % n], 0.0, 1.0))).collect()
    }

    fn check_fill(sides: &[NSidedSide], tol: f64) -> NSidedPatch {
        let f = n_sided_patch(sides, tol).expect("fill");
        assert_eq!(f.patches.len(), sides.len());
        assert!(f.boundary_dev <= tol, "dev {}", f.boundary_dev);
        // boundary interpolation on sampled points
        let n = sides.len();
        for k in 0..n {
            let p = (k + n - 1) % n;
            let c = &sides[k].curve;
            let cp = &sides[p].curve;
            let (a, b) = c.domain();
            let m = 0.5 * (a + b);
            for j in 0..=40 {
                let u = j as f64 / 40.0;
                let e = f.patches[k].eval(u, 0.0).dist(c.eval(a + u * (m - a)));
                assert!(e <= tol, "bottom {e}");
                let (pa, pb) = cp.domain();
                let pm = 0.5 * (pa + pb);
                let e = f.patches[k].eval(0.0, u).dist(cp.eval(pb - u * (pb - pm)));
                assert!(e <= tol, "left {e}");
            }
        }
        // G0 across seams, G1 normal continuity
        assert!(f.seam_gap <= 1e-9, "gap {}", f.seam_gap);
        f
    }

    #[test]
    fn planar_polygons_fill_flat() {
        for n in [3usize, 4, 5, 6] {
            let pts = polygon(n, 1.0, |_| 0.0);
            let f = check_fill(&line_sides(&pts), 1e-6);
            for s in &f.patches {
                for i in 0..=8 {
                    for j in 0..=8 {
                        let q = s.eval(i as f64 / 8.0, j as f64 / 8.0);
                        assert!(q.z.abs() < 1e-9, "n={n} z={}", q.z);
                        assert!(s.normal(i as f64 / 8.0, j as f64 / 8.0).z > 0.99);
                    }
                }
            }
            assert!(f.seam_angle < 1e-6, "n={n} seam angle {}", f.seam_angle);
        }
    }

    /// A smooth saddle-like height field the boundary sits on.
    fn zf(x: f64, y: f64) -> f64 {
        0.25 * (x * x - y * y) + 0.1 * x * y
    }
    fn znormal(x: f64, y: f64) -> Vec3 {
        Vec3::new(-(0.5 * x + 0.1 * y), -(-0.5 * y + 0.1 * x), 1.0).normalize()
    }

    fn surface_edge(p: Vec3, q: Vec3) -> BSplineCurve {
        let pts: Vec<Vec3> = (0..=8)
            .map(|j| {
                let t = j as f64 / 8.0;
                let x = p.x + (q.x - p.x) * t;
                let y = p.y + (q.y - p.y) * t;
                Vec3::new(x, y, zf(x, y))
            })
            .collect();
        crate::fit::interpolate(&pts, 3, crate::fit::Param::Uniform).unwrap()
    }

    fn saddle_pts(n: usize) -> Vec<Vec3> {
        polygon(n, 1.0, |_| 0.0).into_iter().map(|p| Vec3::new(p.x, p.y, zf(p.x, p.y))).collect()
    }

    #[test]
    fn non_planar_boundary_g0() {
        for n in [3usize, 4, 5, 6] {
            let pts = saddle_pts(n);
            let sides: Vec<NSidedSide> = (0..n).map(|i| NSidedSide::new(surface_edge(pts[i], pts[(i + 1) % n]))).collect();
            let f = check_fill(&sides, 1e-5);
            // the patch is genuinely non-planar
            let zs: Vec<f64> = f.patches.iter().flat_map(|s| (0..=8).map(move |i| s.eval(i as f64 / 8.0, 0.5).z)).collect();
            let span = zs.iter().cloned().fold(f64::MIN, f64::max) - zs.iter().cloned().fold(f64::MAX, f64::min);
            assert!(span > 1e-3);
            assert!(f.seam_angle < 0.2, "n={n} seam angle {}", f.seam_angle);
        }
    }

    #[test]
    fn supplied_tangents_give_g1_on_the_boundary_and_seams() {
        for n in [3usize, 4, 5, 6] {
            let pts = saddle_pts(n);
            let mut sides = vec![];
            for i in 0..n {
                let c = surface_edge(pts[i], pts[(i + 1) % n]);
                let (t0, t1) = c.domain();
                let nf = |t: f64| {
                    let q = c.eval(t0 + t * (t1 - t0));
                    znormal(q.x, q.y)
                };
                let cross = cross_from_normals(&c, &|t| nf((t - t0) / (t1 - t0)), 0.5, Some(Vec3::new(0.0, 0.0, zf(0.0, 0.0)))).unwrap();
                sides.push(NSidedSide::with_cross(c, cross));
            }
            let f = check_fill(&sides, 1e-5);
            // patch normals on the boundary sides against the surface normal, away from the corners
            let mut worst: f64 = 0.0;
            for k in 0..n {
                for j in 1..20 {
                    let u = j as f64 / 20.0;
                    let q = f.patches[k].eval(u, 0.0);
                    let nb = f.patches[k].normal(u, 0.0);
                    worst = worst.max(angle(nb, znormal(q.x, q.y)));
                }
            }
            assert!(worst < 0.05, "n={n} boundary normal angle {worst}");
            assert!(f.seam_angle < 0.05, "n={n} seam normal angle {}", f.seam_angle);
        }
    }

    #[test]
    fn vertical_walls_leave_vertically_and_bulge() {
        // a square tube rim: the neighbours are vertical, the patch is a dome
        let pts = polygon(4, 1.0, |_| 0.0);
        let n = pts.len();
        let mut sides = vec![];
        for i in 0..n {
            let c = segment(pts[i], pts[(i + 1) % n], 0.0, 1.0);
            let cross = segment(Vec3::new(0.0, 0.0, 0.8), Vec3::new(0.0, 0.0, 0.8), 0.0, 1.0);
            sides.push(NSidedSide::with_cross(c, cross));
        }
        let f = check_fill(&sides, 1e-6);
        for k in 0..n {
            let nb = f.patches[k].normal(0.5, 0.0);
            assert!(nb.z.abs() < 0.2, "boundary normal {nb:?}");
            assert!(f.patches[k].eval(1.0, 1.0).z > 0.2);
        }
        assert!(f.seam_angle < 0.05, "seam angle {}", f.seam_angle);
    }

    #[test]
    fn bad_input_is_an_error() {
        let pts = polygon(4, 1.0, |_| 0.0);
        let good = line_sides(&pts);
        assert!(n_sided_patch(&good[..2], 1e-6).is_err());
        assert!(n_sided_patch(&[], 1e-6).is_err());
        assert!(n_sided_patch(&good, 0.0).is_err());
        assert!(n_sided_patch(&good, f64::NAN).is_err());
        assert!(n_sided_patch(&good, -1.0).is_err());
        // an open loop (a gap)
        let mut gap = good.clone();
        gap[2] = NSidedSide::new(segment(pts[2] + Vec3::new(0.1, 0.0, 0.0), pts[3], 0.0, 1.0));
        assert!(matches!(n_sided_patch(&gap, 1e-6), Err(NSidedError::Input(_))));
        // a zero-length side
        let mut zero = good.clone();
        zero[1] = NSidedSide::new(segment(pts[1], pts[1], 0.0, 1.0));
        assert!(n_sided_patch(&zero, 1e-6).is_err());
        // non-finite coordinates
        let mut nan = good.clone();
        nan[0] = NSidedSide::new(segment(Vec3::new(f64::NAN, 0.0, 0.0), pts[1], 0.0, 1.0));
        assert!(n_sided_patch(&nan, 1e-6).is_err());
        // too many sides
        let many = polygon(MAX_SIDES + 1, 1.0, |_| 0.0);
        assert!(n_sided_patch(&line_sides(&many), 1e-6).is_err());
        // a cross tangent along its boundary
        let mut along = good.clone();
        along[0] = NSidedSide::with_cross(good[0].curve.clone(), segment(Vec3::new(1.0, 0.0, 0.0), Vec3::new(1.0, 0.0, 0.0), 0.0, 1.0));
        assert!(n_sided_patch(&along, 1e-6).is_err());
        // all points on a line: degenerate, no panic
        let line: Vec<Vec3> = (0..4).map(|i| Vec3::new(i as f64, 0.0, 0.0)).collect();
        let _ = n_sided_patch(&line_sides(&line), 1e-6);
        // a bow tie (self crossing) loop must not panic
        let bow = [Vec3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 1.0, 0.0), Vec3::new(1.0, 0.0, 0.0), Vec3::new(0.0, 1.0, 0.0)];
        let _ = n_sided_patch(&line_sides(&bow), 1e-6);
    }
}
