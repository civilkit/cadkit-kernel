//! Loft centre line (ck-ops/src/loft/centre.rs).
//!
//! A centre line is a planar open chain (the *spine*) that the loft follows:
//! the sections between the profiles are carried along it like beads on a
//! wire (hulls, bottles, ducts, handles).
//!
//! # The method (moving frame carrying local coordinates)
//!
//! Every profile `i` crosses the spine at arc position `s_i` (the point where
//! the spine pierces the profile's plane). At every arc position `s` the spine
//! has the orthonormal frame `F(s) = (T, N, B)`: unit tangent, in-plane normal
//! `N = B x T` and the (constant) normal `B` of the spine's plane. A profile
//! point `P` of station `i` gets its **local coordinates**
//! `q_i(P) = ((P - S(s_i)) . T_i, (P - S(s_i)) . N_i, (P - S(s_i)) . B)`.
//! The point of the loft at arc position `s` is
//!
//! ```text
//!     P(s) = S(s) + F(s) . q(s)        q(s) = natural cubic spline through (s_i, q_i)
//! ```
//!
//! (two profiles: `q` is the straight blend). So a profile is not rotated
//! rigidly: its local coordinates, including the component along the spine
//! tangent, are interpolated, which is why a profile that is not perpendicular
//! to the spine keeps its own plane at its end and the sections in between turn
//! gradually toward the spine's direction. The end profiles are pinned exactly
//! (`P(s_i)` is the profile point), and for a straight spine `P(s)` is the plain
//! loft (a straight centre line, slanted or not, changes nothing).
//!
//! This reproduces the rails of the oracle's `centreLine` loft to the
//! tessellation tolerance (arcs and gentle splines, 2..4 profiles, profiles off
//! the spine, a spine longer than the profile spacing; see
//! `tools/loft_centre_check.mjs`). The rail curves in between come from dense
//! intermediate stations ([`expand`]) fed to the ordinary station machinery
//! (natural cubic rails through the stations, side faces interpolating the
//! stations' control points), so the profile curves stay exact iso-curves.
//!
//! Rational pieces (a circle paired with a polygon) interpolate the homogeneous
//! local coordinates `(w q, w)`, the way the plain loft interpolates homogeneous
//! control points. `startNormal` / `endNormal` (tension `t`) is the slope
//! `q' = (t - 1, 0, 0)` of the local coordinates at that end: the slope the
//! plain loft has for a profile square to the spine (the oracle's bent
//! `startNormal` volume is the bow volume times the plain `startNormal` ratio).
//!
//! # Where the spine meets a profile
//!
//! `s_i` is the first crossing of the profile's plane at or after the previous
//! profile's `s`. A spine that stops short of the plane is accepted when the line
//! from its nearer end along the spine's direction reaches the plane within the
//! profile's circumradius about its centroid (observed on the oracle: the spine
//! end then stands in for the crossing); beyond that the loft is refused:
//! `loftCentreLineReach`. This also holds for a straight spine.
//!
//! # Errors
//!
//! * `loftCentreLineKink` (checked by the caller from [`Spine::kinks`]): a
//!   corner between sketch entities (tangent break above ~5 degrees).
//! * `loftCentreLineReach {detail}`: the spine does not reach a profile.
//! * `loftCentreLineTight {detail}`: the spine bends tighter than the
//!   sections allow: somewhere along it `|curvature| * (profile extent toward the
//!   centre of curvature) >= 1` (the oracle's rule; thresholds match to 1 percent
//!   on arcs), or neighbouring sections cross each other (the oracle returns a
//!   self-crossing solid there, we refuse it: [`sections_cross`]). `detail` is
//!   the largest distance of a profile point from the spine, in the plane
//!   normal to it, mm (the radius of curvature the sections would need).
//! * `loftCentreLineApex`, `loftCentreLineGuides`: a curved centre line cannot
//!   be combined with a point (apex) end section or with guides (both
//!   unsupported; the straight centre line is fine with both).

use super::stations::{Chain, Geom3, Piece, Stations};
use super::{compat, LoftError};
use ck_geom::nurbs::bspline::BSplineCurve;
use ck_geom::Vec3;

/// `loftCentreLineKink`: the chain has a corner (pieces not tangential).
pub const KINK: &str = "loftCentreLineKink";
/// `loftCentreLineTight`: bends too tightly for the sections.
pub const TIGHT: &str = "loftCentreLineTight";
/// `loftCentreLineReach`: the centre line does not reach a profile.
pub const REACH: &str = "loftCentreLineReach";
/// `loftCentreLineApex`: a curved centre line with a point (apex) end.
pub const APEX: &str = "loftCentreLineApex";
/// `loftCentreLineGuides`: a curved centre line together with guides.
pub const GUIDES: &str = "loftCentreLineGuides";

/// Max joint angle still considered tangential (about 5 degrees).
const KINK_ANGLE: f64 = 0.087;
/// Relative collinearity slack for the straightness test.
const STRAIGHT_EPS: f64 = 1e-7;

/// Sampled centre line in world space (dense polyline). `joints` are the
/// indices where one sketch entity ends and the next begins (kink checks);
/// `normal` is the normal of the sketch plane (the spine is planar).
#[derive(Debug, Clone)]
pub struct CentreIn {
    pub pts: Vec<Vec3>,
    /// Sample indices of entity joints (interior vertices of the chain).
    pub joints: Vec<usize>,
    pub normal: Vec3,
}

/// The analysed centre line: arc-length parametrised curve with frames.
#[derive(Debug, Clone)]
pub struct Spine {
    /// Cumulative arc length of the samples.
    s: Vec<f64>,
    p: Vec<Vec3>,
    /// Unit tangents at the samples.
    t: Vec<Vec3>,
    /// Normal of the spine's plane.
    pub b: Vec3,
    /// Corner angles (radians) at entity joints above the kink threshold.
    pub kinks: Vec<f64>,
    /// Every sample lies on the chord line: the loft is unchanged.
    pub straight: bool,
}

/// The frame of the spine at one arc position.
#[derive(Debug, Clone, Copy)]
pub struct Frame3 {
    pub p: Vec3,
    pub t: Vec3,
    pub n: Vec3,
    pub b: Vec3,
}

impl Frame3 {
    /// Local coordinates `(along T, along N, along B)` of a world point.
    pub fn local(&self, x: Vec3) -> [f64; 3] {
        let d = x - self.p;
        [d.dot(self.t), d.dot(self.n), d.dot(self.b)]
    }
    pub fn world(&self, q: [f64; 3]) -> Vec3 {
        self.p + self.t * q[0] + self.n * q[1] + self.b * q[2]
    }
}

impl Spine {
    /// Analyse `cin`. `None` when it has no usable length.
    pub fn new(cin: &CentreIn) -> Option<Spine> {
        // drop coincident consecutive samples (entity joints repeat their end point), remapping joints
        let mut p: Vec<Vec3> = vec![];
        let mut map: Vec<usize> = vec![];
        for q in &cin.pts {
            if p.last().map_or(true, |l| l.dist(*q) > 1e-12) {
                p.push(*q);
            }
            map.push(p.len() - 1);
        }
        if p.len() < 2 {
            return None;
        }
        let b = cin.normal.try_normalize()?;
        let mut s = vec![0.0];
        for i in 1..p.len() {
            let l = s[i - 1] + p[i - 1].dist(p[i]);
            s.push(l);
        }
        // tangents: three-point differences on the arc-length nodes (second order, also at the ends: a chord
        // direction there would be off by half the angle between samples and shift the pinned end frames)
        let n = p.len();
        let mut t = Vec::with_capacity(n);
        for i in 0..n {
            let d = if n == 2 {
                p[1] - p[0]
            } else if i == 0 {
                let (h1, h2) = (s[1] - s[0], s[2] - s[1]);
                p[0] * (-(2.0 * h1 + h2) / (h1 * (h1 + h2))) + p[1] * ((h1 + h2) / (h1 * h2)) - p[2] * (h1 / (h2 * (h1 + h2)))
            } else if i + 1 == n {
                let (h1, h2) = (s[n - 2] - s[n - 3], s[n - 1] - s[n - 2]);
                p[n - 3] * (h2 / (h1 * (h1 + h2))) - p[n - 2] * ((h1 + h2) / (h1 * h2)) + p[n - 1] * ((h1 + 2.0 * h2) / (h2 * (h1 + h2)))
            } else {
                let (h1, h2) = (s[i] - s[i - 1], s[i + 1] - s[i]);
                p[i - 1] * (-h2 / (h1 * (h1 + h2))) + p[i] * ((h2 - h1) / (h1 * h2)) + p[i + 1] * (h1 / (h2 * (h1 + h2)))
            };
            t.push(d.try_normalize()?);
        }
        let a = p[0];
        let chord = (p[n - 1] - a).try_normalize();
        let len = s[n - 1];
        let straight = chord.map_or(false, |d| p.iter().all(|q| (*q - a).cross(d).len() < STRAIGHT_EPS * (1.0 + len)) && (p[n - 1] - a).dot(d) > 0.0);
        // kinks at the entity joints
        let mut kinks = vec![];
        for &j in &cin.joints {
            let j = map.get(j).copied().unwrap_or(0);
            if j == 0 || j + 1 >= n {
                continue;
            }
            if let (Some(d0), Some(d1)) = ((p[j] - p[j - 1]).try_normalize(), (p[j + 1] - p[j]).try_normalize()) {
                let ang = d0.dot(d1).clamp(-1.0, 1.0).acos();
                if ang > KINK_ANGLE {
                    kinks.push(ang);
                }
            }
        }
        Some(Spine { s, p, t, b, kinks, straight })
    }

    /// Total arc length.
    pub fn length(&self) -> f64 {
        *self.s.last().unwrap_or(&0.0)
    }

    pub fn reversed(&self) -> Spine {
        let len = self.length();
        Spine {
            s: self.s.iter().rev().map(|x| len - x).collect(),
            p: self.p.iter().rev().copied().collect(),
            t: self.t.iter().rev().map(|d| -*d).collect(),
            b: self.b,
            kinks: self.kinks.clone(),
            straight: self.straight,
        }
    }

    fn seg(&self, s: f64) -> usize {
        let i = self.s.partition_point(|&x| x <= s);
        i.clamp(1, self.s.len() - 1) - 1
    }

    /// Point and unit tangent at arc position `s` (cubic Hermite between samples).
    pub fn at(&self, s: f64) -> (Vec3, Vec3) {
        let s = s.clamp(0.0, self.length());
        let i = self.seg(s);
        let h = self.s[i + 1] - self.s[i];
        if h < 1e-300 {
            return (self.p[i], self.t[i]);
        }
        let u = ((s - self.s[i]) / h).clamp(0.0, 1.0);
        let (u2, u3) = (u * u, u * u * u);
        let (h00, h10, h01, h11) = (2.0 * u3 - 3.0 * u2 + 1.0, u3 - 2.0 * u2 + u, -2.0 * u3 + 3.0 * u2, u3 - u2);
        let pos = self.p[i] * h00 + self.t[i] * (h10 * h) + self.p[i + 1] * h01 + self.t[i + 1] * (h11 * h);
        let (d00, d10, d01, d11) = (6.0 * u2 - 6.0 * u, 3.0 * u2 - 4.0 * u + 1.0, -6.0 * u2 + 6.0 * u, 3.0 * u2 - 2.0 * u);
        let d = (self.p[i] * d00 + self.t[i] * (d10 * h) + self.p[i + 1] * d01 + self.t[i + 1] * (d11 * h)) / h;
        (pos, d.try_normalize().unwrap_or(self.t[i]))
    }

    /// The frame at arc position `s`.
    pub fn frame(&self, s: f64) -> Frame3 {
        let (p, t) = self.at(s);
        let n = self.b.cross(t).try_normalize().unwrap_or(Vec3::new(0.0, 0.0, 0.0));
        Frame3 { p, t, n, b: self.b }
    }

    /// Signed turning rate `omega = B . (T x T')` (radians per mm, positive
    /// when the spine turns toward `N`) at arc position `s`.
    pub fn omega(&self, s: f64) -> f64 {
        let i = self.seg(s.clamp(0.0, self.length()));
        let h = self.s[i + 1] - self.s[i];
        if h < 1e-300 {
            return 0.0;
        }
        self.b.dot(self.t[i].cross(self.t[i + 1])) / h
    }

    /// Total absolute turning between two arc positions (radians).
    pub fn turning(&self, s0: f64, s1: f64) -> f64 {
        let (i0, i1) = (self.seg(s0), self.seg(s1));
        (i0..=i1).map(|i| self.t[i].dot(self.t[i + 1]).clamp(-1.0, 1.0).acos()).sum()
    }

    /// Arc positions of `planes` (origin, normal, circumradius of the profile). Reverses the spine in place
    /// when it runs from the last profile toward the first. `Err(i)` = profile `i` out of reach.
    pub fn locate(&mut self, planes: &[(Vec3, Vec3, f64)]) -> Result<Vec<f64>, usize> {
        let n = planes.len();
        let d = |k: usize, q: Vec3| (q - planes[k].0).dot(planes[k].1) / planes[k].1.len().max(1e-300);
        let end = |sp: &Spine, k: usize, last: bool| d(k, if last { *sp.p.last().unwrap() } else { sp.p[0] }).abs();
        let fwd = end(self, 0, false) + end(self, n - 1, true);
        let rev = end(self, 0, true) + end(self, n - 1, false);
        if rev + 1e-9 < fwd {
            *self = self.reversed();
        }
        let mut out: Vec<f64> = vec![];
        for k in 0..n {
            let from = out.last().copied().unwrap_or(0.0);
            let dist: Vec<f64> = self.p.iter().map(|q| d(k, *q)).collect();
            // first zero or sign change at or after `from`
            let mut found = None;
            for i in 0..self.p.len() {
                if i + 1 < self.p.len() && self.s[i + 1] < from - 1e-12 {
                    continue;
                }
                if dist[i] == 0.0 && self.s[i] >= from - 1e-9 {
                    found = Some(self.s[i]);
                    break;
                }
                if i + 1 < self.p.len() && (dist[i] < 0.0) != (dist[i + 1] < 0.0) && dist[i + 1] != 0.0 {
                    let (mut lo, mut hi) = (self.s[i], self.s[i + 1]);
                    for _ in 0..60 {
                        let mid = 0.5 * (lo + hi);
                        if (d(k, self.at(mid).0) < 0.0) == (dist[i] < 0.0) {
                            lo = mid;
                        } else {
                            hi = mid;
                        }
                    }
                    let x = 0.5 * (lo + hi);
                    if x >= from - 1e-9 {
                        found = Some(x);
                        break;
                    }
                }
            }
            let s = match found {
                Some(s) => s,
                None => {
                    // no crossing: the nearer spine end stands in when the line from it along the spine's direction
                    // reaches the plane within the profile's circumradius
                    let ends = [(dist[0].abs(), 0.0, self.t[0]), (dist[dist.len() - 1].abs(), self.length(), *self.t.last().unwrap())];
                    let (gap, s, t) = if ends[0].0 <= ends[1].0 { ends[0] } else { ends[1] };
                    let along = gap / (t.dot(planes[k].1) / planes[k].1.len().max(1e-300)).abs().max(1e-12);
                    if along > planes[k].2 * (1.0 + 1e-9) + 1e-9 {
                        return Err(k);
                    }
                    s
                }
            };
            out.push(s.max(from));
        }
        if let Some(i) = out.windows(2).position(|w| w[1] - w[0] < 1e-9 * (1.0 + self.length())) {
            return Err(i + 1);
        }
        Ok(out)
    }
}

/// Cubic spline through `(xs, ys)`: natural ends (linear for two points), or a given first derivative at either
/// end (`d0`, `d1`).
struct Natural {
    xs: Vec<f64>,
    ys: Vec<f64>,
    m: Vec<f64>,
}

impl Natural {
    fn new(xs: &[f64], ys: Vec<f64>) -> Natural {
        Natural::with_ends(xs, ys, None, None)
    }

    fn with_ends(xs: &[f64], ys: Vec<f64>, d0: Option<f64>, d1: Option<f64>) -> Natural {
        let n = xs.len();
        let mut m = vec![0.0; n];
        if n >= 2 && (n > 2 || d0.is_some() || d1.is_some()) {
            let h: Vec<f64> = (0..n - 1).map(|i| xs[i + 1] - xs[i]).collect();
            // tridiagonal system for the second derivatives: rows (sub, diag, super | rhs)
            let (mut a, mut b, mut c, mut d) = (vec![0.0; n], vec![1.0; n], vec![0.0; n], vec![0.0; n]);
            if let Some(d0) = d0 {
                b[0] = 2.0 * h[0];
                c[0] = h[0];
                d[0] = 6.0 * ((ys[1] - ys[0]) / h[0] - d0);
            }
            for i in 1..n - 1 {
                a[i] = h[i - 1];
                b[i] = 2.0 * (h[i - 1] + h[i]);
                c[i] = h[i];
                d[i] = 6.0 * ((ys[i + 1] - ys[i]) / h[i] - (ys[i] - ys[i - 1]) / h[i - 1]);
            }
            if let Some(d1) = d1 {
                a[n - 1] = h[n - 2];
                b[n - 1] = 2.0 * h[n - 2];
                d[n - 1] = 6.0 * (d1 - (ys[n - 1] - ys[n - 2]) / h[n - 2]);
            }
            for i in 1..n {
                let w = a[i] / b[i - 1];
                b[i] -= w * c[i - 1];
                d[i] -= w * d[i - 1];
            }
            m[n - 1] = d[n - 1] / b[n - 1];
            for i in (0..n - 1).rev() {
                m[i] = (d[i] - c[i] * m[i + 1]) / b[i];
            }
        }
        Natural { xs: xs.to_vec(), ys, m }
    }

    fn eval(&self, x: f64) -> f64 {
        let n = self.xs.len();
        let i = self.xs.partition_point(|&v| v <= x).clamp(1, n - 1) - 1;
        let h = self.xs[i + 1] - self.xs[i];
        let (a, b) = ((self.xs[i + 1] - x) / h, (x - self.xs[i]) / h);
        a * self.ys[i] + b * self.ys[i + 1] + ((a * a * a - a) * self.m[i] + (b * b * b - b) * self.m[i + 1]) * h * h / 6.0
    }
}

/// Largest distance of a station's sample points from their mean: the profile's circumradius (reach tolerance).
pub(super) fn circumradius(c: &Chain) -> f64 {
    let pts = samples(c);
    if pts.is_empty() {
        return 0.0;
    }
    let mut m = Vec3::new(0.0, 0.0, 0.0);
    for p in &pts {
        m = m + *p;
    }
    let m = m / pts.len() as f64;
    pts.iter().map(|p| p.dist(m)).fold(0.0, f64::max)
}

/// Sample points of a station (for the tightness test): 9 per piece.
fn samples(c: &Chain) -> Vec<Vec3> {
    let mut v = vec![];
    for p in &c.pieces {
        for k in 0..9 {
            v.push(p.g.at(k as f64 / 8.0));
        }
    }
    v
}

/// The sections of the loft follow the spine from the profile stations `st` (profile `i` pierces the spine at
/// `s[i]`): the stations plus dense intermediate stations on `P(s) = S(s) + F(s) q(s)`.
pub(super) fn expand(st: &Stations, sp: &Spine, s: &[f64], d0: Option<Vec3>, d1: Option<Vec3>) -> Result<(Stations, f64), LoftError> {
    let k = st.chains.len();
    let n = st.chains[0].pieces.len();
    let frames: Vec<Frame3> = s.iter().map(|&x| sp.frame(x)).collect();
    // (1) tightness: along the spine, |omega| * (extent of the sections toward the centre of curvature) < 1
    let pts_by_station: Vec<Vec<Vec3>> = st.chains.iter().map(samples).collect();
    let np = pts_by_station[0].len();
    let local: Vec<Vec<[f64; 3]>> = (0..k).map(|i| pts_by_station[i].iter().map(|p| frames[i].local(*p)).collect()).collect();
    let nat: Vec<[Natural; 3]> = (0..np).map(|j| [0, 1, 2].map(|c| Natural::new(s, (0..k).map(|i| local[i][j][c]).collect()))).collect();
    let (s0, s1) = (s[0], s[k - 1]);
    // the largest distance of a profile point from the spine, in the plane normal to it: the radius of curvature
    // the sections would need (the `detail` of `loftCentreLineTight`)
    let radial = local.iter().flat_map(|l| l.iter()).map(|q| (q[1] * q[1] + q[2] * q[2]).sqrt()).fold(0.0, f64::max);
    let probes = 400;
    for r in 0..=probes {
        let x = s0 + (s1 - s0) * (r as f64 / probes as f64);
        let w = sp.omega(x);
        if w.abs() < 1e-12 {
            continue;
        }
        let sg = w.signum();
        let ext = nat.iter().map(|q| sg * q[1].eval(x)).fold(f64::NEG_INFINITY, f64::max);
        if w.abs() * ext >= 1.0 {
            return Err(tight_error(radial));
        }
    }
    // (2) intermediate stations
    let pieces_nurbs: Vec<Vec<BSplineCurve>> = (0..n)
        .map(|j| compat(st.chains.iter().map(|c| c.pieces[j].g.to_nurbs()).collect()))
        .collect::<Result<_, _>>()?;
    // per piece, per control point: the homogeneous local coordinates (w q, w) as splines over the stations (the
    // plain loft interpolates homogeneous control points; with equal weights this is q itself).
    // `startNormal` / `endNormal` (tension t): in the plain loft the rails leave the profile along its normal at
    // speed t, which for a profile square to the spine is the local slope q' = (t - 1, 0, 0) (the spine already
    // carries the unit speed along T). The oracle applies the same local slope along a curved centre line
    // (volume of `bowStartNormal` = bow volume x the plain `startNormal` ratio 8207.14 / 7000, to 1e-6), so the
    // end constraint is the slope of the local coordinates, not a world direction.
    struct Ctl {
        q: [Natural; 3],
        w: Natural,
    }
    let end_rate = |d: Option<Vec3>| d.map(|v| [v.len() - 1.0, 0.0, 0.0]);
    let ctl: Vec<Vec<Ctl>> = pieces_nurbs
        .iter()
        .map(|cs| {
            (0..cs[0].ctrl.len())
                .map(|c| {
                    let wts: Vec<f64> = (0..k).map(|i| cs[i].weight(c)).collect();
                    let q: Vec<[f64; 3]> = (0..k).map(|i| frames[i].local(cs[i].ctrl[c])).collect();
                    let (r0, r1) = (end_rate(d0), end_rate(d1));
                    Ctl {
                        q: [0, 1, 2].map(|d| Natural::with_ends(s, (0..k).map(|i| wts[i] * q[i][d]).collect(), r0.map(|r| wts[0] * r[d]), r1.map(|r| wts[k - 1] * r[d]))),
                        w: Natural::new(s, wts),
                    }
                })
                .collect()
        })
        .collect();
    let mut chains: Vec<Chain> = vec![];
    let mut dense: Vec<bool> = vec![];
    for i in 0..k {
        chains.push(st.chains[i].clone());
        dense.push(false);
        if i + 1 == k {
            break;
        }
        let turn = sp.turning(s[i], s[i + 1]);
        let m = ((turn / 0.05).ceil() as usize).clamp(8, 64);
        for a in 1..m {
            let x = s[i] + (s[i + 1] - s[i]) * (a as f64 / m as f64);
            let fr = sp.frame(x);
            let mut pieces = vec![];
            for j in 0..n {
                let proto = &pieces_nurbs[j][0];
                let rational = proto.weights.is_some();
                let ctrl: Vec<Vec3> = ctl[j].iter().map(|c| { let w = c.w.eval(x); fr.world([c.q[0].eval(x) / w, c.q[1].eval(x) / w, c.q[2].eval(x) / w]) }).collect();
                let curve = if rational {
                    BSplineCurve::new_rational(proto.degree, proto.knots.clone(), ctrl, ctl[j].iter().map(|c| c.w.eval(x)).collect())
                } else {
                    BSplineCurve::new(proto.degree, proto.knots.clone(), ctrl)
                }
                .map_err(|e| LoftError::failed(&format!("the centre-line station could not be built ({e})"), "simplify the sections"))?;
                let (u0, u1) = curve.domain();
                pieces.push(Piece { tag: st.chains[i].pieces[j].tag.clone(), g: Geom3::Spline { curve, u0, u1 } });
            }
            chains.push(Chain { closed: st.chains[i].closed, pieces });
            dense.push(true);
        }
    }
    if sections_cross(&chains, sp.b) {
        return Err(tight_error(radial));
    }
    let cen: Vec<Vec3> = chains.iter().map(|c| c.centroid()).collect();
    let mut ts = vec![0.0];
    for w in cen.windows(2) {
        let last = *ts.last().unwrap();
        ts.push(last + w[0].dist(w[1]));
    }
    Ok((Stations { closed: st.closed, tags: st.tags.clone(), chains, ts, dense }, radial))
}

/// Do two neighbouring sections cross each other? (The map from the profile to the sections folds: the oracle
/// builds a self-crossing solid there, we refuse it.) Each station's outline is cut by the planes `B = level`
/// (five levels inside the B range common to both stations); the chords cut from neighbouring stations must not
/// cross. Only outlines cut in exactly two points (convex at that level) are compared.
fn sections_cross(chains: &[Chain], b: Vec3) -> bool {
    let outlines: Vec<Vec<Vec3>> = chains.iter().map(samples).collect();
    // an orthonormal basis of the plane normal to B
    let e1 = if b.x.abs() < 0.9 { Vec3::new(1.0, 0.0, 0.0) } else { Vec3::new(0.0, 1.0, 0.0) };
    let e1 = (e1 - b * e1.dot(b)).try_normalize().unwrap_or(e1);
    let e2 = b.cross(e1);
    let chord = |pts: &[Vec3], level: f64| -> Option<[(f64, f64); 2]> {
        let mut hits = vec![];
        for w in pts.windows(2) {
            let (a, c) = (w[0].dot(b) - level, w[1].dot(b) - level);
            if (a < 0.0) != (c < 0.0) && a != c {
                let p = w[0] + (w[1] - w[0]) * (a / (a - c));
                hits.push((p.dot(e1), p.dot(e2)));
            }
        }
        // a closed outline repeats its first point at the end; hits at a shared vertex show up twice at most
        hits.dedup_by(|x, y| (x.0 - y.0).abs() < 1e-9 && (x.1 - y.1).abs() < 1e-9);
        (hits.len() == 2).then(|| [hits[0], hits[1]])
    };
    let orient = |a: (f64, f64), b: (f64, f64), c: (f64, f64)| (b.0 - a.0) * (c.1 - a.1) - (b.1 - a.1) * (c.0 - a.0);
    for w in outlines.windows(2) {
        let range = |pts: &[Vec3]| pts.iter().fold((f64::INFINITY, f64::NEG_INFINITY), |r, p| (r.0.min(p.dot(b)), r.1.max(p.dot(b))));
        let (ra, rb) = (range(&w[0]), range(&w[1]));
        let (lo, hi) = (ra.0.max(rb.0), ra.1.min(rb.1));
        if hi - lo < 1e-9 {
            continue;
        }
        for f in [0.1, 0.3, 0.5, 0.7, 0.9] {
            let level = lo + (hi - lo) * f;
            let (Some(c0), Some(c1)) = (chord(&w[0], level), chord(&w[1], level)) else { continue };
            let eps = 1e-9 * (1.0 + (c0[0].0 - c0[1].0).abs() + (c0[0].1 - c0[1].1).abs());
            let (o1, o2) = (orient(c0[0], c0[1], c1[0]), orient(c0[0], c0[1], c1[1]));
            let (o3, o4) = (orient(c1[0], c1[1], c0[0]), orient(c1[0], c1[1], c0[1]));
            if o1 * o2 < -eps * eps && o3 * o4 < -eps * eps {
                return true;
            }
        }
    }
    false
}

/// `loftCentreLineTight {detail}`: `radial` is the radius of curvature (mm) the sections would need.
pub(super) fn tight_error(radial: f64) -> LoftError {
    LoftError::with(
        TIGHT,
        &format!("{radial:.2}"),
        "the centre line bends too tightly for the sections (they would cross): use a larger bend radius (a flatter curve), smaller sections, or sections closer to square with the centre line",
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Circular arc in the x-y plane bowing toward +x: ends (0, z0) and (0, z1), bow (maximum x) `bow`.
    fn arc_in(bow: f64, z0: f64, z1: f64, n: usize) -> CentreIn {
        let (mid, h) = (0.5 * (z0 + z1), 0.5 * (z1 - z0));
        let r = (bow * bow + h * h) / (2.0 * bow);
        let cx = bow - r;
        let a1 = (h / r).asin();
        let pts: Vec<Vec3> = (0..=n)
            .map(|i| {
                let a = -a1 + 2.0 * a1 * (i as f64 / n as f64);
                Vec3::new(cx + r * a.cos(), mid + r * a.sin(), 0.0)
            })
            .collect();
        CentreIn { pts, joints: vec![], normal: Vec3::new(0.0, 0.0, 1.0) }
    }

    #[test]
    fn straight_is_flagged() {
        let pts: Vec<Vec3> = (0..=32).map(|k| Vec3::new(0.0, 30.0 * k as f64 / 32.0, 0.0)).collect();
        let sp = Spine::new(&CentreIn { pts, joints: vec![], normal: Vec3::new(0.0, 0.0, 1.0) }).unwrap();
        assert!(sp.straight && sp.kinks.is_empty());
        let slanted: Vec<Vec3> = (0..=32).map(|k| Vec3::new(8.0, 30.0, 0.0) * (k as f64 / 32.0)).collect();
        assert!(Spine::new(&CentreIn { pts: slanted, joints: vec![], normal: Vec3::new(0.0, 0.0, 1.0) }).unwrap().straight);
    }

    #[test]
    fn corner_is_a_kink() {
        let mut pts: Vec<Vec3> = (0..=16).map(|k| Vec3::new(0.0, 15.0 * k as f64 / 16.0, 0.0)).collect();
        for k in 1..=16 {
            pts.push(Vec3::new(20.0 * k as f64 / 16.0, 15.0, 0.0));
        }
        let sp = Spine::new(&CentreIn { pts, joints: vec![16], normal: Vec3::new(0.0, 0.0, 1.0) }).unwrap();
        assert_eq!(sp.kinks.len(), 1);
    }

    #[test]
    fn arc_frame_and_length() {
        let sp = Spine::new(&arc_in(5.0, 0.0, 30.0, 256)).unwrap();
        // bow 5 over a 30 chord: R = 25, half angle asin(0.6)
        let want = 2.0 * 25.0 * 0.6f64.asin();
        assert!((sp.length() - want).abs() < 1e-3, "{} vs {}", sp.length(), want);
        let (p, t) = sp.at(sp.length() / 2.0);
        assert!((p.x - 5.0).abs() < 1e-3 && (p.y - 15.0).abs() < 1e-3 && t.y > 0.999, "{p:?} {t:?}");
        assert!((sp.omega(sp.length() / 2.0).abs() - 1.0 / 25.0).abs() < 1e-3);
    }

    #[test]
    fn locate_crossings_and_short_end() {
        let mut sp = Spine::new(&arc_in(5.0, 0.0, 30.0, 256)).unwrap();
        let planes = [(Vec3::ZERO, Vec3::new(0.0, 1.0, 0.0), 14.14), (Vec3::new(0.0, 30.0, 0.0), Vec3::new(0.0, 1.0, 0.0), 7.07)];
        let s = sp.locate(&planes).unwrap();
        assert!(s[0] < 1e-3 && (s[1] - sp.length()).abs() < 1e-3);
        // reversed spine is turned around
        let mut r = sp.reversed();
        let s = r.locate(&planes).unwrap();
        assert!(s[0] < 1e-3 && (s[1] - r.length()).abs() < 1e-3);
        // a spine that stops 20 short of the second plane is out of reach (circumradius 7.07)
        let mut short = Spine::new(&arc_in(2.0, 0.0, 10.0, 64)).unwrap();
        assert_eq!(short.locate(&planes), Err(1));
        // 5 short is within the circumradius: the end stands in
        let mut near = Spine::new(&arc_in(2.0, 0.0, 25.0, 64)).unwrap();
        let s = near.locate(&planes).unwrap();
        assert!((s[1] - near.length()).abs() < 1e-6);
    }

    #[test]
    fn natural_spline_matches_known_values() {
        // y = x^2 sampled at 0,1,2: natural spline between is not x^2, but two points are linear
        let l = Natural::new(&[0.0, 2.0], vec![1.0, 5.0]);
        assert!((l.eval(1.0) - 3.0).abs() < 1e-12);
        let n = Natural::new(&[0.0, 1.0, 2.0], vec![0.0, 1.0, 0.0]);
        assert!((n.eval(1.0) - 1.0).abs() < 1e-12 && n.eval(0.5) > 0.5 && n.eval(0.5) < 0.9);
    }
}
