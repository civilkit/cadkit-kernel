//! Volume, area, centroid and inertia (the `massProps` message), by the
//! divergence theorem evaluated exactly on the trimmed B-rep faces:
//! each face integral ∫∫_D F(σ, σu×σv) du dv is turned into a boundary
//! integral ∮ G dv (Green's theorem, G = ∫_{u*}^{u} F du) over the face's
//! coedge pcurves, both integrals by composite Gauss-Legendre. Analytic
//! faces with exact pcurves reach machine precision (the oracle's numbers
//! are exact too: cylinder volume = π r² h to the last digit).
//!
//! Wire shape (pinned by oracle probes, `{type:"massProps", tag, bodies:
//! true, densities:[..]}`):
//! `{volume, area, com, principal, axes[, bodies:[{volume,area,com}]]
//!   [, byMass:{mass, com, principal, axes}]}` — principal moments
//! ascending (volume-weighted, density 1), `axes[i]` = unit eigenvector of
//! `principal[i]`; `bodies` per lump (wire lump order), `byMass` uses
//! `densities[lump]` (g/mm³, client table Xt).

use crate::body::{Body, FaceId, LumpId};
use ck_geom::curve2::Curve2;
use ck_geom::surface::Surface;
use ck_geom::{Vec2, Vec3};
use ck_json::{Json, Obj};

/// 8-point Gauss-Legendre on [-1, 1].
pub const GL: [(f64, f64); 8] = [
    (-0.960_289_856_497_536_2, 0.101_228_536_290_376_26),
    (-0.796_666_477_413_626_7, 0.222_381_034_453_374_47),
    (-0.525_532_409_916_329, 0.313_706_645_877_887_3),
    (-0.183_434_642_495_649_8, 0.362_683_783_378_362),
    (0.183_434_642_495_649_8, 0.362_683_783_378_362),
    (0.525_532_409_916_329, 0.313_706_645_877_887_3),
    (0.796_666_477_413_626_7, 0.222_381_034_453_374_47),
    (0.960_289_856_497_536_2, 0.101_228_536_290_376_26),
];

/// One boundary piece in uv: either an exact pcurve over [ta,tb] or a
/// polyline (fallback when a coedge has no pcurve).
enum Piece<'a> {
    Exact(&'a Curve2, f64, f64),
    Poly(Vec<Vec2>),
}

fn face_pieces(b: &Body, f: FaceId) -> Vec<Piece<'_>> {
    let mut out = vec![];
    for &l in &b.face(f).loops {
        let mut prev: Option<Vec2> = None;
        for &c in &b.lp(l).coedges {
            let (ta, tb) = b.coedge_params(c);
            match &b.coedge(c).pcurve {
                Some(pc) => {
                    prev = Some(pc.eval(tb));
                    out.push(Piece::Exact(pc, ta, tb));
                }
                None => {
                    let n = 48;
                    let mut pts = vec![];
                    for k in 0..=n {
                        let t = ta + (tb - ta) * k as f64 / n as f64;
                        let uv = b.coedge_uv(c, t, prev);
                        prev = Some(uv);
                        pts.push(uv);
                    }
                    out.push(Piece::Poly(pts));
                }
            }
        }
    }
    out
}

/// B-spline faces are piecewise polynomial: cut straight boundary pcurves
/// where they cross an interior knot line, so that every Gauss panel lies
/// in one polynomial span (loft side faces have knots at the stations).
fn split_at_knots<'a>(surf: &Surface, pieces: Vec<Piece<'a>>) -> Vec<Piece<'a>> {
    let Surface::BSpline(bs) = surf else { return pieces };
    let ((u0, u1), (v0, v1)) = bs.domain();
    let inner = |k: &[f64], lo: f64, hi: f64| -> Vec<f64> {
        let mut out: Vec<f64> = k.iter().copied().filter(|&x| x > lo && x < hi).collect();
        out.dedup();
        out
    };
    let (ku, kv) = (inner(&bs.knots_u, u0, u1), inner(&bs.knots_v, v0, v1));
    if ku.is_empty() && kv.is_empty() {
        return pieces;
    }
    let mut out = vec![];
    for p in pieces {
        match p {
            Piece::Exact(pc @ Curve2::Line { o, d }, ta, tb) => {
                let mut ts: Vec<f64> = vec![];
                for (ks, oo, dd) in [(&ku, o.x, d.x), (&kv, o.y, d.y)] {
                    if dd == 0.0 {
                        continue;
                    }
                    for &k in ks.iter() {
                        let t = (k - oo) / dd;
                        if t > ta.min(*&tb) && t < ta.max(tb) {
                            ts.push(t);
                        }
                    }
                }
                ts.sort_by(|a, b| a.partial_cmp(b).unwrap());
                if tb < ta {
                    ts.reverse();
                }
                let mut a = ta;
                for t in ts {
                    out.push(Piece::Exact(pc, a, t));
                    a = t;
                }
                out.push(Piece::Exact(pc, a, tb));
            }
            p => out.push(p),
        }
    }
    out
}

/// Outer panel count: angular parameters get ≤ π/8 panels (GL8 is then
/// exact to ~1e-16 for the trig-polynomial integrands of analytic faces).
fn panels_for(pc: &Curve2, ta: f64, tb: f64, ang: (bool, bool), curved: bool, spline: bool) -> usize {
    match pc {
        Curve2::Line { d, .. } => {
            let step = std::f64::consts::PI / 8.0;
            let mut n: f64 = if spline { 1.0 } else if curved { 4.0 } else { 2.0 };
            if ang.0 {
                n = n.max((d.x * (tb - ta)).abs() / step);
            }
            if ang.1 {
                n = n.max((d.y * (tb - ta)).abs() / step);
            }
            n.ceil() as usize
        }
        Curve2::Circle { .. } | Curve2::Ellipse { .. } => ((tb - ta).abs() / (std::f64::consts::PI / 8.0)).ceil().max(2.0) as usize,
        Curve2::BSpline(bs) => (bs.ctrl.len() * 2).clamp(4, 256),
    }
}

/// Which parameters are angles (trig dependence).
fn angular(s: &Surface) -> (bool, bool) {
    match s {
        Surface::Plane { .. } => (false, false),
        Surface::Cylinder { .. } | Surface::Cone { .. } | Surface::Revolution { .. } => (true, false),
        Surface::Sphere { .. } | Surface::Torus { .. } => (true, true),
        Surface::Extrusion { profile, .. } => (profile.period().is_some(), false),
        Surface::Offset { base, .. } => angular(base),
        Surface::BSpline(_) => (false, false),
    }
}

/// ∮ G dv over the face boundary = sense · ∫∫_D F(σ, σu×σv) du dv, i.e.
/// the integral of F against the OUTWARD area element. `res` ≥ 1 scales
/// panel counts (region.rs refines near-singular integrands with it).
pub fn face_integral<const K: usize>(b: &Body, f: FaceId, res: usize, integrand: &dyn Fn(Vec3, Vec3) -> [f64; K]) -> [f64; K] {
    let surf = &b.face(f).surface;
    let pieces = split_at_knots(surf, face_pieces(b, f));
    let mut acc = [0.0; K];
    if pieces.is_empty() {
        return acc;
    }
    // reference u* and u extent
    let first = match &pieces[0] {
        Piece::Exact(pc, ta, _) => pc.eval(*ta),
        Piece::Poly(p) => p[0],
    };
    let ustar = first.x;
    let (mut umin, mut umax) = (ustar, ustar);
    let (mut vmin, mut vmax) = (first.y, first.y);
    for p in &pieces {
        match p {
            Piece::Exact(pc, ta, tb) => {
                for k in 0..=8 {
                    let q = pc.eval(ta + (tb - ta) * k as f64 / 8.0);
                    umin = umin.min(q.x);
                    umax = umax.max(q.x);
                    vmin = vmin.min(q.y);
                    vmax = vmax.max(q.y);
                }
            }
            Piece::Poly(pts) => pts.iter().for_each(|q| {
                umin = umin.min(q.x);
                umax = umax.max(q.x);
                vmin = vmin.min(q.y);
                vmax = vmax.max(q.y);
            }),
        }
    }
    let width = (umax - umin).max(1e-12);
    let ang = angular(surf);
    let mut hu = width / (4.0 * res as f64);
    if ang.0 {
        hu = hu.min(std::f64::consts::PI / (8.0 * res as f64));
    }
    if matches!(surf, Surface::BSpline(_) | Surface::Revolution { .. } | Surface::Extrusion { .. }) {
        hu = hu.min(width / (16.0 * res as f64));
    }
    let curved = !matches!(surf, Surface::Plane { .. });
    // non-rational B-spline boundary lines are cut at the knots: pieces are short
    let spline = matches!(surf, Surface::BSpline(b) if b.weights.is_none());
    // B-spline faces: cut the u-integral at the interior knots (rational
    // quarter-turn arcs have C0 knots; a panel across one loses ~1e-4)
    let knots_u: Vec<f64> = match surf {
        Surface::BSpline(bs) => {
            let (a, z) = bs.domain().0;
            let mut k: Vec<f64> = bs.knots_u.iter().copied().filter(|&x| x > a && x < z).collect();
            k.dedup();
            k
        }
        _ => vec![],
    };
    // ∫ from u = a to u = c at fixed v, cut at the interior knots
    let seg_integral = |a0: f64, u: f64, v: f64| -> [f64; K] {
        let mut g = [0.0; K];
        if a0 == u {
            return g;
        }
        // cut points: a0, the interior knots strictly between a0 and u (in
        // travel order), u -- indexed, no allocation
        let (lo, hi) = (a0.min(u), a0.max(u));
        let (i0, i1) = (knots_u.partition_point(|&k| k <= lo), knots_u.partition_point(|&k| k < hi));
        let m_in = i1.saturating_sub(i0);
        let cut = |j: usize| -> f64 {
            if j == 0 {
                a0
            } else if j == m_in + 1 {
                u
            } else if u >= a0 {
                knots_u[i0 + j - 1]
            } else {
                knots_u[i1 - j]
            }
        };
        // non-rational B-spline: fixed-v evaluator (v basis once; v direction
        // collapsed per knot span); each cut segment lies in one polynomial
        // span, where two GL8 panels are exact for the area-element
        // polynomials and ~1e-13 for |Su x Sv|
        let vl = match surf {
            Surface::BSpline(bs) => bs.vline(v),
            _ => None,
        };
        for j in 0..=m_in {
            let (a, c) = (cut(j), cut(j + 1));
            let span = c - a;
            if span == 0.0 {
                continue;
            }
            let fast = vl.as_ref().and_then(|l| l.segment(0.5 * (a + c))).filter(|s| s.covers(a, c));
            let n = if fast.is_some() { 2 * res } else { ((span.abs() / hu).ceil() as usize).clamp(1, 512 * res) };
            let h = span / n as f64;
            for i in 0..n {
                let m = a + h * (i as f64 + 0.5);
                for &(x, w) in GL.iter() {
                    let s = m + 0.5 * h * x;
                    let (pt, nrm) = match &fast {
                        Some(sg) => {
                            let (p, du, dv) = sg.eval1(s);
                            (p, du.cross(dv))
                        }
                        None => {
                            let d = surf.derivs2(s, v);
                            (d.p, d.du.cross(d.dv))
                        }
                    };
                    let val = integrand(pt, nrm);
                    for k in 0..K {
                        g[k] += 0.5 * h * w * val[k];
                    }
                }
            }
        }
        g
    };
    // long multi-knot B-spline faces (screw sweeps): the inner integral over
    // u from u* would cost O(spans) per boundary sample. The integrand is a
    // low-degree polynomial in v on a span, so keep the cumulative integral
    // at every u knot for a few v nodes and Lagrange-interpolate in v
    let long_spline = knots_u.len() >= 32;
    let vtab: Option<(Vec<f64>, Vec<f64>, Vec<Vec<[f64; K]>>)> = match surf {
        Surface::BSpline(bs) if long_spline && bs.knots_v.windows(2).filter(|w| w[0] < w[1]).count() == 1 => {
            let ((du0, du1), (dv0, dv1)) = bs.domain();
            // Chebyshev-Lobatto nodes: exact for the polynomial integrands,
            // ~1e-12 for the smooth area element |Su x Sv|
            let nvn = 3 * bs.deg_v + 6;
            let vn: Vec<f64> = (0..nvn).map(|j| dv0 + (dv1 - dv0) * 0.5 * (1.0 - (std::f64::consts::PI * j as f64 / (nvn - 1) as f64).cos())).collect();
            let mut ks = vec![du0];
            ks.extend(knots_u.iter().copied());
            ks.push(du1);
            let cum: Vec<Vec<[f64; K]>> = vn
                .iter()
                .map(|&v| {
                    let mut c = vec![[0.0; K]];
                    for w in ks.windows(2) {
                        let d = seg_integral(w[0], w[1], v);
                        let mut g = *c.last().unwrap();
                        for k in 0..K {
                            g[k] += d[k];
                        }
                        c.push(g);
                    }
                    c
                })
                .collect();
            Some((ks, vn, cum))
        }
        _ => None,
    };
    let h_at = |u: f64, v: f64| -> [f64; K] {
        let (ks, vn, cum) = vtab.as_ref().unwrap();
        let k = (ks.partition_point(|&x| x <= u).max(1) - 1).min(ks.len() - 2);
        let mut g = [0.0; K];
        for j in 0..vn.len() {
            let mut w = 1.0;
            for m in 0..vn.len() {
                if m != j {
                    w *= (v - vn[m]) / (vn[j] - vn[m]);
                }
            }
            for i in 0..K {
                g[i] += w * cum[j][k][i];
            }
        }
        // the partial span is integrated directly at the true v (one
        // seg_integral instead of one per interpolation node)
        let part = seg_integral(ks[k], u, v);
        for i in 0..K {
            g[i] += part[i];
        }
        g
    };
    let period_tab: Option<(Vec<f64>, Vec<[f64; K]>)> = match (surf, surf.periods().0) {
        (Surface::Cylinder { .. } | Surface::Cone { .. }, Some(per)) if umax - umin > 1.5 * per && vmax > vmin => {
            let vn: Vec<f64> = (0..6).map(|j| vmin + (vmax - vmin) * j as f64 / 5.0).collect();
            let tab = vn.iter().map(|&v| seg_integral(ustar, ustar + per, v)).collect();
            Some((vn, tab))
        }
        _ => None,
    };
    // (a long face with several v spans: integrals checkpointed per v, the
    // v of a constant-v edge being equal to rounding)
    let ckpt: std::cell::RefCell<std::collections::HashMap<u64, Vec<(f64, [f64; K])>>> = Default::default();
    let inner = |u: f64, v: f64| -> [f64; K] {
        if u - ustar == 0.0 {
            return [0.0; K];
        }
        if long_spline && vtab.is_none() {
            let v = (v * 1e9).round() / 1e9;
            let mut map = ckpt.borrow_mut();
            let list = map.entry(v.to_bits()).or_insert_with(|| vec![(ustar, [0.0; K])]);
            let idx = list.partition_point(|e| e.0 < u);
            let cand = [idx.checked_sub(1), (idx < list.len()).then_some(idx)];
            let best = cand.iter().flatten().copied().min_by(|&x, &y| (list[x].0 - u).abs().partial_cmp(&(list[y].0 - u).abs()).unwrap()).unwrap();
            let (uc, gc) = list[best];
            if uc == u {
                return gc;
            }
            let d = seg_integral(uc, u, v);
            let mut g = gc;
            for k in 0..K {
                g[k] += d[k];
            }
            list.insert(idx, (u, g));
            return g;
        }
        if vtab.is_some() {
            let (a, b) = (h_at(u, v), h_at(ustar, v));
            let mut g = a;
            for i in 0..K {
                g[i] -= b[i];
            }
            return g;
        }
        // a face wound many times round its axis (cylinder cut by a
        // helical groove): whole periods integrate to the same value, a
        // polynomial in v for cylinders / cones (tabulated at a few nodes)
        if let (Some(per), Some((vn, tab))) = (surf.periods().0, period_tab.as_ref()) {
            if (u - ustar).abs() > 1.5 * per {
                let k = ((u - ustar) / per).trunc();
                let mut full = [0.0; K];
                for j in 0..vn.len() {
                    let mut w = 1.0;
                    for m in 0..vn.len() {
                        if m != j {
                            w *= (v - vn[m]) / (vn[j] - vn[m]);
                        }
                    }
                    for i in 0..K {
                        full[i] += w * tab[j][i];
                    }
                }
                let mut g = seg_integral(ustar, u - k * per, v);
                for i in 0..K {
                    g[i] += k * full[i];
                }
                return g;
            }
        }
        seg_integral(ustar, u, v)
    };
    let mut add = |uv: Vec2, dvdt: f64, wt: f64| {
        if dvdt == 0.0 {
            return;
        }
        let g = inner(uv.x, uv.y);
        for k in 0..K {
            acc[k] += wt * dvdt * g[k];
        }
    };
    for p in &pieces {
        match p {
            Piece::Exact(pc, ta, tb) => {
                let n = panels_for(pc, *ta, *tb, ang, curved, spline) * res;
                let h = (tb - ta) / n as f64;
                for i in 0..n {
                    let m = ta + h * (i as f64 + 0.5);
                    for &(x, w) in GL.iter() {
                        let t = m + 0.5 * h * x;
                        let d = pc.derivs(t, 1);
                        add(d[0], d[1].y, 0.5 * h * w);
                    }
                }
            }
            Piece::Poly(pts) => {
                for s in pts.windows(2) {
                    let (a, c) = (s[0], s[1]);
                    let dv = c.y - a.y;
                    for &(x, w) in GL.iter() {
                        let t = 0.5 * (x + 1.0);
                        add(a.lerp(c, t), dv, 0.5 * w);
                    }
                }
            }
        }
    }
    acc
}

/// Raw moments of a set of faces about `r` (density 1).
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Moments {
    pub volume: f64,
    pub area: f64,
    /// ∫ (p - r) dV
    pub first: Vec3,
    /// ∫ (p-r)_i (p-r)_j dV as [xx, yy, zz, xy, yz, xz]
    pub second: [f64; 6],
    pub r: Vec3,
}

impl Moments {
    pub fn com(&self) -> Vec3 {
        self.r + self.first / self.volume
    }
    /// Re-express about another reference point.
    pub fn about(&self, r: Vec3) -> Moments {
        let d = self.r - r; // p - r = (p - self.r) + d
        let (f, v, s) = (self.first, self.volume, self.second);
        let sec = [
            s[0] + 2.0 * d.x * f.x + v * d.x * d.x,
            s[1] + 2.0 * d.y * f.y + v * d.y * d.y,
            s[2] + 2.0 * d.z * f.z + v * d.z * d.z,
            s[3] + d.x * f.y + d.y * f.x + v * d.x * d.y,
            s[4] + d.y * f.z + d.z * f.y + v * d.y * d.z,
            s[5] + d.x * f.z + d.z * f.x + v * d.x * d.z,
        ];
        Moments { volume: v, area: self.area, first: f + d * v, second: sec, r }
    }
    pub fn scaled(&self, k: f64) -> Moments {
        let mut m = *self;
        m.volume *= k;
        m.area *= k;
        m.first = m.first * k;
        m.second.iter_mut().for_each(|x| *x *= k);
        m
    }
    pub fn add(&self, o: &Moments) -> Moments {
        let o = o.about(self.r);
        let mut s = self.second;
        for i in 0..6 {
            s[i] += o.second[i];
        }
        Moments { volume: self.volume + o.volume, area: self.area + o.area, first: self.first + o.first, second: s, r: self.r }
    }
    /// Inertia tensor about the centroid (rows).
    pub fn inertia(&self) -> [[f64; 3]; 3] {
        let c = self.about(self.com());
        let s = c.second;
        [[s[1] + s[2], -s[3], -s[5]], [-s[3], s[0] + s[2], -s[4]], [-s[5], -s[4], s[0] + s[1]]]
    }
}

/// Moments of the given faces (normally one lump / the whole body).
pub fn faces_moments(b: &Body, faces: &[FaceId], r: Vec3) -> Moments {
    let mut m = Moments { r, ..Default::default() };
    for &f in faces {
        let v = face_integral::<11>(b, f, 1, &|p, n| {
            let q = p - r;
            [
                q.dot(n) / 3.0,
                n.len(),
                0.5 * q.x * q.x * n.x,
                0.5 * q.y * q.y * n.y,
                0.5 * q.z * q.z * n.z,
                q.x * q.x * q.x * n.x / 3.0,
                q.y * q.y * q.y * n.y / 3.0,
                q.z * q.z * q.z * n.z / 3.0,
                0.5 * q.x * q.x * q.y * n.x,
                0.5 * q.y * q.y * q.z * n.y,
                0.5 * q.z * q.z * q.x * n.z,
            ]
        });
        let sg = if b.face(f).sense { 1.0 } else { -1.0 };
        m.volume += v[0];
        m.area += sg * v[1];
        m.first = m.first + Vec3::new(v[2], v[3], v[4]);
        for i in 0..6 {
            m.second[i] += v[5 + i];
        }
    }
    m
}

pub fn lump_faces(b: &Body, l: LumpId) -> Vec<FaceId> {
    let mut out = vec![];
    for &s in &b.lump(l).shells {
        if b.shell(s).alive {
            out.extend(b.shell(s).faces.iter().copied().filter(|&f| b.face(f).alive));
        }
    }
    out
}

fn ref_point(b: &Body) -> Vec3 {
    let (lo, hi) = b.bbox();
    if lo.is_finite() && hi.is_finite() {
        (lo + hi) * 0.5
    } else {
        Vec3::ZERO
    }
}

/// Per-lump moments (wire lump order) about a common reference point.
pub fn lump_moments(b: &Body) -> Vec<Moments> {
    let r = ref_point(b);
    b.lump_ids().map(|l| faces_moments(b, &lump_faces(b, l), r)).collect()
}

/// Signed volume of a set of faces (negative = inside-out / void shell).
pub fn signed_volume(b: &Body, faces: &[FaceId]) -> f64 {
    let r = ref_point(b);
    faces
        .iter()
        .map(|&f| face_integral::<1>(b, f, 1, &|p, n| [(p - r).dot(n) / 3.0])[0])
        .sum()
}

#[derive(Debug, Clone, PartialEq)]
pub struct MassProps {
    pub volume: f64,
    pub area: f64,
    pub com: Vec3,
    pub principal: [f64; 3],
    pub axes: [Vec3; 3],
}

impl MassProps {
    pub fn from_moments(m: &Moments) -> MassProps {
        let (principal, axes) = eigen_sym(m.inertia());
        MassProps { volume: m.volume, area: m.area, com: m.com(), principal, axes }
    }
}

pub fn mass_props(b: &Body) -> MassProps {
    let ms = lump_moments(b);
    let total = ms.iter().skip(1).fold(ms.first().copied().unwrap_or_default(), |a, m| a.add(m));
    MassProps::from_moments(&total)
}

/// The `massProps` reply `result` object.
pub fn mass_props_json(b: &Body, bodies: bool, densities: Option<&[f64]>) -> Json {
    let ms = lump_moments(b);
    let total = ms.iter().skip(1).fold(ms.first().copied().unwrap_or_default(), |a, m| a.add(m));
    let mp = MassProps::from_moments(&total);
    let v3 = crate::body::v3;
    let axes = |a: &[Vec3; 3]| Json::Arr(a.iter().map(|&x| v3(x)).collect());
    let mut o = Obj::new()
        .put("volume", mp.volume)
        .put("area", mp.area)
        .put("com", v3(mp.com))
        .put("principal", Json::Arr(mp.principal.iter().map(|&x| Json::Num(x)).collect()))
        .put("axes", axes(&mp.axes));
    if bodies {
        let arr = ms
            .iter()
            .map(|m| Obj::new().put("volume", m.volume).put("area", m.area).put("com", v3(m.com())).build())
            .collect();
        o = o.put("bodies", Json::Arr(arr));
    }
    if let Some(ds) = densities {
        // GUESS: missing density entries count as 1.
        let mut acc: Option<Moments> = None;
        for (i, m) in ms.iter().enumerate() {
            let w = m.scaled(ds.get(i).copied().unwrap_or(1.0));
            acc = Some(match acc {
                None => w,
                Some(a) => a.add(&w),
            });
        }
        if let Some(a) = acc {
            let p = MassProps::from_moments(&a);
            o = o.put(
                "byMass",
                Obj::new()
                    .put("mass", a.volume)
                    .put("com", v3(p.com))
                    .put("principal", Json::Arr(p.principal.iter().map(|&x| Json::Num(x)).collect()))
                    .put("axes", axes(&p.axes)),
            );
        }
    }
    o.build()
}

/// Symmetric 3×3 eigen-decomposition, decoded from the oracle (WAT f3939,
/// called by the massProps builder f3561): cyclic Jacobi over the pairs
/// (0,1), (0,2), (1,2), up to 64 sweeps, stopping only when
/// |a01|+|a02|+|a12| < 1e-300; a pair is skipped when |apq| < 1e-300;
/// θ = (aqq − app)/(apq + apq), t = sign(θ)/(|θ| + √(θ²+1)) (1 when θ = 0),
/// c = 1/√(t²+1), s = t·c; A ← JᵀAJ (columns, then rows), V ← VJ.
/// Eigenvalues sorted ascending (total order, stable); eigenvectors are the
/// columns of V scaled by 1/|v| — NO sign convention: the signs are those
/// the rotations produce (oracle rev90 / rev90flip axes
/// [.707,0,.707],[.707,0,−.707] / [−.707,0,.707],[.707,0,.707]). For
/// repeated moments (cylinder, sphere, torus) the result depends on the
/// rounding noise of the tensor (e.g. a 45° pair when two diagonal entries
/// are bit-equal), so it matches the oracle only as far as the integrated
/// tensor does.
pub fn eigen_sym(m: [[f64; 3]; 3]) -> ([f64; 3], [Vec3; 3]) {
    let mut a = m;
    let mut v = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    for _ in 0..64 {
        if a[0][1].abs() + a[0][2].abs() + a[1][2].abs() < 1e-300 {
            break;
        }
        for (p, q) in [(0, 1), (0, 2), (1, 2)] {
            if a[p][q].abs() < 1e-300 {
                continue;
            }
            let apq = a[p][q];
            let theta = (a[q][q] - a[p][p]) / (apq + apq);
            let sg = if theta.is_nan() { f64::NAN } else { 1f64.copysign(theta) };
            let t = if theta == 0.0 { 1.0 } else { sg / (theta.abs() + (theta * theta + 1.0).sqrt()) };
            let c = 1.0 / (t * t + 1.0).sqrt();
            let s = t * c;
            for k in 0..3 {
                let (akp, akq) = (a[k][p], a[k][q]);
                a[k][p] = c * akp - s * akq;
                a[k][q] = s * akp + c * akq;
            }
            for k in 0..3 {
                let (apk, aqk) = (a[p][k], a[q][k]);
                a[p][k] = c * apk - s * aqk;
                a[q][k] = s * apk + c * aqk;
            }
            for row in v.iter_mut() {
                let (vp, vq) = (row[p], row[q]);
                row[p] = c * vp - s * vq;
                row[q] = s * vp + c * vq;
            }
        }
    }
    let mut idx = [0usize, 1, 2];
    idx.sort_by(|&i, &j| a[i][i].total_cmp(&a[j][j]));
    let mut vals = [0.0; 3];
    let mut vecs = [Vec3::ZERO; 3];
    for (k, &i) in idx.iter().enumerate() {
        vals[k] = a[i][i];
        vecs[k] = Vec3::new(v[0][i], v[1][i], v[2][i]).normalize();
    }
    (vals, vecs)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::make::tests::{box_body, rect, top_frame};
    use crate::make::{extrude, revolve::revolve, ExtrudeOpts, Profile, ProfileLoop, ProfileRegion, ProfileSeg};
    use ck_geom::Frame;
    use std::f64::consts::{PI, TAU};

    fn close(a: f64, b: f64, rel: f64) -> bool {
        (a - b).abs() <= rel * b.abs().max(1.0)
    }
    fn vclose(a: Vec3, b: [f64; 3], tol: f64) -> bool {
        (a.x - b[0]).abs() < tol && (a.y - b[1]).abs() < tol && (a.z - b[2]).abs() < tol
    }

    #[test]
    fn box_mass_matches_oracle() {
        // oracle: volume 8000, area 2800, com [20,5,-10],
        // principal [333333.33, 1133333.33, 1333333.33], axes [[1,0,0],[0,0,1],[0,1,0]]
        let b = box_body();
        let m = mass_props(&b);
        assert!(close(m.volume, 8000.0, 1e-13), "{}", m.volume);
        assert!(close(m.area, 2800.0, 1e-13));
        assert!(vclose(m.com, [20.0, 5.0, -10.0], 1e-12));
        assert!(close(m.principal[0], 333333.3333333342, 1e-12));
        assert!(close(m.principal[1], 1133333.333333336, 1e-12));
        assert!(close(m.principal[2], 1333333.3333333367, 1e-12));
        assert!(vclose(m.axes[0], [1.0, 0.0, 0.0], 1e-12));
        assert!(vclose(m.axes[1], [0.0, 0.0, 1.0], 1e-12));
        assert!(vclose(m.axes[2], [0.0, 1.0, 0.0], 1e-12));
        // byMass with aluminium 0.0027 -> mass 21.6, principal[0] 900
        let j = mass_props_json(&b, true, Some(&[0.0027]));
        let bm = j.get("byMass").unwrap();
        assert!(close(bm.get_f64("mass").unwrap(), 21.6, 1e-13));
        assert!(close(bm.get_arr("principal").unwrap()[0].as_f64().unwrap(), 900.0, 1e-12));
        let keys: Vec<_> = j.as_obj().unwrap().iter().map(|(k, _)| k.as_str()).collect();
        assert_eq!(keys, ["volume", "area", "com", "principal", "axes", "bodies", "byMass"]);
    }

    #[test]
    fn cylinder_and_holes_mass() {
        let prof = Profile::new(
            top_frame(),
            vec![ProfileRegion {
                outer: ProfileLoop::new(vec![ProfileSeg::circle("e7", Vec2::new(5.0, 3.0), 10.0)]),
                holes: vec![],
            }],
        );
        let b = extrude("F2", &prof, ExtrudeOpts::blind(10.0)).unwrap();
        let m = mass_props(&b);
        // oracle: 3141.592653589795, area 1256.6370614359173, com [5,5,-3], principal [104719.755.., .., 157079.63..]
        assert!(close(m.volume, 3141.592653589795, 1e-13), "{}", m.volume);
        assert!(close(m.area, 1256.6370614359173, 1e-13));
        assert!(vclose(m.com, [5.0, 5.0, -3.0], 1e-12));
        assert!(close(m.principal[0], 104719.75511965966, 1e-12), "{:?} {:?}", m.principal, m.axes);
        assert!(close(m.principal[2], 157079.63267948956, 1e-12));

        // box3: outer 40x20 with circle r4 @ (30,10) and 10x10 square hole, depth 10
        let fr = Frame::identity();
        let hole = rect(["h1", "h2", "h3", "h4"], 5.0, 5.0, 15.0, 15.0).reversed();
        let circ = ProfileLoop::new(vec![ProfileSeg::circle("e5", Vec2::new(30.0, 10.0), 4.0).reversed()]);
        let prof = Profile::new(fr, vec![ProfileRegion { outer: rect(["e1", "e2", "e3", "e4"], 0.0, 0.0, 40.0, 20.0), holes: vec![circ, hole] }]);
        let b = extrude("F2", &prof, ExtrudeOpts::blind(10.0)).unwrap();
        let m = mass_props(&b);
        // oracle: 6497.345175425632, area 3150.79644737231, com [20.765459063659883,10,5],
        // principal [310467.2571635832, 956394.8009831763, 1158572.9718896654]
        assert!(close(m.volume, 6497.345175425632, 1e-13));
        assert!(close(m.area, 3150.79644737231, 1e-13));
        assert!(vclose(m.com, [20.765459063659883, 10.0, 5.0], 1e-11));
        assert!(close(m.principal[0], 310467.2571635832, 1e-12));
        assert!(close(m.principal[1], 956394.8009831763, 1e-12));
        assert!(close(m.principal[2], 1158572.9718896654, 1e-12));
    }

    #[test]
    fn draft_mass() {
        let prof = Profile::new(fr_id(), vec![ProfileRegion { outer: rect(["e1", "e2", "e3", "e4"], 0.0, 0.0, 40.0, 20.0), holes: vec![] }]);
        let mut o = ExtrudeOpts::blind(10.0);
        o.draft = 10f64.to_radians();
        let b = extrude("F2", &prof, o).unwrap();
        let m = mass_props(&b);
        // oracle: volume 6983.493054573724, area 2547.7371956219913, com z 4.762349236064046
        assert!(close(m.volume, 6983.493054573724, 1e-10), "{}", m.volume);
        assert!(close(m.area, 2547.7371956219913, 1e-10), "{}", m.area);
        assert!((m.com.z - 4.762349236064046).abs() < 1e-9);
        assert!(close(m.principal[0], 253799.75274091944, 1e-9));
    }
    fn fr_id() -> Frame {
        Frame::identity()
    }

    #[test]
    fn revolve_mass_matches_oracle() {
        let rect_p = |x0: f64| {
            let p = [Vec2::new(x0, 0.0), Vec2::new(20.0, 0.0), Vec2::new(20.0, 5.0), Vec2::new(x0, 5.0)];
            let outer = ProfileLoop::new((0..4).map(|i| ProfileSeg::line(&format!("e{}", i + 1), p[i], p[(i + 1) % 4])).collect());
            Profile::new(Frame::identity(), vec![ProfileRegion { outer, holes: vec![] }])
        };
        // rev360 (10..20): 4712.3889803846905 area 2827.433388230816 principal [598866.09.., .., 1178097.245..]
        let m = mass_props(&revolve("F2", &rect_p(10.0), Vec3::ZERO, Vec3::Y, 0.0, TAU).unwrap());
        assert!(close(m.volume, 4712.3889803846905, 1e-13), "{}", m.volume);
        assert!(close(m.area, 2827.433388230816, 1e-13));
        assert!(close(m.principal[0], 598866.0995905546, 1e-12));
        assert!(close(m.principal[2], 1178097.2450961727, 1e-12));
        // rev90: 1178.0972450961717 area 806.8583470577036 com [9.902974236829046,2.5,-9.902974236829046]
        let m = mass_props(&revolve("F2", &rect_p(10.0), Vec3::ZERO, Vec3::Y, 0.0, PI / 2.0).unwrap());
        assert!(close(m.volume, 1178.0972450961717, 1e-13));
        assert!(close(m.area, 806.8583470577036, 1e-13));
        assert!(vclose(m.com, [9.902974236829046, 2.5, -9.902974236829046], 1e-11));
        // revax360 (solid cylinder): 6283.185307179589 area 3141.5926535897943
        let m = mass_props(&revolve("F2", &rect_p(0.0), Vec3::ZERO, Vec3::Y, 0.0, TAU).unwrap());
        assert!(close(m.volume, 6283.185307179589, 1e-13), "{}", m.volume);
        assert!(close(m.area, 3141.5926535897943, 1e-13));
        // revax90: 1570.7963267948962 area 985.3981633974479 principal [33998.42818517255, 60352.12.., 87805.56..]
        let m = mass_props(&revolve("F2", &rect_p(0.0), Vec3::ZERO, Vec3::Y, 0.0, PI / 2.0).unwrap());
        assert!(close(m.volume, 1570.7963267948962, 1e-13));
        assert!(close(m.area, 985.3981633974479, 1e-13));
        assert!(close(m.principal[0], 33998.42818517255, 1e-12));
        assert!(close(m.principal[2], 87805.56851717288, 1e-12));
    }

    #[test]
    fn sphere_torus_cone_mass() {
        let fr = Frame::identity();
        let semi = ProfileLoop::new(vec![
            ProfileSeg::arc("e1", Vec2::ZERO, Vec2::new(0.0, -10.0), Vec2::new(0.0, 10.0)),
            ProfileSeg::line("e2", Vec2::new(0.0, 10.0), Vec2::new(0.0, -10.0)),
        ]);
        let m = mass_props(&revolve("F2", &Profile::new(fr, vec![ProfileRegion { outer: semi, holes: vec![] }]), Vec3::ZERO, Vec3::Y, 0.0, TAU).unwrap());
        // oracle 4188.79020478639, 1256.6370614359178, principal 167551.6081914555
        assert!(close(m.volume, 4188.79020478639, 1e-13), "{}", m.volume);
        assert!(close(m.area, 1256.6370614359178, 1e-13));
        assert!(close(m.principal[0], 167551.60819145554, 1e-12));
        let tor = ProfileLoop::new(vec![ProfileSeg::circle("e1", Vec2::new(20.0, 0.0), 5.0)]);
        let m = mass_props(&revolve("F2", &Profile::new(fr, vec![ProfileRegion { outer: tor, holes: vec![] }]), Vec3::ZERO, Vec3::Y, 0.0, TAU).unwrap());
        // oracle 9869.604401089362, 3947.841760435745, principal [2128133.4489848926, .., 4132896.8429561704]
        assert!(close(m.volume, 9869.604401089362, 1e-13), "{}", m.volume);
        assert!(close(m.area, 3947.841760435745, 1e-13));
        assert!(close(m.principal[2], 4132896.8429561704, 1e-12));
        let tri = ProfileLoop::new(vec![
            ProfileSeg::line("e1", Vec2::new(5.0, 0.0), Vec2::new(15.0, 0.0)),
            ProfileSeg::line("e2", Vec2::new(15.0, 0.0), Vec2::new(5.0, 10.0)),
            ProfileSeg::line("e3", Vec2::new(5.0, 10.0), Vec2::new(5.0, 0.0)),
        ]);
        let m = mass_props(&revolve("F2", &Profile::new(fr, vec![ProfileRegion { outer: tri, holes: vec![] }]), Vec3::ZERO, Vec3::Y, 0.0, 120f64.to_radians()).unwrap());
        // oracle 872.664625997165, 710.3514612362036, com [3.721470044097097,3,-6.445775195221761]
        assert!(close(m.volume, 872.664625997165, 1e-13), "{}", m.volume);
        assert!(close(m.area, 710.3514612362036, 1e-13));
        assert!(vclose(m.com, [3.721470044097097, 3.0, -6.445775195221761], 1e-11));
        assert!(close(m.principal[1], 24886.35364447574, 1e-12));
        // oracle axes (raw Jacobi signs, brep probe `cone`)
        for (a, o) in m.axes.iter().zip([
            [0.8660254037844387, 1.6315768633458208e-16, 0.49999999999999983],
            [-0.3896325508076662, 0.6266944242646857, 0.6748633742815399],
            [0.3133472121323425, 0.7792651016153322, -0.542733291823281],
        ]) {
            assert!(vclose(*a, o, 1e-9), "{a:?} {o:?}");
        }
    }

    #[test]
    fn jacobi_signs_are_the_rotations() {
        // oracle rev90 / rev90flip (front rect (10,0)-(20,5) about +Y, ±90°):
        // the 45° pair comes out of the (0,2) rotation unsigned-normalised
        let fr = Frame::identity();
        let r = || ProfileRegion { outer: rect(["e1", "e2", "e3", "e4"], 10.0, 0.0, 20.0, 5.0), holes: vec![] };
        let m = mass_props(&revolve("F2", &Profile::new(fr, vec![r()]), Vec3::ZERO, Vec3::Y, 0.0, PI / 2.0).unwrap());
        let h = std::f64::consts::FRAC_1_SQRT_2;
        assert!(vclose(m.axes[0], [h, 0.0, h], 1e-9) && vclose(m.axes[1], [h, 0.0, -h], 1e-9), "{:?}", m.axes);
        // a hand-made tensor: off-diagonal sign decides the rotation sign
        let (_, v) = eigen_sym([[2.0, 0.0, -0.5], [0.0, 5.0, 0.0], [-0.5, 0.0, 3.0]]);
        assert!(v[0].x > 0.0 && v[0].z > 0.0 && v[1].x < 0.0 && v[1].z > 0.0, "{v:?}");
    }

    #[test]
    fn d_shape_and_two_lumps() {
        let fr = Frame::identity();
        let d = ProfileLoop::new(vec![
            ProfileSeg::line("e1", Vec2::new(-10.0, 0.0), Vec2::new(10.0, 0.0)),
            ProfileSeg::arc("e2", Vec2::ZERO, Vec2::new(10.0, 0.0), Vec2::new(-10.0, 0.0)),
        ]);
        let m = mass_props(&extrude("F2", &Profile::new(fr, vec![ProfileRegion { outer: d, holes: vec![] }]), ExtrudeOpts::blind(5.0)).unwrap());
        // oracle 785.3981633974481, 571.238898038469, com y 4.244131815783879
        assert!(close(m.volume, 785.3981633974481, 1e-13));
        assert!(close(m.area, 571.238898038469, 1e-13));
        assert!((m.com.y - 4.244131815783879).abs() < 1e-12);
        let prof2 = Profile::new(
            fr,
            vec![
                ProfileRegion { outer: rect(["a0", "a1", "a2", "a3"], 0.0, 0.0, 10.0, 10.0), holes: vec![] },
                ProfileRegion { outer: rect(["b0", "b1", "b2", "b3"], 20.0, 0.0, 30.0, 10.0), holes: vec![] },
            ],
        );
        let b = extrude("F2", &prof2, ExtrudeOpts::blind(5.0)).unwrap();
        let j = mass_props_json(&b, true, Some(&[1.0, 2.0]));
        // oracle byMass mass 1500, com x 18.33333333333334, principal [15625, 148958.33, 158333.33]
        let bm = j.get("byMass").unwrap();
        assert!(close(bm.get_f64("mass").unwrap(), 1500.0, 1e-13));
        assert!((bm.get("com").unwrap().as_vec3().unwrap()[0] - 18.33333333333334).abs() < 1e-11);
        assert!(close(bm.get_arr("principal").unwrap()[1].as_f64().unwrap(), 148958.33333333308, 1e-12));
        assert_eq!(j.get_arr("bodies").unwrap().len(), 2);
        assert!(close(j.get_arr("bodies").unwrap()[1].get_f64("volume").unwrap(), 500.0, 1e-13));
    }
}
