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

thread_local! {
    static COARSE: std::cell::Cell<bool> = const { std::cell::Cell::new(false) };
}

/// Run `f` with the cheap tier of the keyed (post-boolean sanity) integrals: for a non-rational spline
/// face, only as many Gauss points across a span and as many table nodes across v as the polynomial
/// integrand's degree needs (exact to rounding, a third to a half fewer evaluations); every other face
/// and the boundary quadrature are untouched.
pub fn with_coarse<R>(f: impl FnOnce() -> R) -> R {
    let prev = COARSE.with(|c| c.replace(true));
    let r = f();
    COARSE.with(|c| c.set(prev));
    r
}

/// One boundary piece in uv: either an exact pcurve over [ta,tb] or a
/// polyline (fallback when a coedge has no pcurve).
enum Piece<'a> {
    Exact(&'a Curve2, f64, f64),
    Poly(Vec<Vec2>),
}

/// Pcurves for the coedges of `f` that carry none, where the 3D edge has an exact planar image: a line,
/// circle, ellipse or B-spline lying in a planar face is a line, circle, ellipse or B-spline of the plane's
/// own coordinates (the orthographic projection is linear, so a spline keeps its knots and weights and its
/// control points are the projected ones; the parameter is the edge's). The 48-chord polyline used otherwise
/// cuts 0.3 percent off a full circle and as much off a long spline edge. One entry per coedge in loop order.
fn derived_pcurves(b: &Body, f: FaceId) -> Vec<Option<Curve2>> {
    let surf = &b.face(f).surface;
    let periods = surf.periods();
    let mut out = vec![];
    for &l in &b.face(f).loops {
        // end of the previous coedge in the loop (None where unknown)
        let mut prev: Option<Vec2> = None;
        for &c in &b.lp(l).coedges {
            let co = b.coedge(c);
            let (ta, tb) = b.coedge_params(c);
            if let Some(pc) = &co.pcurve {
                out.push(None);
                prev = Some(pc.eval(tb));
                continue;
            }
            let ed = b.edge(co.edge);
            let planar = matches!(surf, Surface::Plane { .. });
            let mut pc = ck_geom::pcurve::exact_pcurve(surf, &ed.curve, 1e-7);
            if pc.is_none() {
                if let (Surface::Plane { frame }, ck_geom::curve::Curve::BSpline(bs)) = (surf, &ed.curve) {
                    let scale = 1.0 + bs.ctrl.iter().map(|p| p.len()).fold(0.0, f64::max);
                    if bs.ctrl.iter().all(|p| (*p - frame.o).dot(frame.z).abs() <= 1e-7 * scale) {
                        let ctrl = bs
                            .ctrl
                            .iter()
                            .map(|p| {
                                let q = frame.to_local(*p);
                                Vec2::new(q.x, q.y)
                            })
                            .collect();
                        pc = Some(Curve2::BSpline(ck_geom::nurbs::bspline::BSplineCurve { degree: bs.degree, knots: bs.knots.clone(), ctrl, weights: bs.weights.clone() }));
                    }
                }
            }
            // a periodic surface: the exact line must start where the previous coedge ended (modulo a period)
            if !planar {
                if let Some(Curve2::Line { o, d }) = &mut pc {
                    match prev {
                        Some(pv) => {
                            let st = *o + *d * ta;
                            if let Some(per) = periods.0 {
                                o.x += per * ((pv.x - st.x) / per).round();
                            }
                            if let Some(per) = periods.1 {
                                o.y += per * ((pv.y - st.y) / per).round();
                            }
                        }
                        None if periods.0.is_some() || periods.1.is_some() => pc = None,
                        None => {}
                    }
                }
            }
            prev = pc.as_ref().map(|q| q.eval(tb));
            out.push(pc);
        }
    }
    out
}

/// Segments for the polyline image of a coedge without a pcurve: 48, doubled until the 3D curve's chord
/// sagitta over every segment is under 1e-5 of the edge's extent (a 48-chord circle is 0.3 percent short of
/// its area; a long spline edge 1e-4). `fine` off (the post-boolean sanity tier, 1e-3 of the volume) keeps 48.
fn poly_segments(b: &Body, c: crate::body::CoedgeId, fine: bool) -> usize {
    let (ta, tb) = b.coedge_params(c);
    if !fine {
        return 48;
    }
    let cv = &b.edge(b.coedge(c).edge).curve;
    let ext = (0..=8).map(|k| cv.eval(ta + (tb - ta) * k as f64 / 8.0)).collect::<Vec<_>>();
    let len: f64 = ext.windows(2).map(|w| w[0].dist(w[1])).sum();
    let tol = 1e-5 * len.max(1e-9);
    let mut n = 48;
    while n < 3072 {
        let pts: Vec<Vec3> = (0..=n).map(|k| cv.eval(ta + (tb - ta) * k as f64 / n as f64)).collect();
        let worst = (0..n)
            .map(|k| cv.eval(ta + (tb - ta) * (k as f64 + 0.5) / n as f64).dist((pts[k] + pts[k + 1]) * 0.5))
            .fold(0.0, f64::max);
        if worst <= tol {
            break;
        }
        n *= 2;
    }
    n
}

fn face_pieces<'a>(b: &'a Body, f: FaceId, derived: &'a [Option<Curve2>], fine: bool) -> Vec<Piece<'a>> {
    let mut out = vec![];
    let mut k_co = 0;
    for &l in &b.face(f).loops {
        let mut prev: Option<Vec2> = None;
        for &c in &b.lp(l).coedges {
            let (ta, tb) = b.coedge_params(c);
            let pcurve = b.coedge(c).pcurve.as_ref().or(derived[k_co].as_ref());
            k_co += 1;
            match pcurve {
                Some(pc) => {
                    prev = Some(pc.eval(tb));
                    out.push(Piece::Exact(pc, ta, tb));
                }
                None => {
                    let n = poly_segments(b, c, fine);
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
fn panels_for(pc: &Curve2, ta: f64, tb: f64, ang: (bool, bool), curved: bool, spline: bool, poly: bool) -> usize {
    match pc {
        Curve2::Line { d, .. } => {
            let step = std::f64::consts::PI / 8.0;
            let mut n: f64 = if spline && (poly || d.y == 0.0) { 1.0 } else if curved { 4.0 } else { 2.0 };
            if ang.0 {
                n = n.max((d.x * (tb - ta)).abs() / step);
            }
            if ang.1 {
                n = n.max((d.y * (tb - ta)).abs() / step);
            }
            n.ceil() as usize
        }
        Curve2::Circle { .. } | Curve2::Ellipse { .. } => ((tb - ta).abs() / (std::f64::consts::PI / 8.0)).ceil().max(2.0) as usize,
        Curve2::BSpline(bs) => (bs.ctrl.len() * 2).clamp(4, 4096),
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
        Surface::Screw { .. } => (true, false),
    }
}

/// ∮ G dv over the face boundary = sense · ∫∫_D F(σ, σu×σv) du dv, i.e.
/// the integral of F against the OUTWARD area element. `res` ≥ 1 scales
/// panel counts (region.rs refines near-singular integrands with it).
pub fn face_integral<const K: usize>(b: &Body, f: FaceId, res: usize, integrand: &dyn Fn(Vec3, Vec3) -> [f64; K]) -> [f64; K] {
    face_integral_with(b, f, res, integrand, None, false, None)
}

/// [`face_integral`] for an integrand that is a polynomial in the surface parameter `v` on cylinders and
/// cones (the area element and the moments of a cylinder / cone are): a face with a long dense boundary
/// (cut by helical flutes) then tabulates its inner integrals (see `analytic_tab`).
pub fn face_integral_vpoly<const K: usize>(b: &Body, f: FaceId, res: usize, integrand: &dyn Fn(Vec3, Vec3) -> [f64; K]) -> [f64; K] {
    face_integral_with(b, f, res, integrand, None, true, None)
}

/// n-point Gauss-Legendre rule on [-1, 1] (Newton on the Legendre polynomial).
fn gauss_legendre(n: usize) -> Vec<(f64, f64)> {
    let mut out = Vec::with_capacity(n);
    for i in 0..n {
        let mut x = (std::f64::consts::PI * (i as f64 + 0.75) / (n as f64 + 0.5)).cos();
        let mut dp = 1.0;
        for _ in 0..100 {
            let (mut p0, mut p1) = (1.0, x);
            for k in 2..=n {
                let p2 = ((2 * k - 1) as f64 * x * p1 - (k - 1) as f64 * p0) / k as f64;
                p0 = p1;
                p1 = p2;
            }
            if n == 1 {
                p0 = 1.0;
                p1 = x;
            }
            dp = n as f64 * (x * p1 - p0) / (x * x - 1.0);
            let dx = p1 / dp;
            x -= dx;
            if dx.abs() < 1e-16 {
                break;
            }
        }
        out.push((x, 2.0 / ((1.0 - x * x) * dp * dp)));
    }
    out.reverse();
    out
}

/// [`face_integral`] for an integrand that is a polynomial of degree `ord`
/// in the position times the (bilinear) area element, at `res` = 1. On a
/// non-rational B-spline face every Gauss panel is then exact with just the
/// points the degree needs (one panel per u span, fewer points across v),
/// which matters on long screw sweeps (hundreds of spans).
pub fn face_integral_poly<const K: usize>(b: &Body, f: FaceId, ord: usize, integrand: &dyn Fn(Vec3, Vec3) -> [f64; K]) -> [f64; K] {
    face_integral_with(b, f, 1, integrand, Some(ord), true, None)
}

thread_local! {
    /// Cumulative-integral tables of long spline faces (see `build_vtab`), by caller-chosen key.
    static VTAB_CACHE: std::cell::RefCell<std::collections::HashMap<u64, std::rc::Rc<dyn std::any::Any>>> = std::cell::RefCell::new(std::collections::HashMap::new());
}

/// [`face_integral_poly`] for an integrand that is the same function on every call with the same
/// `key` (the key must identify the integrand and the surface): the tables a long spline face
/// builds depend on the surface and the integrand, not on the face's trimming, so a surface that
/// comes back with other trimming (a cut through a shell wall) reuses them.
pub fn face_integral_poly_keyed<const K: usize>(b: &Body, f: FaceId, ord: usize, key: u64, integrand: &dyn Fn(Vec3, Vec3) -> [f64; K]) -> [f64; K] {
    face_integral_with(b, f, 1, integrand, Some(ord), true, Some(key ^ (K as u64).wrapping_mul(0x9e3779b97f4a7c15) ^ (ord as u64) << 56))
}

fn face_integral_with<const K: usize>(b: &Body, f: FaceId, res: usize, integrand: &dyn Fn(Vec3, Vec3) -> [f64; K], poly_ord: Option<usize>, vpoly: bool, tab_key: Option<u64>) -> [f64; K] {
    let surf = &b.face(f).surface;
    let coarse = tab_key.is_some() && COARSE.with(|c| c.get());
    let tab_key = tab_key.map(|k| if coarse { k ^ 0xc0a2_5e00_0000_0001 } else { k });
    let derived = derived_pcurves(b, f);
    let pieces = split_at_knots(surf, face_pieces(b, f, &derived, tab_key.is_none()));
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
    // (many boundary samples: a face cut by helical flutes)
    let dense_boundary = vpoly && {
        let dense: usize = pieces
            .iter()
            .map(|p| match p {
                Piece::Exact(pc @ Curve2::BSpline(_), ta, tb) => panels_for(pc, *ta, *tb, ang, true, false, false) * 8,
                Piece::Poly(pts) => pts.len() * 8,
                _ => 0,
            })
            .sum();
        dense > 1500
    };
    let mut hu = width / (4.0 * res as f64);
    if ang.0 {
        hu = hu.min(std::f64::consts::PI / (8.0 * res as f64));
    }
    // (the keyed caller is the post-boolean volume check at 1e-3 of the volume: the original kernel
    // integrates with one panel per knot span there; `width / 4` is enough for it)
    if tab_key.is_none() && matches!(surf, Surface::BSpline(_) | Surface::Revolution { .. } | Surface::Extrusion { .. } | Surface::Screw { .. }) {
        hu = hu.min(width / (16.0 * res as f64));
    }
    // a plane's polynomial integrands are exact in one GL8 panel: with a dense boundary do not subdivide
    if dense_boundary && matches!(surf, Surface::Plane { .. }) {
        hu = width * 2.0;
    }
    // polynomial integrand on a non-rational spline: exact panel counts
    // (u: degree (ord+2) pu, cumulated; v: degree (ord+2) pv - 1, GL points)
    let poly_deg = match (surf, poly_ord) {
        (Surface::BSpline(bs), Some(ord)) if bs.weights.is_none() => Some(((ord + 2) * bs.deg_u, (ord + 2) * bs.deg_v)),
        _ => None,
    };
    let fast_panels = match poly_deg {
        Some((du, _)) if du <= 15 => 1,
        _ => 2,
    };
    // coarse tier: across a polynomial span the Gauss rule with just the points the degree needs
    // (exact to rounding, like the 8 points, which are more than a cubic needs)
    let coarse_rule: Option<Vec<(f64, f64)>> = if coarse { poly_deg.filter(|&(du, _)| du <= 15).map(|(du, _)| gauss_legendre(((du + 2) / 2).clamp(2, 8))) } else { None };
    let v_rule: Option<Vec<(f64, f64)>> = poly_deg.map(|(_, dv)| gauss_legendre(((dv + 1) / 2).clamp(2, 8)));
    // a cylinder / cone cut by helical flutes: thousands of boundary samples, each needing an inner integral
    // over up to a turn. The integrands are polynomial in v, so tabulate the cumulative integral at the
    // pi/8 cuts for a few v nodes (as for the screw patches) and interpolate
    let analytic_tab = dense_boundary && matches!(surf, Surface::Cylinder { .. } | Surface::Cone { .. }) && umax - umin > 0.8 && vmax > vmin;
    let curved = !matches!(surf, Surface::Plane { .. });
    // non-rational B-spline boundary lines are cut at the knots: pieces are short
    // (the one-panel rule is exact for polynomial integrands only; an area element is not, e.g. a ruled twisted
    // face. A piece along u (v constant) contributes nothing, so it keeps the single panel)
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
        // a screw patch: cut every pi/8 (a hair less, so a cut segment never needs a second panel), where one
        // GL8 panel is exact to rounding for the trigonometric integrands
        Surface::Screw { urange: (a, z), .. } => {
            let step = std::f64::consts::PI / 8.0 * 0.9999;
            (1..).map(|i| a + step * i as f64).take_while(|&x| x < z - 1e-12).collect()
        }
        _ if analytic_tab => {
            let step = std::f64::consts::PI / 8.0 * 0.9999;
            (1..).map(|i| umin + step * i as f64).take_while(|&x| x < umax - 1e-12).collect()
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
        let sl = surf.screw_vline(v);
        for j in 0..=m_in {
            let (a, c) = (cut(j), cut(j + 1));
            let span = c - a;
            if span == 0.0 {
                continue;
            }
            let fast = vl.as_ref().and_then(|l| l.segment(0.5 * (a + c))).filter(|s| s.covers(a, c));
            let n = if fast.is_some() { fast_panels * res } else { ((span.abs() / hu).ceil() as usize).clamp(1, 512 * res) };
            let h = span / n as f64;
            let rule: &[(f64, f64)] = match &coarse_rule {
                Some(r) if fast.is_some() => r,
                _ => &GL,
            };
            for i in 0..n {
                let m = a + h * (i as f64 + 0.5);
                for &(x, w) in rule.iter() {
                    let s = m + 0.5 * h * x;
                    let (pt, nrm) = match &fast {
                        Some(sg) => {
                            let (p, du, dv) = sg.eval1(s);
                            (p, du.cross(dv))
                        }
                        None if sl.is_some() => {
                            let (p, du, dv) = sl.as_ref().unwrap().eval1(s);
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
    let long_spline = analytic_tab || knots_u.len() >= if matches!(surf, Surface::Screw { .. }) { 6 } else { 32 };
    // per v span: Chebyshev-Lobatto nodes (exact for the polynomial integrands, ~1e-12 for the smooth
    // area element |Su x Sv|) and the cumulative integral at every u knot for each node
    type VSpanTab<const K: usize> = (f64, f64, Vec<f64>, Vec<Vec<[f64; K]>>);
    let vspans: Vec<(f64, f64)> = match surf {
        Surface::BSpline(bs) if long_spline => {
            let ((_, _), (dv0, dv1)) = bs.domain();
            let mut ks: Vec<f64> = vec![dv0];
            ks.extend(bs.knots_v.iter().copied().filter(|&x| x > dv0 && x < dv1));
            ks.push(dv1);
            ks.dedup();
            ks.windows(2).map(|w| (w[0], w[1])).collect()
        }
        Surface::Screw { vrange, .. } if long_spline && vrange.1 > vrange.0 => vec![*vrange],
        _ if analytic_tab => vec![(vmin, vmax)],
        _ => vec![],
    };
    let build_vtab_raw = || -> Option<(Vec<f64>, Vec<VSpanTab<K>>)> {
        let ((du0, du1), nvn) = match surf {
            Surface::BSpline(bs) => (bs.domain().0, match poly_deg {
                    // (the integrand is a polynomial of degree dv in v: dv + 1 nodes interpolate it exactly)
                    Some((_, dv)) if coarse => dv + 1,
                    _ => 3 * bs.deg_v + 6,
                }),
            // (volume-type integrands are trigonometric polynomials in v: 24 nodes are spectral; the area
            // element of a strongly twisted face has singularities near the real axis: 56)
            Surface::Screw { urange, .. } => (*urange, if poly_ord.is_some() { 24 } else { 56 }),
            _ if analytic_tab => ((umin, umax), 10),
            _ => return None,
        };
        if !long_spline || vspans.is_empty() {
            return None;
        }
        let mut ks = vec![du0];
        ks.extend(knots_u.iter().copied());
        ks.push(du1);
        let tabs = vspans
            .iter()
            .map(|&(a, b)| {
                let vn: Vec<f64> = (0..nvn).map(|j| a + (b - a) * 0.5 * (1.0 - (std::f64::consts::PI * j as f64 / (nvn - 1) as f64).cos())).collect();
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
                (a, b, vn, cum)
            })
            .collect();
        Some((ks, tabs))
    };
    type VTab<const K: usize> = Option<(Vec<f64>, Vec<VSpanTab<K>>)>;
    let cache_get = || -> Option<std::rc::Rc<VTab<K>>> {
        let key = tab_key?;
        VTAB_CACHE.with(|c| c.borrow().get(&key).cloned()).and_then(|r| r.downcast::<VTab<K>>().ok())
    };
    let build_vtab = || -> std::rc::Rc<VTab<K>> {
        if let Some(hit) = cache_get() {
            return hit;
        }
        let t = std::rc::Rc::new(build_vtab_raw());
        if let (Some(key), true) = (tab_key, t.is_some()) {
            VTAB_CACHE.with(|c| {
                let mut c = c.borrow_mut();
                if c.len() >= 16 {
                    c.clear();
                }
                c.insert(key, t.clone() as std::rc::Rc<dyn std::any::Any>);
            });
        }
        t
    };
    // a ruled screw face (one v span): the table is always cheaper; a face with several v spans builds it
    // only once as many boundary samples as table nodes sit at distinct v (a cut curve across the sweep)
    let vtab: std::cell::OnceCell<std::rc::Rc<VTab<K>>> = std::cell::OnceCell::new();
    if vspans.len() == 1 {
        let _ = vtab.set(build_vtab());
    }
    let new_lists = std::cell::Cell::new(0usize);
    let table_ready = || -> bool {
        if vtab.get().is_none() && vspans.len() > 1 {
            if let Some(hit) = cache_get() {
                let _ = vtab.set(hit);
            }
        }
        if vspans.len() == 1 || vtab.get().is_some() {
            return vtab.get().map_or(false, |t| t.is_some());
        }
        // (building costs one inner integral per table node: worth it once that many boundary samples
        // sit at distinct v)
        let nodes = vspans.len() * match surf {
            Surface::BSpline(bs) => 3 * bs.deg_v + 6,
            Surface::Screw { .. } => 56,
            _ => 0,
        };
        new_lists.get() >= nodes && vtab.get_or_init(build_vtab).is_some()
    };
    // h(u, v) - h(u*, v) from the table: the v interpolation (barycentric form on the Chebyshev-Lobatto
    // nodes) is shared by both ends; the partial spans are integrated directly at the true v (one
    // seg_integral instead of one per interpolation node)
    let h_diff = |u: f64, v: f64| -> [f64; K] {
        let (ks, tabs) = (**vtab.get().unwrap()).as_ref().unwrap();
        let kof = |x: f64| (ks.partition_point(|&y| y <= x).max(1) - 1).min(ks.len() - 2);
        let (k, k0) = (kof(u), kof(ustar));
        let si = (tabs.partition_point(|t| t.0 <= v).max(1) - 1).min(tabs.len() - 1);
        let (_, _, vn, cum) = &tabs[si];
        let n = vn.len();
        let mut w = [0.0f64; 128];
        let (mut sum, mut exact) = (0.0, None);
        for j in 0..n {
            let d = v - vn[j];
            if d == 0.0 {
                exact = Some(j);
                break;
            }
            let lam = if j % 2 == 0 { 1.0 } else { -1.0 } * if j == 0 || j + 1 == n { 0.5 } else { 1.0 };
            w[j] = lam / d;
            sum += w[j];
        }
        if let Some(j) = exact {
            w[..n].fill(0.0);
            w[j] = 1.0;
            sum = 1.0;
        }
        let mut g = [0.0; K];
        for j in 0..n {
            let wj = w[j] / sum;
            for i in 0..K {
                g[i] += wj * (cum[j][k][i] - cum[j][k0][i]);
            }
        }
        let (part, part0) = (seg_integral(ks[k], u, v), seg_integral(ks[k0], ustar, v));
        for i in 0..K {
            g[i] += part[i] - part0[i];
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
        if long_spline && table_ready() {
            return h_diff(u, v);
        }
        if long_spline {
            let v = (v * 1e9).round() / 1e9;
            let mut map = ckpt.borrow_mut();
            let list = map.entry(v.to_bits()).or_insert_with(|| {
                new_lists.set(new_lists.get() + 1);
                vec![(ustar, [0.0; K])]
            });
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
    // a generic (non-polynomial) integrand on a screw patch -- the area element of a strongly twisted face
    // has complex singularities close to the real axis -- is integrated along the boundary adaptively
    let adaptive = matches!(surf, Surface::Screw { .. }) && poly_ord.is_none();
    let mut acc_ad = [0.0; K];
    let panel = |pc: &Curve2, a: f64, b: f64| -> [f64; K] {
        let mut s = [0.0; K];
        let (m, hh) = (0.5 * (a + b), 0.5 * (b - a));
        for &(x, w) in GL.iter() {
            let d = pc.derivs(m + hh * x, 1);
            if d[1].y == 0.0 {
                continue;
            }
            let g = inner(d[0].x, d[0].y);
            for k in 0..K {
                s[k] += hh * w * d[1].y * g[k];
            }
        }
        s
    };
    for p in &pieces {
        match p {
            // (the dense panels of a fitted boundary curve already follow it; the straight cap edges have 4)
            Piece::Exact(pc, ta, tb) if adaptive && matches!(pc, Curve2::Line { d, .. } if d.y != 0.0) => {
                let n = panels_for(pc, *ta, *tb, ang, curved, spline, false) * res;
                let h = (tb - ta) / n as f64;
                for i in 0..n {
                    let (a, b) = (ta + h * i as f64, ta + h * (i + 1) as f64);
                    let g = adapt_panel(&|x, y| panel(pc, x, y), a, b, panel(pc, a, b), 0);
                    for k in 0..K {
                        acc_ad[k] += g[k];
                    }
                }
            }
            Piece::Exact(pc, ta, tb) => {
                let mut n = panels_for(pc, *ta, *tb, ang, curved, spline, poly_ord.is_some()) * res;
                // (the keyed caller is the post-boolean volume check, which needs ~1e-4 of the volume: a quarter
                // of the usual two panels per control point of a fitted trim curve is good to ~5e-5 there)
                if tab_key.is_some() && matches!(pc, Curve2::BSpline(_)) {
                    n = (n / 16).max(2).min(n);
                }
                let h = (tb - ta) / n as f64;
                // a piece along v at constant u (a cap of a long sweep): the
                // cumulated integrand is a polynomial in v of the degree above
                let rule: &[(f64, f64)] = match (&v_rule, pc) {
                    (Some(r), Curve2::Line { d, .. }) if d.x == 0.0 && n == 1 => r,
                    _ => &GL,
                };
                for i in 0..n {
                    let m = ta + h * (i as f64 + 0.5);
                    for &(x, w) in rule.iter() {
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
    for k in 0..K {
        acc[k] += acc_ad[k];
    }
    acc
}

/// Panel `[a, b]` of a boundary integral refined (by halves, to depth 6) until the two halves agree with
/// the whole to 1e-10 in every component.
fn adapt_panel<const K: usize>(panel: &dyn Fn(f64, f64) -> [f64; K], a: f64, b: f64, whole: [f64; K], depth: u32) -> [f64; K] {
    let m = 0.5 * (a + b);
    let (l, r) = (panel(a, m), panel(m, b));
    let mut ok = true;
    let mut sum = [0.0; K];
    for k in 0..K {
        sum[k] = l[k] + r[k];
        if (sum[k] - whole[k]).abs() > 1e-10 * (l[k].abs() + r[k].abs()) + 1e-12 {
            ok = false;
        }
    }
    if ok || depth >= 6 {
        return sum;
    }
    let (gl, gr) = (adapt_panel(panel, a, m, l, depth + 1), adapt_panel(panel, m, b, r, depth + 1));
    for k in 0..K {
        sum[k] = gl[k] + gr[k];
    }
    sum
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
        let v = face_integral_vpoly::<11>(b, f, 1, &|p, n| {
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

/// Reference point (bbox centre) the moments are taken about.
pub fn ref_point_of(b: &Body) -> Vec3 {
    ref_point(b)
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
        .map(|&f| face_integral_poly::<1>(b, f, 1, &|p, n| [(p - r).dot(n) / 3.0])[0])
        .sum()
}

/// One face's share of the signed volume, measured from the origin (so it depends on the face
/// alone and can be cached per face; the sum over a closed shell does not depend on the origin).
pub fn face_signed_volume(b: &Body, f: FaceId) -> f64 {
    face_integral_poly_keyed::<1>(b, f, 1, b.face(f).surface.fingerprint() ^ 0x5eed_0001, &|p, n| [p.dot(n) / 3.0])[0]
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

    #[test]
    fn gauss_legendre_rules() {
        for (g, r) in GL.iter().zip(gauss_legendre(8)) {
            assert!((g.0 - r.0).abs() < 1e-14 && (g.1 - r.1).abs() < 1e-14, "{g:?} {r:?}");
        }
        // 5 points integrate x^8 exactly: 2/9
        let s: f64 = gauss_legendre(5).iter().map(|&(x, w)| w * x.powi(8)).sum();
        assert!((s - 2.0 / 9.0).abs() < 1e-15, "{s}");
        let s: f64 = gauss_legendre(2).iter().map(|&(x, w)| w * x * x).sum();
        assert!((s - 2.0 / 3.0).abs() < 1e-15, "{s}");
    }
}
