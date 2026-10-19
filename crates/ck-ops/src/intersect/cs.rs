//! Curve-surface intersection (ck-ops/src/intersect/cs.rs).
//!
//! Closed forms: line vs plane / cylinder / cone / sphere (quadratic) /
//! torus (quartic, ck-core::quad::quartic); circle vs plane
//! (a cos t + b sin t + c = 0). Everything else: sampled seeds of the
//! curve-to-surface distance refined by 3-variable Newton on
//! C(t) − S(u,v) = 0. Curves lying in the surface are reported as
//! `on` parameter intervals.

use super::{fit_params_all, param_slack, solve3};
use ck_core::quad;
use ck_geom::curve::Curve;
use ck_geom::surface::Surface;
use ck_geom::Vec3;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CsHit {
    pub t: f64,
    pub uv: (f64, f64),
    pub p: Vec3,
}

#[derive(Debug, Clone, Default, PartialEq)]
pub struct CsResult {
    pub hits: Vec<CsHit>,
    /// Curve parameter intervals lying on the surface.
    pub on: Vec<(f64, f64)>,
}

/// Intersect curve `c` over `r` with the (untrimmed) surface `s`.
pub fn intersect(c: &Curve, r: (f64, f64), s: &Surface, tol: f64) -> CsResult {
    let mut out = CsResult::default();
    // curve entirely on the surface?
    if lies_on(c, r, s, tol) {
        out.on.push(r);
        return out;
    }
    let ts: Option<Vec<f64>> = match (c, s) {
        (Curve::Line { o, d }, _) => line_params(*o, *d, s, tol),
        (Curve::Circle { frame, r: rad }, Surface::Plane { frame: pf }) => {
            Some(super::cc::circle_plane_params(frame, *rad, pf.o, pf.z))
        }
        (Curve::Ellipse { frame, a, b }, Surface::Plane { frame: pf }) => {
            Some(super::cc::ellipse_plane_params(frame, *a, *b, pf.o, pf.z))
        }
        _ => None,
    };
    match ts {
        Some(ts) => {
            for t in ts {
                for tt in fit_params_all(c, t, r, param_slack(c, t, tol)) {
                    let p = c.eval(tt);
                    let (u, v, q) = s.closest_point(p);
                    if q.dist(p) <= tol.max(1e-9) * 10.0 {
                        out.hits.push(CsHit { t: tt, uv: (u, v), p });
                    }
                }
            }
        }
        None => general(c, r, s, tol, &mut out),
    }
    out.hits.sort_by(|a, b| a.t.partial_cmp(&b.t).unwrap_or(std::cmp::Ordering::Equal));
    out.hits.dedup_by(|a, b| (a.t - b.t).abs() < 1e-10);
    out
}

/// Sampled test: is the whole range on the surface?
fn lies_on(c: &Curve, r: (f64, f64), s: &Surface, tol: f64) -> bool {
    let (a, b) = if r.0.is_finite() && r.1.is_finite() { r } else { (-1.0, 1.0) };
    (0..=8).all(|i| {
        let p = c.eval(a + (b - a) * (i as f64 + 0.37) / 9.0);
        s.distance(p) <= tol
    }) && match c {
        // lines: two points suffice for plane/cyl/cone generators only if
        // also direction-consistent; the samples above cover it
        _ => true,
    }
}

/// Line `o + t d` against analytic surfaces (None = not a closed-form pair).
pub fn line_params(o: Vec3, d: Vec3, s: &Surface, tol: f64) -> Option<Vec<f64>> {
    let roots = |r: quad::Roots| r.iter().collect::<Vec<f64>>();
    match s {
        Surface::Plane { frame } => {
            let den = d.dot(frame.z);
            if den.abs() < 1e-300 {
                return Some(vec![]);
            }
            Some(vec![(frame.o - o).dot(frame.z) / den])
        }
        Surface::Cylinder { frame, r } => {
            let (lo, ld) = (frame.to_local(o), frame.vec_to_local(d));
            let a = ld.x * ld.x + ld.y * ld.y;
            let b = 2.0 * (lo.x * ld.x + lo.y * ld.y);
            let c = lo.x * lo.x + lo.y * lo.y - r * r;
            if a < 1e-24 {
                return Some(vec![]);
            }
            Some(tangent_quadratic(a, b, c, tol, r.max(1.0)))
        }
        Surface::Cone { frame, r, semi_angle } => {
            let (lo, ld) = (frame.to_local(o), frame.vec_to_local(d));
            let k = semi_angle.tan();
            let rr = r + k * lo.z;
            let a = ld.x * ld.x + ld.y * ld.y - k * k * ld.z * ld.z;
            let b = 2.0 * (lo.x * ld.x + lo.y * ld.y - k * ld.z * rr);
            let c = lo.x * lo.x + lo.y * lo.y - rr * rr;
            Some(tangent_quadratic(a, b, c, tol, r.max(1.0)))
        }
        Surface::Sphere { frame, r } => {
            let w = o - frame.o;
            Some(tangent_quadratic(d.dot(d), 2.0 * w.dot(d), w.dot(w) - r * r, tol, r.max(1.0)))
        }
        Surface::Torus { frame, major, minor } => {
            let (lo, ld) = (frame.to_local(o), frame.vec_to_local(d));
            let a = ld.dot(ld);
            let b = 2.0 * lo.dot(ld);
            let c = lo.dot(lo) + major * major - minor * minor;
            let r4 = 4.0 * major * major;
            let q4 = a * a;
            let q3 = 2.0 * a * b;
            let q2 = b * b + 2.0 * a * c - r4 * (ld.x * ld.x + ld.y * ld.y);
            let q1 = 2.0 * b * c - 2.0 * r4 * (lo.x * ld.x + lo.y * ld.y);
            let q0 = c * c - r4 * (lo.x * lo.x + lo.y * lo.y);
            Some(roots(quad::quartic(q4, q3, q2, q1, q0)))
        }
        Surface::BSpline(bs) => line_bspline(o, d, bs, tol),
        Surface::Screw { .. } => line_screw(o, d, s, tol),
        Surface::Revolution { profile, axis_o, axis } => line_revolution(o, d, s, profile, *axis_o, *axis, tol),
        Surface::Extrusion { profile, dir } => line_extrusion(o, d, profile, *dir),
        _ => None,
    }
}

/// Line `o + t d` against a linear extrusion `C(u) + v dir`. The line meets the
/// surface where the profile crosses the plane through the line spanned by `d`
/// and `dir`: roots of the scalar `g(u) = n . (C(u) - o)`, `n = dir x d`, found by
/// sign changes of a dense scan and bisection. `None` (the caller falls back) for a
/// line parallel to the generators, an unbounded or unsupported profile, a scan that
/// touches zero without crossing (tangent) and roots whose residual is not tiny.
fn line_extrusion(o: Vec3, d: Vec3, profile: &Curve, dir: Vec3) -> Option<Vec<f64>> {
    let n = dir.cross(d);
    let nl = n.len();
    if !(nl > 1e-9 * dir.len() * d.len()) || !nl.is_finite() {
        return None;
    }
    let n = n * (1.0 / nl);
    let (a, b) = profile.range();
    if !(a.is_finite() && b.is_finite() && b > a) {
        return None;
    }
    let steps = match profile {
        Curve::BSpline(c) => 32 * (c.knots.len().saturating_sub(2 * c.degree + 1)).max(1),
        Curve::Circle { .. } | Curve::Ellipse { .. } => 256,
        _ => return None,
    }
    .clamp(64, 20000);
    let g = |t: f64| n.dot(profile.eval(t) - o);
    let ts: Vec<f64> = (0..=steps).map(|i| a + (b - a) * i as f64 / steps as f64).collect();
    let gs: Vec<f64> = ts.iter().map(|&t| g(t)).collect();
    let scale = gs.iter().fold(0.0f64, |m, v| m.max(v.abs())).max(1e-12);
    let near0 = 1e-9 * scale.max(1.0);
    let mut roots: Vec<f64> = vec![];
    for i in 0..steps {
        let (g0, g1) = (gs[i], gs[i + 1]);
        if g0 == 0.0 {
            roots.push(ts[i]);
        } else if g0 * g1 < 0.0 {
            let (mut lo, mut hi, mut glo) = (ts[i], ts[i + 1], g0);
            for _ in 0..80 {
                let mid = 0.5 * (lo + hi);
                let gm = g(mid);
                if gm == 0.0 {
                    lo = mid;
                    hi = mid;
                    break;
                }
                if gm * glo < 0.0 {
                    hi = mid;
                } else {
                    lo = mid;
                    glo = gm;
                }
                if hi - lo < 1e-15 * (1.0 + hi.abs()) {
                    break;
                }
            }
            roots.push(0.5 * (lo + hi));
        } else if i > 0 {
            // a local extremum of |g| that comes within noise of zero without a sign change
            let (gp, gc, gn) = (gs[i - 1].abs(), g0.abs(), g1.abs());
            if gc <= gp && gc <= gn && gc < 1e-3 * scale && gc > 0.0 && g0 * g1 > 0.0 && gs[i - 1] * g0 > 0.0 {
                // flat minimum: tangent line, ambiguous
                if gc < 1e-6 * scale + near0 {
                    return None;
                }
            }
        }
    }
    if gs[steps] == 0.0 {
        roots.push(ts[steps]);
    }
    let dxl = d.cross(dir);
    let dxl2 = dxl.len2();
    let dir2 = dir.len2();
    let mut out: Vec<f64> = vec![];
    for t in roots {
        let p = profile.eval(t);
        let lam = (p - o).cross(dir).dot(dxl) / dxl2;
        let q = o + d * lam;
        let sv = (q - p).dot(dir) / dir2;
        let res = (q - (p + dir * sv)).len();
        if !(res <= 1e-7 * (1.0 + p.len().max(o.len()))) {
            return None;
        }
        if !out.iter().any(|&x| (x - lam).abs() < 1e-9 * (1.0 + lam.abs())) {
            out.push(lam);
        }
    }
    Some(out)
}

/// Line `o + t d` against a surface of revolution. In the meridian
/// coordinates (z along the axis, rho radial) the line is the conic
/// `rho² = A t² + B t + C`, `z = z0 + zd t`; the profile, sampled as a
/// polyline in (z, rho), is intersected segment by segment (a quadratic per
/// segment) and every candidate is refined by Newton on `S(u, v) - (o + t d)`.
fn line_revolution(o: Vec3, d: Vec3, s: &Surface, profile: &Curve, axis_o: Vec3, axis: Vec3, tol: f64) -> Option<Vec<f64>> {
    let a = axis.try_normalize()?;
    let w = o - axis_o;
    let (wz, dz) = (w.dot(a), d.dot(a));
    let (wp, dp) = (w - a * wz, d - a * dz);
    let (qa, qb, qc) = (dp.dot(dp), 2.0 * wp.dot(dp), wp.dot(wp));
    let ((_, _), (v0, v1)) = s.domain();
    if !(v0.is_finite() && v1.is_finite()) {
        return None;
    }
    const N: usize = 192;
    let pts: Vec<(f64, f64, f64)> = (0..=N)
        .map(|k| {
            let v = v0 + (v1 - v0) * k as f64 / N as f64;
            let q = profile.eval(v) - axis_o;
            let z = q.dot(a);
            (v, z, (q - a * z).len())
        })
        .collect();
    let mut cands: Vec<(f64, f64)> = vec![]; // (t, v)
    for w2 in pts.windows(2) {
        let ((va, za, ra), (vb, zb, rb)) = (w2[0], w2[1]);
        let (dzs, drs) = (zb - za, rb - ra);
        // rho(s)^2 - (qa t^2 + qb t + qc) = 0 with t(s) = (za + s dzs - wz) / dz
        let (c2, c1, c0);
        if dz.abs() > 1e-12 {
            let (t0, t1) = ((za - wz) / dz, dzs / dz);
            c2 = drs * drs - qa * t1 * t1;
            c1 = 2.0 * ra * drs - (2.0 * qa * t0 * t1 + qb * t1);
            c0 = ra * ra - (qa * t0 * t0 + qb * t0 + qc);
        } else {
            // line normal to the axis: it meets the profile where z = wz
            if dzs.abs() < 1e-300 {
                continue;
            }
            let sx = (wz - za) / dzs;
            if !(-1e-9..=1.0 + 1e-9).contains(&sx) {
                continue;
            }
            let rho = ra + sx * drs;
            // rho^2 = qa t^2 + qb t + qc
            for t in tangent_quadratic(qa, qb, qc - rho * rho, tol, rho.max(1.0)) {
                cands.push((t, va + (vb - va) * sx));
            }
            continue;
        }
        let sols: Vec<f64> = if c2.abs() < 1e-14 * (1.0 + c1.abs()) {
            if c1.abs() > 1e-300 { vec![-c0 / c1] } else { vec![] }
        } else {
            let disc = c1 * c1 - 4.0 * c2 * c0;
            if disc < 0.0 {
                // near-tangent: keep the vertex of the parabola if it nearly touches
                let sv = -c1 / (2.0 * c2);
                let fv = c0 - c1 * c1 / (4.0 * c2);
                if fv.abs() < 1e-9 { vec![sv] } else { vec![] }
            } else {
                let sq = disc.sqrt();
                vec![(-c1 - sq) / (2.0 * c2), (-c1 + sq) / (2.0 * c2)]
            }
        };
        for sx in sols {
            if (-1e-6..=1.0 + 1e-6).contains(&sx) {
                let t = (za + sx * dzs - wz) / dz;
                cands.push((t, va + (vb - va) * sx.clamp(0.0, 1.0)));
            }
        }
    }
    let mut ts: Vec<f64> = vec![];
    for (t0, v) in cands {
        // refine (u, v, t): S(u, v) - o - t d = 0 by Newton (Cramer on the 3x3)
        let (u, _) = s.closest_param(o + d * t0);
        let (mut u, mut v, mut t) = (u, v, t0);
        for _ in 0..30 {
            let sd = s.derivs2(u, v);
            let f = sd.p - (o + d * t);
            if f.len() < 1e-12 * (1.0 + sd.p.len()) {
                break;
            }
            let (c1, c2, c3) = (sd.du, sd.dv, -d);
            let det = c1.dot(c2.cross(c3));
            if det.abs() < 1e-300 {
                break;
            }
            let r = -f;
            u += r.dot(c2.cross(c3)) / det;
            v += r.dot(c3.cross(c1)) / det;
            t += r.dot(c1.cross(c2)) / det;
        }
        // keep only converged roots (the polyline can offer spurious ones)
        if (s.eval(u, v) - (o + d * t)).len() <= tol.max(1e-9) * 10.0 && v >= v0 - 1e-9 && v <= v1 + 1e-9 {
            ts.push(t);
        }
    }
    ts.sort_by(|a, b| a.total_cmp(b));
    ts.dedup_by(|a, b| (*a - *b).abs() < 1e-9);
    Some(ts)
}

/// Line `o + t d` against a B-spline surface: the control
/// net box of every knot-span cell prunes (convex hull), then a 3-variable
/// Newton on `o + t d - S(u, v)` per cell hit. Roots over the whole line
/// (the whole surface domain), de-duplicated.
fn line_bspline(o: Vec3, d: Vec3, bs: &ck_geom::nurbs::bsurf::BSplineSurface, tol: f64) -> Option<Vec<f64>> {
    // rational surfaces too: `derivs` handles the weights and the control
    // net's box still bounds the surface (weights are positive)
    let dd = d.dot(d);
    if dd < 1e-300 {
        return Some(vec![]);
    }
    let ((u0, u1), (v0, v1)) = bs.domain();
    let spans = |kn: &[f64], p: usize, n: usize| -> Vec<usize> { (p..n).filter(|&s| kn[s] < kn[s + 1]).collect() };
    let (su, sv) = (spans(&bs.knots_u, bs.deg_u, bs.nu), spans(&bs.knots_v, bs.deg_v, bs.nv));
    let pad = tol.max(1e-9) * 10.0;
    // the cells whose padded box the line crosses, in (u span, v span) order, with the slab interval
    let mut hits: Vec<(usize, usize, f64, f64)> = vec![];
    if su.len() * sv.len() >= LINE_GRID_CELLS {
        let grid = line_grid(bs, &su, &sv);
        if grid.finite {
            for t in &grid.tiles {
                // (a tile's box holds its cells' boxes: the slab test is monotone in the box, so a tile
                // the line misses is missed by each of its cells)
                if slab(o, d, t.lo, t.hi, pad).is_none() {
                    continue;
                }
                for ia in t.a0..t.a1 {
                    for ib in t.b0..t.b1 {
                        let (lo, hi) = grid.boxes[ia * sv.len() + ib];
                        if let Some((t0, t1)) = slab(o, d, lo, hi, pad) {
                            hits.push((ia, ib, t0, t1));
                        }
                    }
                }
            }
            hits.sort_by_key(|h| (h.0, h.1));
        } else {
            for ia in 0..su.len() {
                for ib in 0..sv.len() {
                    let (lo, hi) = grid.boxes[ia * sv.len() + ib];
                    if let Some((t0, t1)) = slab(o, d, lo, hi, pad) {
                        hits.push((ia, ib, t0, t1));
                    }
                }
            }
        }
    } else {
        for (ia, &a) in su.iter().enumerate() {
            for (ib, &b) in sv.iter().enumerate() {
                let (lo, hi) = cell_box(bs, a, b);
                if let Some((t0, t1)) = slab(o, d, lo, hi, pad) {
                    hits.push((ia, ib, t0, t1));
                }
            }
        }
    }
    let mut roots: Vec<f64> = vec![];
    for (ia, ib, t0, t1) in hits {
        let (a, b) = (su[ia], sv[ib]);
        let (ua, ub, va, vb) = (bs.knots_u[a], bs.knots_u[a + 1], bs.knots_v[b], bs.knots_v[b + 1]);
        let tm = if t0.is_finite() && t1.is_finite() { 0.5 * (t0 + t1) } else { 0.0 };
        for (fu, fv) in [(0.5, 0.5), (0.2, 0.2), (0.8, 0.8), (0.2, 0.8), (0.8, 0.2)] {
            let (mut u, mut v, mut t) = (ua + (ub - ua) * fu, va + (vb - va) * fv, tm);
            let mut ok = false;
            for _ in 0..25 {
                let (pt, su_, sv_) = match bs.derivs1_nr(u, v) {
                    Some(r) => r,
                    None => {
                        let ders = bs.derivs(u, v, 1);
                        (ders[0][0], ders[1][0], ders[0][1])
                    }
                };
                let f = o + d * t - pt;
                if f.len() < tol * 0.01 {
                    ok = true;
                    break;
                }
                let rows = [Vec3::new(d.x, -su_.x, -sv_.x), Vec3::new(d.y, -su_.y, -sv_.y), Vec3::new(d.z, -su_.z, -sv_.z)];
                let Some(dl) = solve3(rows, [-f.x, -f.y, -f.z]) else { break };
                t += dl.x;
                u = (u + dl.y).clamp(u0, u1);
                v = (v + dl.z).clamp(v0, v1);
            }
            if ok {
                // the root must lie in this cell (or a neighbour reports it)
                if u >= ua - 1e-9 && u <= ub + 1e-9 && v >= va - 1e-9 && v <= vb + 1e-9 && !roots.iter().any(|r| (r - t).abs() < 1e-7 * (1.0 + t.abs())) {
                    roots.push(t);
                }
                break;
            }
        }
    }
    Some(roots)
}

/// Surfaces with at least this many knot-span cells keep their cell boxes (and tiles of them) between rays.
const LINE_GRID_CELLS: usize = 256;

/// Box of the control points of the knot-span cell `(a, b)` (spans, i.e. the knot indices).
fn cell_box(bs: &ck_geom::nurbs::bsurf::BSplineSurface, a: usize, b: usize) -> (Vec3, Vec3) {
    let (mut lo, mut hi) = (Vec3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY), Vec3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY));
    for i in a - bs.deg_u..=a {
        for j in b - bs.deg_v..=b {
            let c = bs.cp(i, j);
            lo = Vec3::new(lo.x.min(c.x), lo.y.min(c.y), lo.z.min(c.z));
            hi = Vec3::new(hi.x.max(c.x), hi.y.max(c.y), hi.z.max(c.z));
        }
    }
    (lo, hi)
}

/// Slab test of the line `o + t d` against the box padded by `pad`: the parameter interval, `None` for a miss.
fn slab(o: Vec3, d: Vec3, lo: Vec3, hi: Vec3, pad: f64) -> Option<(f64, f64)> {
    let (mut t0, mut t1) = (f64::NEG_INFINITY, f64::INFINITY);
    let mut miss = false;
    for (oo, dk, l, h) in [(o.x, d.x, lo.x, hi.x), (o.y, d.y, lo.y, hi.y), (o.z, d.z, lo.z, hi.z)] {
        let (l, h) = (l - pad, h + pad);
        if dk.abs() < 1e-300 {
            if oo < l || oo > h {
                miss = true;
            }
        } else {
            let (x, y) = ((l - oo) / dk, (h - oo) / dk);
            t0 = t0.max(x.min(y));
            t1 = t1.min(x.max(y));
        }
    }
    if miss || t0 > t1 {
        None
    } else {
        Some((t0, t1))
    }
}

/// Cell boxes of a B-spline surface in (u span, v span) order, and tiles of up to 4 x 4 cells.
struct LineGrid {
    boxes: Vec<(Vec3, Vec3)>,
    tiles: Vec<LineTile>,
    /// every box coordinate is finite (the tile culling relies on ordered comparisons)
    finite: bool,
}

struct LineTile {
    lo: Vec3,
    hi: Vec3,
    a0: usize,
    a1: usize,
    b0: usize,
    b1: usize,
}

fn line_grid(bs: &ck_geom::nurbs::bsurf::BSplineSurface, su: &[usize], sv: &[usize]) -> std::rc::Rc<LineGrid> {
    use std::cell::RefCell;
    use std::rc::Rc;
    thread_local! {
        static CACHE: RefCell<Vec<(u64, Rc<LineGrid>)>> = const { RefCell::new(Vec::new()) };
    }
    let key = bs.fingerprint();
    if let Some(g) = CACHE.with(|c| c.borrow().iter().find(|e| e.0 == key).map(|e| e.1.clone())) {
        return g;
    }
    let boxes: Vec<(Vec3, Vec3)> = su.iter().flat_map(|&a| sv.iter().map(move |&b| (a, b))).map(|(a, b)| cell_box(bs, a, b)).collect();
    let finite = boxes.iter().all(|(l, h)| [l.x, l.y, l.z, h.x, h.y, h.z].iter().all(|x| x.is_finite()));
    let mut tiles = vec![];
    for a0 in (0..su.len()).step_by(4) {
        let a1 = (a0 + 4).min(su.len());
        for b0 in (0..sv.len()).step_by(4) {
            let b1 = (b0 + 4).min(sv.len());
            let (mut lo, mut hi) = (Vec3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY), Vec3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY));
            for ia in a0..a1 {
                for ib in b0..b1 {
                    let (l, h) = boxes[ia * sv.len() + ib];
                    lo = lo.min(l);
                    hi = hi.max(h);
                }
            }
            tiles.push(LineTile { lo, hi, a0, a1, b0, b1 });
        }
    }
    let g = Rc::new(LineGrid { boxes, tiles, finite });
    CACHE.with(|c| {
        let mut c = c.borrow_mut();
        if c.len() >= 24 {
            c.remove(0);
        }
        c.push((key, g.clone()));
    });
    g
}

/// Line `o + t d` against a screw patch: the patch is cut into cells (pi/16 of rotation by the profile's
/// pieces), each bounded by its corner box padded by the bilinear interpolation error
/// `du^2/8 |Suu| + dv^2/8 |Svv|`; the cells the line's slab test passes are solved by Newton on
/// `S(u, v) - (o + t d)` (a root counts in the cell that contains it).
fn line_screw(o: Vec3, d: Vec3, s: &Surface, tol: f64) -> Option<Vec<f64>> {
    let Surface::Screw { profile, axis_o, axis, urange: (u0, u1), vrange: (v0, v1), .. } = s else { return None };
    if d.dot(d) < 1e-300 {
        return Some(vec![]);
    }
    let nu = (((u1 - u0) / (std::f64::consts::PI / 16.0)).ceil() as usize).max(1);
    let nv = if matches!(**profile, Curve::Line { .. }) {
        1
    } else {
        (((v1 - v0) / (std::f64::consts::PI / 8.0)).ceil() as usize).clamp(2, 24)
    };
    // profile samples: points relative to the axis, and the largest |c''| and radius about the axis
    let mut prof: Vec<Vec3> = Vec::with_capacity(nv + 1);
    let (mut rmax, mut cdd) = (0.0f64, 0.0f64);
    for j in 0..=nv {
        let v = v0 + (v1 - v0) * j as f64 / nv as f64;
        let c = profile.derivs(v, 2);
        let w = c[0] - *axis_o;
        rmax = rmax.max((w - *axis * w.dot(*axis)).len());
        cdd = cdd.max(c[2].len());
        prof.push(w);
    }
    let (du, dv) = ((u1 - u0) / nu as f64, (v1 - v0) / nv as f64);
    let pad = tol.max(1e-9) * 10.0 + 1.25 * (du * du / 8.0 * rmax + dv * dv / 8.0 * cdd);
    let rot_row = |u: f64| -> Vec<Vec3> {
        let (su, cu) = u.sin_cos();
        let rise = if let Surface::Screw { rise, .. } = s { *rise } else { 0.0 };
        prof.iter().map(|&w| *axis_o + w * cu + *axis * (axis.dot(w) * (1.0 - cu) + rise * u) + axis.cross(w) * su).collect()
    };
    let mut roots: Vec<f64> = vec![];
    let mut lower = rot_row(*u0);
    for i in 0..nu {
        let ua = u0 + du * i as f64;
        let ub = if i + 1 == nu { *u1 } else { ua + du };
        let upper = rot_row(ub);
        for j in 0..nv {
            let (va, vb) = (v0 + dv * j as f64, if j + 1 == nv { *v1 } else { v0 + dv * (j + 1) as f64 });
            let cs = [lower[j], lower[j + 1], upper[j], upper[j + 1]];
            let mut lo = cs[0];
            let mut hi = cs[0];
            for c in &cs[1..] {
                lo = lo.min(*c);
                hi = hi.max(*c);
            }
            let (mut t0, mut t1) = (f64::NEG_INFINITY, f64::INFINITY);
            let mut miss = false;
            for (oo, dk, l, h) in [(o.x, d.x, lo.x, hi.x), (o.y, d.y, lo.y, hi.y), (o.z, d.z, lo.z, hi.z)] {
                let (l, h) = (l - pad, h + pad);
                if dk.abs() < 1e-300 {
                    if oo < l || oo > h {
                        miss = true;
                    }
                } else {
                    let (x, y) = ((l - oo) / dk, (h - oo) / dk);
                    t0 = t0.max(x.min(y));
                    t1 = t1.min(x.max(y));
                }
            }
            if miss || t0 > t1 {
                continue;
            }
            let tm = if t0.is_finite() && t1.is_finite() { 0.5 * (t0 + t1) } else { 0.0 };
            for (fu, fv) in [(0.5, 0.5), (0.2, 0.2), (0.8, 0.8), (0.2, 0.8), (0.8, 0.2)] {
                let (mut u, mut v, mut t) = (ua + (ub - ua) * fu, va + (vb - va) * fv, tm);
                let mut ok = false;
                for _ in 0..25 {
                    let sd = s.derivs2(u, v);
                    let f = o + d * t - sd.p;
                    if f.len() < tol * 0.01 {
                        ok = true;
                        break;
                    }
                    let rows = [Vec3::new(d.x, -sd.du.x, -sd.dv.x), Vec3::new(d.y, -sd.du.y, -sd.dv.y), Vec3::new(d.z, -sd.du.z, -sd.dv.z)];
                    let Some(dl) = solve3(rows, [-f.x, -f.y, -f.z]) else { break };
                    t += dl.x;
                    u = (u + dl.y).clamp(*u0, *u1);
                    v = (v + dl.z).clamp(*v0, *v1);
                }
                if ok {
                    if u >= ua - 1e-9 && u <= ub + 1e-9 && v >= va - 1e-9 && v <= vb + 1e-9 && !roots.iter().any(|r| (r - t).abs() < 1e-7 * (1.0 + t.abs())) {
                        roots.push(t);
                    }
                    break;
                }
            }
        }
        lower = upper;
    }
    Some(roots)
}

/// Quadratic roots; a slightly negative discriminant within `tol` of
/// tangency gives the double root.
fn tangent_quadratic(a: f64, b: f64, c: f64, tol: f64, scale: f64) -> Vec<f64> {
    let r = quad::quadratic(a, b, c);
    if !r.is_empty() {
        return r.iter().collect();
    }
    if a.abs() > 1e-300 {
        let t = -b / (2.0 * a);
        let val = (a * t * t + b * t + c).abs();
        // residual ~ 2 r δ for a miss distance δ
        if val <= 2.0 * scale * tol {
            return vec![t];
        }
    }
    vec![]
}

/// Sampling + Newton for arbitrary pairs.
fn general(c: &Curve, r: (f64, f64), s: &Surface, tol: f64, out: &mut CsResult) {
    let (a, b) = if r.0.is_finite() && r.1.is_finite() { r } else { return };
    let n = match c {
        Curve::Line { .. } => 32,
        Curve::Circle { .. } | Curve::Ellipse { .. } => 96,
        Curve::BSpline(bs) => (bs.ctrl.len() * 12).clamp(32, 600),
        Curve::Conic { .. } | Curve::Sinusoid { .. } => 96,
    };
    let ts: Vec<f64> = (0..=n).map(|i| a + (b - a) * i as f64 / n as f64).collect();
    // signed distance along the sample (sign from the surface normal)
    let sd: Vec<(f64, (f64, f64))> = ts
        .iter()
        .map(|&t| {
            let p = c.eval(t);
            let (u, v, q) = s.closest_point(p);
            ((p - q).dot(s.normal(u, v)), (u, v))
        })
        .collect();
    let mut seeds = vec![];
    for i in 0..n {
        let (d0, d1) = (sd[i].0, sd[i + 1].0);
        if d0 == 0.0 || d0 * d1 < 0.0 {
            seeds.push((ts[i] + (ts[i + 1] - ts[i]) * d0.abs() / (d0.abs() + d1.abs()).max(1e-300), sd[i].1));
        } else if i > 0 {
            // local minimum of |d| (tangency candidate)
            let dm = sd[i - 1].0.abs();
            if d0.abs() <= dm && d0.abs() <= d1.abs() && d0.abs() < (b - a) / n as f64 {
                seeds.push((ts[i], sd[i].1));
            }
        }
    }
    if sd[n].0 == 0.0 {
        seeds.push((ts[n], sd[n].1));
    }
    for (t0, uv0) in seeds {
        if let Some((t, u, v)) = newton3(c, s, t0, uv0) {
            if t < a - 1e-9 || t > b + 1e-9 {
                continue;
            }
            let p = c.eval(t);
            if s.eval(u, v).dist(p) <= tol {
                out.hits.push(CsHit { t: t.max(a).min(b), uv: (u, v), p });
            }
        }
    }
}

/// Newton on C(t) − S(u,v) = 0.
pub fn newton3(c: &Curve, s: &Surface, mut t: f64, (mut u, mut v): (f64, f64)) -> Option<(f64, f64, f64)> {
    for _ in 0..40 {
        let cd = c.derivs(t, 1);
        let sd = s.derivs2(u, v);
        let f = cd[0] - sd.p;
        if f.len() < 1e-14 {
            break;
        }
        // [C' , -Su, -Sv] δ = -f  → rows of the transpose system
        let cols = [cd[1], -sd.du, -sd.dv];
        let rows = [
            Vec3::new(cols[0].x, cols[1].x, cols[2].x),
            Vec3::new(cols[0].y, cols[1].y, cols[2].y),
            Vec3::new(cols[0].z, cols[1].z, cols[2].z),
        ];
        let dx = solve3(rows, [-f.x, -f.y, -f.z])?;
        t += dx.x;
        u += dx.y;
        v += dx.z;
        if !(t.is_finite() && u.is_finite() && v.is_finite()) {
            return None;
        }
        if dx.len() < 1e-15 {
            break;
        }
    }
    Some((t, u, v))
}

#[cfg(test)]
mod tests {
    use super::*;
    use ck_geom::Frame;
    use std::f64::consts::TAU;

    fn all() -> (f64, f64) {
        (f64::NEG_INFINITY, f64::INFINITY)
    }

    /// The version of `line_bspline` before the cell-box cache, tiles and the allocation-free derivatives.
    #[allow(clippy::all)]
    fn line_bspline_reference(o: Vec3, d: Vec3, bs: &ck_geom::nurbs::bsurf::BSplineSurface, tol: f64) -> Option<Vec<f64>> {
        // rational surfaces too: `derivs` handles the weights and the control
        // net's box still bounds the surface (weights are positive)
        let dd = d.dot(d);
        if dd < 1e-300 {
            return Some(vec![]);
        }
        let ((u0, u1), (v0, v1)) = bs.domain();
        let spans = |kn: &[f64], p: usize, n: usize| -> Vec<usize> { (p..n).filter(|&s| kn[s] < kn[s + 1]).collect() };
        let (su, sv) = (spans(&bs.knots_u, bs.deg_u, bs.nu), spans(&bs.knots_v, bs.deg_v, bs.nv));
        let pad = tol.max(1e-9) * 10.0;
        let mut roots: Vec<f64> = vec![];
        for &a in &su {
            for &b in &sv {
                let (mut lo, mut hi) = (Vec3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY), Vec3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY));
                for i in a - bs.deg_u..=a {
                    for j in b - bs.deg_v..=b {
                        let c = bs.cp(i, j);
                        lo = Vec3::new(lo.x.min(c.x), lo.y.min(c.y), lo.z.min(c.z));
                        hi = Vec3::new(hi.x.max(c.x), hi.y.max(c.y), hi.z.max(c.z));
                    }
                }
                // slab test of the line against the padded box
                let (mut t0, mut t1) = (f64::NEG_INFINITY, f64::INFINITY);
                let mut miss = false;
                for (oo, dk, l, h) in [(o.x, d.x, lo.x, hi.x), (o.y, d.y, lo.y, hi.y), (o.z, d.z, lo.z, hi.z)] {
                    let (l, h) = (l - pad, h + pad);
                    if dk.abs() < 1e-300 {
                        if oo < l || oo > h {
                            miss = true;
                        }
                    } else {
                        let (x, y) = ((l - oo) / dk, (h - oo) / dk);
                        t0 = t0.max(x.min(y));
                        t1 = t1.min(x.max(y));
                    }
                }
                if miss || t0 > t1 {
                    continue;
                }
                let (ua, ub, va, vb) = (bs.knots_u[a], bs.knots_u[a + 1], bs.knots_v[b], bs.knots_v[b + 1]);
                let tm = if t0.is_finite() && t1.is_finite() { 0.5 * (t0 + t1) } else { 0.0 };
                for (fu, fv) in [(0.5, 0.5), (0.2, 0.2), (0.8, 0.8), (0.2, 0.8), (0.8, 0.2)] {
                    let (mut u, mut v, mut t) = (ua + (ub - ua) * fu, va + (vb - va) * fv, tm);
                    let mut ok = false;
                    for _ in 0..25 {
                        let ders = bs.derivs(u, v, 1);
                        let f = o + d * t - ders[0][0];
                        if f.len() < tol * 0.01 {
                            ok = true;
                            break;
                        }
                        let (su_, sv_) = (ders[1][0], ders[0][1]);
                        let rows = [Vec3::new(d.x, -su_.x, -sv_.x), Vec3::new(d.y, -su_.y, -sv_.y), Vec3::new(d.z, -su_.z, -sv_.z)];
                        let Some(dl) = solve3(rows, [-f.x, -f.y, -f.z]) else { break };
                        t += dl.x;
                        u = (u + dl.y).clamp(u0, u1);
                        v = (v + dl.z).clamp(v0, v1);
                    }
                    if ok {
                        // the root must lie in this cell (or a neighbour reports it)
                        if u >= ua - 1e-9 && u <= ub + 1e-9 && v >= va - 1e-9 && v <= vb + 1e-9 && !roots.iter().any(|r| (r - t).abs() < 1e-7 * (1.0 + t.abs())) {
                            roots.push(t);
                        }
                        break;
                    }
                }
            }
        }
        Some(roots)
    }

    /// `line_bspline` (prepared cell boxes, tile culling, `derivs1_nr`) finds the very roots, in the very
    /// order, of the plain scan over every cell.
    #[test]
    fn line_bspline_matches_the_plain_scan() {
        use ck_geom::nurbs::{basis, bsurf::BSplineSurface};
        let (nu, nv) = (23, 19);
        let mut ctrl = Vec::new();
        for i in 0..nu {
            for j in 0..nv {
                let (x, y) = (i as f64 * 2.0, j as f64 * 2.0);
                ctrl.push(Vec3::new(x, y, 3.0 * (x * 0.21).sin() * (y * 0.17).cos() + 0.05 * x));
            }
        }
        for rational in [false, true] {
            let ws = rational.then(|| (0..nu * nv).map(|k| 1.0 + 0.2 * (k % 5) as f64).collect::<Vec<_>>());
            let bs = BSplineSurface::new(3, 3, basis::clamped_uniform(nu, 3), basis::clamped_uniform(nv, 3), nu, nv, ctrl.clone(), ws).unwrap();
            let mut found = 0;
            for k in 0..300 {
                let o = Vec3::new(1.0 + (k % 17) as f64 * 2.3, 2.0 + (k % 13) as f64 * 2.9, 20.0);
                let d = Vec3::new(0.01 * ((k % 7) as f64 - 3.0), 0.02 * ((k % 5) as f64 - 2.0), -1.0);
                let a = line_bspline(o, d, &bs, 1e-6).unwrap();
                let b = line_bspline_reference(o, d, &bs, 1e-6).unwrap();
                assert_eq!(a.iter().map(|x| x.to_bits()).collect::<Vec<_>>(), b.iter().map(|x| x.to_bits()).collect::<Vec<_>>(), "{k}");
                found += a.len();
            }
            assert!(found > 100, "{found}");
        }
    }

    #[test]
    fn line_vs_screw_patch() {
        use ck_geom::curve::Curve;
        let prof = Curve::circle_x(Vec3::new(5., 0., 0.), Vec3::Y, Vec3::X, 1.5);
        let s = Surface::Screw {
            profile: Box::new(prof),
            axis_o: Vec3::ZERO,
            axis: Vec3::Z,
            rise: 0.4,
            urange: (0.0, 3.0 * TAU),
            vrange: (0.0, std::f64::consts::PI),
        };
        // a line through the tube along the axis direction offset to x = 5: crosses the patch at every turn
        let (o, d) = (Vec3::new(5.0, 0.0, -10.0), Vec3::Z);
        let ts = line_params(o, d, &s, 1e-9).unwrap();
        // brute force: points where the line meets the patch are S(u, v) with x = 5, y = 0
        for &t in &ts {
            let p = o + d * t;
            let (u, v, foot) = s.closest_point(p);
            assert!(foot.dist(p) < 1e-7, "t = {t}: u {u} v {v}");
        }
        assert!(ts.len() >= 4 && ts.len() <= 6, "{ts:?}");
        // a point sampled on the patch, hit from outside along its normal
        let (u, v) = (7.0, 1.2);
        let (p, n) = (s.eval(u, v), s.normal(u, v));
        let ts = line_params(p + n * 2.0, -n, &s, 1e-9).unwrap();
        assert!(ts.iter().any(|t| (t - 2.0).abs() < 1e-7), "{ts:?}");
    }

    #[test]
    fn line_vs_revolution() {
        use ck_geom::curve::Curve;
        // sphere as a half-circle profile revolved about z
        let circ = Curve::Circle { frame: ck_geom::frame_from(Vec3::ZERO, Vec3::Y, Vec3::X), r: 5.0 };
        let sph = Surface::Revolution { profile: Box::new(circ), axis_o: Vec3::ZERO, axis: Vec3::Z };
        let o = Vec3::new(-10.0, 1.0, 0.5);
        let d = Vec3::new(1.0, 0.1, 0.05);
        let mine = line_revolution(o, d, &sph, match &sph { Surface::Revolution { profile, .. } => profile, _ => unreachable!() }, Vec3::ZERO, Vec3::Z, 1e-9).unwrap();
        let exact = line_params(o, d, &Surface::Sphere { frame: Frame::identity(), r: 5.0 }, 1e-9).unwrap();
        assert_eq!(mine.len(), 2, "{mine:?}");
        for t in exact {
            assert!(mine.iter().any(|m| (m - t).abs() < 1e-7), "{t} vs {mine:?}");
        }
    }

    #[test]
    fn line_vs_quadrics() {
        let f = Frame::identity();
        let l = Curve::line(Vec3::new(-10., 0.5, 0.3), Vec3::X);
        let cyl = Surface::Cylinder { frame: f, r: 2.0 };
        let r = intersect(&l, all(), &cyl, 1e-9);
        assert_eq!(r.hits.len(), 2);
        for h in &r.hits {
            assert!(cyl.distance(h.p) < 1e-12);
        }
        let sph = Surface::Sphere { frame: f, r: 3.0 };
        let r = intersect(&l, (0., 100.), &sph, 1e-9);
        assert_eq!(r.hits.len(), 2);
        let cone = Surface::Cone { frame: f, r: 1.0, semi_angle: 0.3 };
        let r = intersect(&l, all(), &cone, 1e-9);
        assert_eq!(r.hits.len(), 2);
        for h in &r.hits {
            assert!(cone.distance(h.p) < 1e-12);
        }
        let tor = Surface::Torus { frame: f, major: 5.0, minor: 1.0 };
        let l2 = Curve::line(Vec3::new(-10., 0., 0.), Vec3::X);
        let r = intersect(&l2, all(), &tor, 1e-9);
        let xs: Vec<f64> = r.hits.iter().map(|h| h.p.x).collect();
        assert_eq!(xs.len(), 4);
        for (x, e) in xs.iter().zip([-6., -4., 4., 6.]) {
            assert!((x - e).abs() < 1e-10, "{xs:?}");
        }
        // tangent line to sphere
        let t = Curve::line(Vec3::new(-10., 3., 0.), Vec3::X);
        assert_eq!(intersect(&t, all(), &sph, 1e-9).hits.len(), 1);
        // generator lying on the cylinder
        let g = Curve::line(Vec3::new(2., 0., 0.), Vec3::Z);
        let r = intersect(&g, (0., 5.), &cyl, 1e-9);
        assert!(r.hits.is_empty() && r.on == vec![(0., 5.)]);
    }

    #[test]
    fn circle_vs_plane_and_general() {
        let c = Curve::circle_x(Vec3::ZERO, Vec3::Z, Vec3::X, 2.0);
        let p = Surface::plane(Vec3::new(1., 0., 0.), Vec3::X);
        let r = intersect(&c, (0., TAU), &p, 1e-9);
        assert_eq!(r.hits.len(), 2);
        for h in &r.hits {
            assert!((h.p.x - 1.0).abs() < 1e-12);
        }
        // circle vs sphere (general path): circle radius 2 at z=0 vs sphere
        // r=2 centred (2,0,0) → x = 1 points
        let s = Surface::Sphere { frame: Frame { o: Vec3::new(2., 0., 0.), ..Frame::identity() }, r: 2.0 };
        let r = intersect(&c, (0., TAU), &s, 1e-9);
        assert_eq!(r.hits.len(), 2, "{r:?}");
        for h in &r.hits {
            assert!((h.p.x - 1.0).abs() < 1e-9);
        }
    }
}

#[cfg(test)]
mod extrusion_tests {
    use super::*;
    use ck_geom::nurbs::bspline::BSplineCurve;

    #[test]
    fn line_through_a_spline_extrusion() {
        // arch profile in the xy plane extruded along z
        let c = BSplineCurve::new(
            3,
            vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
            vec![Vec3::new(0.0, 0.0, 0.0), Vec3::new(2.0, 6.0, 0.0), Vec3::new(8.0, 6.0, 0.0), Vec3::new(10.0, 0.0, 0.0)],
        )
        .unwrap();
        let s = Surface::Extrusion { profile: Box::new(Curve::BSpline(c.clone())), dir: Vec3::Z };
        // a slanted line through the arch at two places
        let (o, d) = (Vec3::new(-1.0, 3.0, -2.0), Vec3::new(1.0, 0.0, 0.3).normalize());
        let ts = line_params(o, d, &s, 1e-6).expect("closed form");
        assert_eq!(ts.len(), 2, "{ts:?}");
        for t in &ts {
            let p = o + d * *t;
            assert!(s.distance(p) < 1e-9, "t {t}: {}", s.distance(p));
        }
        // a line above the arch misses it
        assert_eq!(line_params(Vec3::new(-1.0, 9.0, 0.0), Vec3::X, &s, 1e-6).unwrap().len(), 0);
        // along the generators: no closed form
        assert!(line_params(o, Vec3::Z, &s, 1e-6).is_none());
        // a circle profile agrees with the cylinder
        let circle = Curve::Circle { frame: ck_geom::Frame::identity(), r: 3.0 };
        let ex = Surface::Extrusion { profile: Box::new(circle), dir: Vec3::Z };
        let cy = Surface::Cylinder { frame: ck_geom::Frame::identity(), r: 3.0 };
        let (o, d) = (Vec3::new(-5.0, 1.0, 2.0), Vec3::new(1.0, 0.2, 0.1).normalize());
        let (mut a, mut b) = (line_params(o, d, &ex, 1e-6).unwrap(), line_params(o, d, &cy, 1e-6).unwrap());
        a.sort_by(|x, y| x.total_cmp(y));
        b.sort_by(|x, y| x.total_cmp(y));
        assert_eq!(a.len(), b.len());
        for (x, y) in a.iter().zip(&b) {
            assert!((x - y).abs() < 1e-9, "{a:?} vs {b:?}");
        }
    }
}
