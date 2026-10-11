//! Curvature combs (`comb` request): sample points and curvature vectors
//! along edges and along the iso-parameter lines of faces.
//!
//! Oracle shapes (tools/query_golden.mjs `comb`):
//! * edge -> one curve of `density + 1` points at uniform edge parameters
//!   (`t0 + (t1 - t0)·i/density`, the two end teeth 1e-9 of the range
//!   inside), `k` = curvature vector κ·N (towards the centre of curvature;
//!   zero on lines);
//! * face -> `lines` iso-lines per direction, line j at `(j + 0.5)/lines` of
//!   the face's uv box, each `density + 1` samples between the two ends of
//!   its chord inside the trimmed face; `dir` "u" lines vary u (v const) and
//!   come first, then "v"; `k` = curvature vector of the iso-curve. The uv
//!   box and chords come from the DISPLAY tessellation's boundary samples
//!   (so a circular cap's lines are slightly inside the exact circle), and
//!   a direction that is periodic and fully covered starts at -0.38 period.
//!
//! GUESS: boundary polygons of faces with holes / seams beyond the cylinder
//! family are only checked against the brep cases.

use ck_geom::analysis::curvature_vector;
use ck_geom::curve::Curve;
use ck_geom::surface::Surface;
use ck_geom::Vec2 as CVec2;
use ck_geom::Vec3;
use ck_topo::body::{Body, FaceId};

/// One comb curve: flat `p` / `k` triples and the face direction tag.
#[derive(Debug, Clone, PartialEq)]
pub struct CombCurve {
    pub p: Vec<f64>,
    pub k: Vec<f64>,
    pub dir: Option<&'static str>,
}

fn push(c: &mut CombCurve, p: Vec3, k: Vec3) {
    c.p.extend([p.x, p.y, p.z]);
    c.k.extend([k.x, k.y, k.z]);
}

/// Sample fraction of tooth `i` of `n`: the end teeth sit 1e-9 inside the
/// range (oracle), the rest at `i/n`.
fn tooth(i: usize, n: usize) -> f64 {
    if i == 0 {
        1e-9
    } else if i == n {
        1.0 - 1e-9
    } else {
        i as f64 / n as f64
    }
}

/// Comb of an edge curve on `[t0, t1]`.
pub fn edge_comb(curve: &Curve, t0: f64, t1: f64, density: usize) -> CombCurve {
    let mut n = density.max(1);
    if let Curve::BSpline(b) = curve {
        // "at least 5 teeth per span"
        let spans = b.knots.windows(2).filter(|w| w[1] > w[0]).count().max(1);
        n = n.max(5 * spans);
    }
    let mut c = CombCurve { p: vec![], k: vec![], dir: None };
    for i in 0..=n {
        let t = t0 + (t1 - t0) * tooth(i, n);
        let d = curve.derivs(t, 2);
        push(&mut c, d[0], curvature_vector(d[1], d[2]));
    }
    c
}

/// Segments of an edge's boundary polygon: lines 1, circles one per 2pi/25
/// of sweep, everything else 25 (oracle: a regular 25-gon for any full
/// circle, whatever its radius or the body size).
fn polygon_segments(curve: &Curve, ta: f64, tb: f64) -> usize {
    match curve {
        Curve::Line { .. } => 1,
        Curve::Circle { .. } => (((tb - ta).abs() / (std::f64::consts::TAU / 25.0) - 1e-9).ceil() as usize).max(1),
        _ => 25,
    }
}

/// The face's trimmed region in uv: loops as polygons (coarse boundary
/// samples, see [`polygon_segments`]) plus the uv box `[u0, v0, u1, v1]`.
fn uv_region(body: &Body, f: FaceId) -> (Vec<Vec<[f64; 2]>>, [f64; 4]) {
    let mut loops = vec![];
    let (mut lo, mut hi) = ([f64::INFINITY; 2], [f64::NEG_INFINITY; 2]);
    for &l in &body.face(f).loops {
        let mut poly: Vec<[f64; 2]> = vec![];
        let mut near: Option<CVec2> = None;
        for &c in &body.lp(l).coedges {
            let e = body.coedge(c).edge;
            let (ta, tb) = body.coedge_params(c);
            let n = polygon_segments(&body.edge(e).curve, ta, tb);
            // every sample but the last (the next coedge starts there)
            for i in 0..n {
                let t = ta + ((tb - ta) * i as f64) / n as f64;
                let uv = body.coedge_uv(c, t, near);
                near = Some(uv);
                poly.push([uv.x, uv.y]);
                for k in 0..2 {
                    lo[k] = lo[k].min([uv.x, uv.y][k]);
                    hi[k] = hi[k].max([uv.x, uv.y][k]);
                }
            }
        }
        loops.push(poly);
    }
    if !(lo[0].is_finite() && hi[0].is_finite()) {
        let ((u0, u1), (v0, v1)) = body.face(f).surface.domain();
        return (loops, [u0, v0, u1, v1]);
    }
    (loops, [lo[0], lo[1], hi[0], hi[1]])
}

/// Even-odd chords of the line `across = c` through the polygons: sorted
/// (a, b) pairs along the line.
fn chords(loops: &[Vec<[f64; 2]>], c: f64, swap: bool) -> Vec<(f64, f64)> {
    let g = |p: [f64; 2]| if swap { (p[1], p[0]) } else { (p[0], p[1]) }; // (along, across)
    let mut xs = vec![];
    for poly in loops {
        let n = poly.len();
        for i in 0..n {
            let (a, b) = (g(poly[i]), g(poly[(i + 1) % n]));
            if (a.1 <= c) != (b.1 <= c) {
                let t = (c - a.1) / (b.1 - a.1);
                xs.push(a.0 + t * (b.0 - a.0));
            }
        }
    }
    xs.sort_by(|a, b| a.total_cmp(b));
    xs.chunks(2).filter(|w| w.len() == 2).map(|w| (w[0], w[1])).collect()
}

/// Iso-line combs of a face. `dir`: "u", "v" or anything else = both.
pub fn face_comb(body: &Body, f: FaceId, density: usize, lines: usize, dir: &str) -> Vec<CombCurve> {
    let s: &Surface = &body.face(f).surface;
    let (loops, [bu0, bv0, bu1, bv1]) = uv_region(body, f);
    let (mut u0, mut v0, mut u1, mut v1) = (bu0, bv0, bu1, bv1);
    // fully periodic directions start at -0.38 period (the oracle's seamless
    // periodic window)
    let (pu, pv) = s.periods();
    let full = |p: Option<f64>, lo: f64, hi: f64| p.filter(|&p| hi - lo >= p - 1e-6);
    let (fu, fv) = (full(pu, bu0, bu1), full(pv, bv0, bv1));
    if let Some(p) = fu {
        (u0, u1) = (-0.38 * p, -0.38 * p + p);
    }
    if let Some(p) = fv {
        (v0, v1) = (-0.38 * p, -0.38 * p + p);
    }
    let n = density.max(1);
    let lines = lines.max(1);
    let mut out = vec![];
    for (swap, tag) in [(false, "u"), (true, "v")] {
        if (dir == "u" && tag == "v") || (dir == "v" && tag == "u") {
            continue;
        }
        for j in 0..lines {
            let frac = (j as f64 + 0.5) / lines as f64;
            // u-lines hold v fixed, v-lines hold u fixed
            let across = if swap { u0 + frac * (u1 - u0) } else { v0 + frac * (v1 - v0) };
            // the polygon lives in the surface's own parameter window
            let (poly_lo, per_across, full_along, window) = if swap {
                (bu0, pu, fv, v0)
            } else {
                (bv0, pv, fu, u0)
            };
            let mut q = across;
            if let Some(p) = per_across {
                while q < poly_lo {
                    q += p;
                }
                while q >= poly_lo + p {
                    q -= p;
                }
            }
            let cs = if loops.iter().all(|l| l.len() < 3) {
                let (a, b) = if swap { (v0, v1) } else { (u0, u1) };
                vec![(a, b)]
            } else {
                let mut cs = chords(&loops, q, swap);
                if let Some(p) = full_along {
                    // a chord spanning the whole period starts at the window
                    for c in cs.iter_mut() {
                        if c.1 - c.0 >= p - 1e-6 {
                            *c = (window, window + p);
                        }
                    }
                }
                cs
            };
            for (a, b) in cs {
                let mut c = CombCurve { p: vec![], k: vec![], dir: Some(tag) };
                for i in 0..=n {
                    let t = a + (b - a) * tooth(i, n);
                    let (u, v) = if swap { (across, t) } else { (t, across) };
                    let d = s.derivs2(u, v);
                    let k = if swap { curvature_vector(d.dv, d.dvv) } else { curvature_vector(d.du, d.duu) };
                    push(&mut c, d.p, k);
                }
                out.push(c);
            }
        }
    }
    out
}
