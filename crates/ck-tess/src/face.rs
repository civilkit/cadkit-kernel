//! Per-face triangulation in (u,v) space.
//!
//! Pipeline per face (mirrors what the oracle does, see README):
//! 1. Boundary: every non-seam coedge contributes the shared edge samples
//!    (params from [`crate::mesh::EdgeSampling`]), mapped through its pcurve
//!    to (u,v). 3D positions of boundary samples come from the EDGE curve so
//!    neighbouring faces share bit-identical vertices.
//! 2. Periodic directions are cut open at the face's cut value (largest
//!    boundary-sample gap, [`crate::curvature::periodic_cut`]); the face is
//!    meshed in the strip `[cut, cut + P)`. The cut lines (and pole lines /
//!    whole-domain rectangles for faces without trimming loops, e.g. full
//!    spheres and tori) become constrained boundary segments, sampled with the
//!    chord/angle rule.
//! 3. Constrained Delaunay triangulation of the boundary ([`crate::cdt`]) in a
//!    metric-scaled (u,v) frame, inside = odd parity. Pinned: the oracle
//!    meshes planar caps and cylinder sides from boundary points only (cap of
//!    an N-gon = N−2 triangles, cylinder side = 2N), which this reproduces.
//! 4. Refinement: insert the (u,v) centroid of every triangle whose centroid
//!    deviates more than the chord tolerance from the flat triangle, or whose
//!    centroid normal is more than the angle tolerance off a vertex normal
//!    (GUESS: the oracle's f3325 evaluates centroids with a 1/3 constant;
//!    the exact acceptance test was not decompiled).
//!
//! Triangles are CCW about the OUTWARD normal (flipped for `sense == false`).
use crate::cdt::{self, Pt};
use crate::curvature::{self, Tol};
use ck_geom::surface::Surface;
use ck_geom::{Vec2, Vec3};
use ck_topo::body::{Body, FaceId};
use std::collections::HashMap;

/// Periodic cuts of a face: `(u cut, v cut)` for the periodic directions.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct FaceCut {
    pub u: Option<f64>,
    pub v: Option<f64>,
}

/// Triangulated face.
#[derive(Debug, Clone, Default)]
pub struct FaceMesh {
    pub face: Option<FaceId>,
    /// (u,v) of every vertex (strip coordinates: cut copies differ by a period).
    pub uv: Vec<Vec2>,
    /// 3D position of every vertex.
    pub pts: Vec<Vec3>,
    /// Outward unit normal of every vertex.
    pub nrm: Vec<Vec3>,
    /// Triangles, CCW about the outward normal.
    pub tris: Vec<[u32; 3]>,
}

impl FaceMesh {
    /// Outward surface normal at (u,v) (fallback near poles).
    pub fn outward(surf: &Surface, sense: bool, uv: Vec2) -> Vec3 {
        let n = curvature::surface_normal(surf, uv.x, uv.y);
        if sense {
            n
        } else {
            -n
        }
    }
}

/// One boundary vertex before deduplication.
#[derive(Clone, Copy)]
struct BPt {
    uv: Vec2,
    p: Vec3,
}

struct Builder<'a> {
    surf: &'a Surface,
    pts: Vec<BPt>,
    index: HashMap<(u64, u64), usize>,
    segs: Vec<(usize, usize)>,
}

impl<'a> Builder<'a> {
    fn add(&mut self, uv: Vec2, p: Vec3) -> usize {
        let key = (uv.x.to_bits(), uv.y.to_bits());
        if let Some(&i) = self.index.get(&key) {
            return i;
        }
        self.pts.push(BPt { uv, p });
        self.index.insert(key, self.pts.len() - 1);
        self.pts.len() - 1
    }
    fn add_surf(&mut self, uv: Vec2, eval_uv: Vec2) -> usize {
        let mut p = self.surf.eval(eval_uv.x, eval_uv.y);
        // weld onto an existing boundary point (poles: S(u, ±π/2) differs in the last bits)
        let tol = 1e-12 * (1.0 + p.len());
        if let Some(q) = self.pts.iter().find(|q| q.p.dist(p) <= tol) {
            p = q.p;
        }
        self.add(uv, p)
    }
    fn seg(&mut self, a: usize, b: usize) {
        if a != b {
            self.segs.push((a, b));
        }
    }
}

/// Samples the iso-parameter segment `uv(s), s in [0,1]` with the chord / angle rule
/// (uniform initial split then bisection, depth ≤ 10). Returns the interior + end parameters in (0,1].
fn sample_iso(surf: &Surface, a: Vec2, b: Vec2, tol: &Tol) -> Vec<f64> {
    fn rec(surf: &Surface, a: Vec2, b: Vec2, s0: f64, s1: f64, tol: &Tol, depth: u32, out: &mut Vec<f64>) {
        let at = |s: f64| a.lerp(b, s);
        let (u0, u1) = (at(s0), at(s1));
        let (p0, p1) = (surf.eval(u0.x, u0.y), surf.eval(u1.x, u1.y));
        let sm = 0.5 * (s0 + s1);
        let um = at(sm);
        let pm = surf.eval(um.x, um.y);
        let d = p1 - p0;
        let l = d.len();
        let sag = if l < 1e-300 { pm.dist(p0) } else { d.cross(pm - p0).len() / l };
        let n0 = curvature::surface_normal(surf, u0.x, u0.y);
        let n1 = curvature::surface_normal(surf, u1.x, u1.y);
        let ang = curvature::normal_angle(n0, n1);
        if depth < 10 && (sag > tol.chord || ang > tol.angle) {
            rec(surf, a, b, s0, sm, tol, depth + 1, out);
            rec(surf, a, b, sm, s1, tol, depth + 1, out);
        } else {
            out.push(s1);
        }
    }
    let mut out = vec![];
    // degenerate iso line (pole): one 3D point, sample along the parameter with the angle step
    {
        let (p0, p1) = (surf.eval(a.x, a.y), surf.eval(b.x, b.y));
        let m = a.lerp(b, 0.5);
        let pm = surf.eval(m.x, m.y);
        let scale = 1.0 + p0.len();
        if p0.dist(p1) <= 1e-12 * scale && p0.dist(pm) <= 1e-12 * scale {
            let n = (a.dist(b) / tol.angle).ceil().max(1.0) as usize;
            return (1..=n).map(|i| i as f64 / n as f64).collect();
        }
    }
    let n0 = 4usize;
    for i in 0..n0 {
        rec(surf, a, b, i as f64 / n0 as f64, (i + 1) as f64 / n0 as f64, tol, 0, &mut out);
    }
    // collapse to the minimal set: if the whole line is flat (e.g. a cylinder ruling) keep only the end
    if out.len() == n0 {
        let ends = [a, b];
        let (p0, p1) = (surf.eval(ends[0].x, ends[0].y), surf.eval(ends[1].x, ends[1].y));
        let all_flat = (1..n0).all(|i| {
            let s = i as f64 / n0 as f64;
            let u = a.lerp(b, s);
            let pm = surf.eval(u.x, u.y);
            let d = p1 - p0;
            let l = d.len();
            let sag = if l < 1e-300 { pm.dist(p0) } else { d.cross(pm - p0).len() / l };
            sag <= tol.chord * 1e-3
        }) && curvature::normal_angle(
            curvature::surface_normal(surf, a.x, a.y),
            curvature::surface_normal(surf, b.x, b.y),
        ) <= tol.angle;
        if all_flat {
            return vec![1.0];
        }
    }
    out
}

/// Shift `x` by whole periods so the reference `m` lands in `[c, c + p)`.
fn shift_into(x: f64, m: f64, c: f64, p: f64) -> f64 {
    let k = ((m - c) / p).floor();
    x - k * p
}

/// Triangulates face `f`. `params[e]` = final sorted param list of edge `e`.
pub fn mesh_face(body: &Body, f: FaceId, params: &[Vec<f64>], cut: FaceCut, tol: &Tol) -> Result<FaceMesh, String> {
    let face = body.face(f);
    crate::tr!(9, 0, 0, f.idx(), 0);
    let surf = &face.surface;
    let (pu, pv) = surf.periods();
    // The original opens a periodic direction at the window [cut - p, cut] (its uv bounds are exactly that window).
    let cu = pu.map(|p| cut.u.unwrap_or(p) - p);
    let cv = pv.map(|p| cut.v.unwrap_or(p) - p);

    let mut b = Builder { surf, pts: vec![], index: HashMap::new(), segs: vec![] };
    let eps_u = pu.map(|p| p * 1e-9).unwrap_or(0.0);
    let eps_v = pv.map(|p| p * 1e-9).unwrap_or(0.0);

    // ---- 1. boundary polylines (unwrapped along each loop)
    let mut v_lo = f64::INFINITY;
    let mut v_hi = f64::NEG_INFINITY;
    let mut u_lo = f64::INFINITY;
    let mut u_hi = f64::NEG_INFINITY;
    let mut polys: Vec<Vec<BPt>> = vec![];
    for &l in &face.loops {
        let lp = body.lp(l);
        if !lp.alive {
            continue;
        }
        let mut poly: Vec<BPt> = vec![];
        let mut prev: Option<Vec2> = None;
        let backwards = false;
        for &c in lp.coedges.iter() {
            let ce = body.coedge(c);
            if !ce.alive {
                continue;
            }
            let e = ce.edge;
            let seam = body.is_seam(e);
            let ed = body.edge(e);
            let mut ps: Vec<f64> = params.get(e.idx()).cloned().unwrap_or_default();
            if ps.len() < 2 {
                ps = vec![ed.t0, ed.t1];
            }
            if ce.rev != backwards {
                ps.reverse();
            }
            let (va, vb) = {
                let (s0, s1) = (body.vertex(body.coedge_start(c)).p, body.vertex(body.coedge_end(c)).p);
                if backwards { (s1, s0) } else { (s0, s1) }
            };
            let mut pl: Vec<BPt> = Vec::with_capacity(ps.len());
            for (i, &t) in ps.iter().enumerate() {
                let mut uv = body.coedge_uv(c, t, prev);
                if let (Some(per), Some(pr)) = (pu, prev) {
                    uv.x = ck_geom::pcurve::unwrap_near(uv.x, pr.x, per);
                }
                if let (Some(per), Some(pr)) = (pv, prev) {
                    uv.y = ck_geom::pcurve::unwrap_near(uv.y, pr.y, per);
                }
                let p = if i == 0 {
                    va
                } else if i + 1 == ps.len() {
                    vb
                } else {
                    ed.curve.eval(t)
                };
                prev = Some(uv);
                v_lo = v_lo.min(uv.y);
                v_hi = v_hi.max(uv.y);
                u_lo = u_lo.min(uv.x);
                u_hi = u_hi.max(uv.x);
                pl.push(BPt { uv, p });
            }
            if std::env::var("CK_DUMP3").is_ok() { eprintln!("COEDGE f{:?} c={:?} rev={} backwards={} n={} first={:?} last={:?}", f, c, ce.rev, backwards, pl.len(), pl.first().map(|q| q.uv), pl.last().map(|q| q.uv)); }
            if seam {
                // seams are interior once the face is cut open; they only bound the v/u range
                poly.push(BPt { uv: Vec2::new(f64::NAN, f64::NAN), p: Vec3::ZERO });
                continue;
            }
            if let Some(last) = poly.last().copied() {
                if last.uv.x.is_finite() {
                    join(&mut poly, last, pl[0], tol);
                    if same_uv(last.uv, pl[0].uv) {
                        pl.remove(0);
                    }
                }
            }
            poly.extend(pl);
        }
        // close the loop
        if poly.len() >= 2 && poly.iter().all(|q| q.uv.x.is_finite()) {
            let (first, last) = (poly[0], *poly.last().unwrap());
            if same_uv(first.uv, last.uv) {
                poly.pop();
            } else {
                join(&mut poly, last, first, tol);
            }
            if !face.sense && poly.len() > 1 {
                // a reversed-sense face walks its polygon backwards from the same first vertex
                poly[1..].reverse();
            }
        }
        // split at NaN markers into open chains; a loop without markers is closed
        polys.push(poly);
    }

    // ---- 2. map into the strip and add boundary segments
    let mut chains: Vec<(Vec<BPt>, bool)> = vec![];
    for poly in polys {
        let closed = !poly.iter().any(|q| !q.uv.x.is_finite());
        if closed {
            chains.push((poly, true));
        } else {
            // rotate so a marker is first, then split
            let start = poly.iter().position(|q| !q.uv.x.is_finite()).unwrap();
            let mut rot: Vec<BPt> = poly[start..].to_vec();
            rot.extend_from_slice(&poly[..start]);
            let mut cur: Vec<BPt> = vec![];
            for q in rot {
                if !q.uv.x.is_finite() {
                    if cur.len() >= 2 {
                        chains.push((std::mem::take(&mut cur), false));
                    }
                    cur.clear();
                } else {
                    cur.push(q);
                }
            }
            if cur.len() >= 2 {
                chains.push((cur, false));
            }
        }
    }
    // an open chain whose ends meet in uv (closed edge between slits) is a closed loop
    for (ch, closed) in chains.iter_mut() {
        if !*closed && ch.len() >= 3 && same_uv(ch[0].uv, ch[ch.len() - 1].uv) {
            ch.pop();
            *closed = true;
        }
    }
    let snap = |x: f64, c: Option<f64>, p: Option<f64>, eps: f64| -> f64 {
        match (c, p) {
            (Some(c), Some(p)) => {
                if (x - c).abs() <= eps {
                    c
                } else if (x - (c + p)).abs() <= eps {
                    c + p
                } else {
                    x
                }
            }
            _ => x,
        }
    };
    let mut seg_list: Vec<(BPt, BPt)> = vec![];
    for (chain, closed) in &chains {
        let n = chain.len();
        let m = if *closed { n } else { n - 1 };
        for i in 0..m {
            let (mut a, mut q) = (chain[i], chain[(i + 1) % n]);
            if a.uv.dist(q.uv) == 0.0 {
                continue;
            }
            if let (Some(c), Some(p)) = (cu, pu) {
                let mid = 0.5 * (a.uv.x + q.uv.x);
                a.uv.x = shift_into(a.uv.x, mid, c, p);
                q.uv.x = shift_into(q.uv.x, mid, c, p);
            }
            if let (Some(c), Some(p)) = (cv, pv) {
                let mid = 0.5 * (a.uv.y + q.uv.y);
                a.uv.y = shift_into(a.uv.y, mid, c, p);
                q.uv.y = shift_into(q.uv.y, mid, c, p);
            }
            a.uv.x = snap(a.uv.x, cu, pu, eps_u);
            q.uv.x = snap(q.uv.x, cu, pu, eps_u);
            a.uv.y = snap(a.uv.y, cv, pv, eps_v);
            q.uv.y = snap(q.uv.y, cv, pv, eps_v);
            seg_list.push((a, q));
        }
    }
    for (a, q) in &seg_list {
        let ia = b.add(a.uv, a.p);
        let iq = b.add(q.uv, q.p);
        b.seg(ia, iq);
    }

    // ---- 3. cut lines / domain rectangle
    let (dom_u, dom_v) = surf.domain();
    let has_loops = !seg_list.is_empty();
    if !v_lo.is_finite() {
        v_lo = if dom_v.0.is_finite() { dom_v.0 } else { 0.0 };
        v_hi = if dom_v.1.is_finite() { dom_v.1 } else { 1.0 };
    }
    if !u_lo.is_finite() {
        u_lo = if dom_u.0.is_finite() { dom_u.0 } else { 0.0 };
        u_hi = if dom_u.1.is_finite() { dom_u.1 } else { 1.0 };
    }
    // iso line in the strip: points uv(s) of the segment a..b, evaluated at the wrapped (cut) parameter
    let iso = |b: &mut Builder, a: Vec2, z: Vec2, eval_shift: Vec2| {
        let ss = sample_iso(surf, a, z, tol);
        let mut prev = {
            let uv = a;
            let i = match b.index.get(&(uv.x.to_bits(), uv.y.to_bits())) {
                Some(&i) => i,
                None => b.add_surf(uv, uv - eval_shift),
            };
            i
        };
        for s in ss {
            let uv = if s == 1.0 { z } else { a.lerp(z, s) };
            let i = match b.index.get(&(uv.x.to_bits(), uv.y.to_bits())) {
                Some(&i) => i,
                None => b.add_surf(uv, uv - eval_shift),
            };
            b.seg(prev, i);
            prev = i;
        }
    };
    match (cu, pu, cv, pv) {
        (Some(c), Some(p), Some(cv0), Some(qv)) if !has_loops => {
            // torus-like: full rectangle
            let (a, bb, cc, d) = (
                Vec2::new(c, cv0),
                Vec2::new(c + p, cv0),
                Vec2::new(c + p, cv0 + qv),
                Vec2::new(c, cv0 + qv),
            );
            iso(&mut b, a, bb, Vec2::ZERO);
            iso(&mut b, bb, cc, Vec2::new(p, 0.0));
            iso(&mut b, d, cc, Vec2::new(0.0, qv));
            iso(&mut b, a, d, Vec2::ZERO);
        }
        (Some(c), Some(p), _, _) => {
            if !has_loops {
                // full sphere / revolution: rectangle in the v range of the seams / domain
                let (a, bb, cc, d) =
                    (Vec2::new(c, v_lo), Vec2::new(c + p, v_lo), Vec2::new(c + p, v_hi), Vec2::new(c, v_hi));
                iso(&mut b, a, bb, Vec2::ZERO);
                iso(&mut b, bb, cc, Vec2::new(p, 0.0));
                iso(&mut b, d, cc, Vec2::ZERO);
                iso(&mut b, a, d, Vec2::ZERO);
            } else {
                // boundary points on the cut lines -> inside intervals
                let mut on: Vec<f64> = b.pts.iter().filter(|q| q.uv.x == c).map(|q| q.uv.y).collect();
                on.extend(b.pts.iter().filter(|q| q.uv.x == c + p).map(|q| q.uv.y));
                on.sort_by(|x, y| x.total_cmp(y));
                on.dedup();
                for w in on.windows(2) {
                    let vm = 0.5 * (w[0] + w[1]);
                    if inside_parity(&seg_list, Vec2::new(c + 0.5 * p * 1e-6, vm)) {
                        iso(&mut b, Vec2::new(c, w[0]), Vec2::new(c, w[1]), Vec2::ZERO);
                        iso(&mut b, Vec2::new(c + p, w[0]), Vec2::new(c + p, w[1]), Vec2::new(p, 0.0));
                    }
                }
            }
        }
        (None, None, Some(c), Some(p)) if has_loops => {
            let mut on: Vec<f64> = b.pts.iter().filter(|q| q.uv.y == c).map(|q| q.uv.x).collect();
            on.extend(b.pts.iter().filter(|q| q.uv.y == c + p).map(|q| q.uv.x));
            on.sort_by(|x, y| x.total_cmp(y));
            on.dedup();
            for w in on.windows(2) {
                let um = 0.5 * (w[0] + w[1]);
                if inside_parity_v(&seg_list, Vec2::new(um, c + 0.5 * p * 1e-6)) {
                    iso(&mut b, Vec2::new(w[0], c), Vec2::new(w[1], c), Vec2::ZERO);
                    iso(&mut b, Vec2::new(w[0], c + p), Vec2::new(w[1], c + p), Vec2::new(0.0, p));
                }
            }
        }
        _ => {}
    }
    let _ = (u_lo, u_hi);
    if b.pts.len() < 3 {
        return Err("face has fewer than 3 boundary points".into());
    }

    // ---- 4. CDT in a metric-scaled frame
    // uv bounds of the face (original f3578): the sample bbox grown by 1e-3 of its extent on each side; a periodic direction
    // is exactly its window.
    let infl = |lo: f64, hi: f64| -> (f64, f64) {
        let e = (hi - lo) * 1e-3;
        (lo - e, hi + e)
    };
    let ubnd = match (pu, cut.u) {
        (Some(p), Some(c)) => (c - p, c),
        _ => infl(u_lo, u_hi),
    };
    let vbnd = match (pv, cut.v) {
        (Some(p), Some(c)) => (c - p, c),
        _ => infl(v_lo, v_hi),
    };
    let cuv = Vec2::new((ubnd.0 + ubnd.1) * 0.5, (vbnd.0 + vbnd.1) * 0.5);
    if std::env::var("CK_BND").is_ok() { eprintln!("BND {:?} {:?}", ubnd, vbnd); }
    crate::tr!(20, ubnd.0, vbnd.0, ubnd.1, vbnd.1);
    let (su, sv) = {
        let (su, sv) = metric_scale(surf, ubnd, vbnd);
        (if su.is_nan() { 1e-12 } else { su.max(1e-12) }, if sv.is_nan() { 1e-12 } else { sv.max(1e-12) })
    };
    let mir = !face.sense && std::env::var("CK_MIR").is_ok();
    let mx = if mir { -1.0 } else { 1.0 };
    let to_c = |uv: Vec2| -> Pt { [mx * (uv.x - cuv.x) * su, (uv.y - cuv.y) * sv] };
    let from_c = |p: Pt| -> Vec2 { Vec2::new(mx * p[0] / su + cuv.x, p[1] / sv + cuv.y) };
    if let Ok(spec) = std::env::var("CK_RING") {
        // EXPERIMENT: reorder a single boundary cycle (rotation, direction)
        let mut it = spec.split(',');
        let st: usize = it.next().unwrap().parse().unwrap();
        let dir: usize = it.next().unwrap().parse().unwrap();
        let n = b.pts.len();
        let mut adj: Vec<Vec<usize>> = vec![vec![]; n];
        for &(x, y) in &b.segs { adj[x].push(y); adj[y].push(x); }
        if adj.iter().all(|a| a.len() == 2) && b.segs.len() == n {
            let mut ring = vec![0usize];
            let mut prev = usize::MAX;
            let mut cur = 0usize;
            loop {
                let nx = if adj[cur][0] != prev { adj[cur][0] } else { adj[cur][1] };
                if nx == 0 { break; }
                ring.push(nx);
                prev = cur;
                cur = nx;
                if ring.len() > n { break; }
            }
            if ring.len() == n {
                if dir == 1 { ring.reverse(); }
                ring.rotate_left(st % n);
                for (j, &i) in ring.iter().enumerate().take(5) { eprintln!("RINGPT f{:?} {} {:?} cu={:?} cv={:?}", f, j, b.pts[i].uv, cu, cv); }
                let np: Vec<BPt> = ring.iter().map(|&i| b.pts[i]).collect();
                b.pts = np;
                b.segs = (0..n).map(|i| (i, (i + 1) % n)).collect();
            }
        }
    }
    if let Ok(spec) = std::env::var("CK_INS") {
        let ord: Vec<usize> = spec.split(',').map(|x| x.parse().unwrap()).collect();
        if ord.len() == b.pts.len() {
            let mut inv = vec![0usize; ord.len()];
            for (ni, &oi) in ord.iter().enumerate() { inv[oi] = ni; }
            let np: Vec<BPt> = ord.iter().map(|&i| b.pts[i]).collect();
            b.segs = b.segs.iter().map(|&(x, y)| (inv[x], inv[y])).collect();
            b.pts = np;
        }
    }
    let cpts: Vec<Pt> = b.pts.iter().map(|q| to_c(q.uv)).collect();
    if std::env::var("CK_DUMP").is_ok() { eprintln!("CPTS_BITS {:?}", cpts.iter().map(|p| [p[0].to_bits(), p[1].to_bits()]).collect::<Vec<_>>()); }
    let mut tri = cdt::triangulate(&cpts, &b.segs)?;
    if std::env::var("CK_DUMP2").is_ok() { eprintln!("F2 {:?} sense={} cpts={:?} vmap={:?} tris={:?}", f, face.sense, cpts, tri.vmap, tri.cdt.inside_tris().iter().map(|x| x.1).collect::<Vec<_>>()); }
    // vertex data indexed by CDT vertex
    let mut vdata: HashMap<u32, (Vec2, Vec3)> = HashMap::new();
    for (i, &v) in tri.vmap.iter().enumerate() {
        vdata.entry(v).or_insert((b.pts[i].uv, b.pts[i].p));
    }
    let wrap_eval = |uv: Vec2| -> Vec2 {
        let mut e = uv;
        if let (Some(c), Some(p)) = (cu, pu) {
            if e.x >= c + p {
                e.x -= p;
            }
        }
        if let (Some(c), Some(p)) = (cv, pv) {
            if e.y >= c + p {
                e.y -= p;
            }
        }
        e
    };

    // ---- 5. refinement
    let budget = 20000 + 50 * b.pts.len();
    let mut inserted = 0usize;
    let sense = face.sense;
    for _pass in 0..64 {
        let mut cands: Vec<(Pt, u32)> = vec![];
        for (ti, v) in tri.cdt.inside_tris() {
            let ids = [v[0], v[1], v[2]];
            let get = |k: u32| -> (Vec2, Vec3) {
                match vdata.get(&k) {
                    Some(&d) => d,
                    None => {
                        let uv = from_c(tri.cdt.pts[k as usize]);
                        let e = wrap_eval(uv);
                        (uv, surf.eval(e.x, e.y))
                    }
                }
            };
            let (a, bb, c) = (get(ids[0]), get(ids[1]), get(ids[2]));
            let n = (bb.1 - a.1).cross(c.1 - a.1);
            let area2 = n.len();
            if !(area2 > 1e-24 * (1.0 + a.1.len2())) {
                continue;
            }
            let nn = n / area2;
            let cuv3 = (a.0 + bb.0 + c.0) * (1.0 / 3.0);
            let e = wrap_eval(cuv3);
            let sp = surf.eval(e.x, e.y);
            let dev = (sp - a.1).dot(nn).abs();
            // edge-midpoint chord height (GUESS: the oracle also refines diagonal edges of ruled faces)
            let mut dev = dev;
            for (x, y) in [(a, bb), (bb, c), (c, a)] {
                let w = wrap_eval((x.0 + y.0) * 0.5);
                let sm = surf.eval(w.x, w.y);
                dev = dev.max(sm.dist((x.1 + y.1) * 0.5));
            }
            let ns = curvature::surface_normal(surf, e.x, e.y);
            let mut ang: f64 = 0.0;
            for q in [a.0, bb.0, c.0] {
                let w = wrap_eval(q);
                ang = ang.max(curvature::normal_angle(ns, curvature::surface_normal(surf, w.x, w.y)));
            }
            if dev > tol.chord || ang > tol.angle {
                let cp = to_c(cuv3);
                cands.push((cp, ti));
            }
        }
        if cands.is_empty() {
            break;
        }
        let mut any = false;
        for (p, hint) in cands {
            if inserted >= budget {
                break;
            }
            if let Some(v) = tri.cdt.insert_interior(p) {
                let uv = from_c(p);
                let e = wrap_eval(uv);
                vdata.insert(v, (uv, surf.eval(e.x, e.y)));
                inserted += 1;
                any = true;
            }
        }
        if !any || inserted >= budget {
            break;
        }
    }

    // ---- 6. output
    let mut out = FaceMesh { face: Some(f), ..Default::default() };
    let mut local: HashMap<u32, u32> = HashMap::new();
    for (_, v) in tri.cdt.inside_tris() {
        let mut ids = [0u32; 3];
        for k in 0..3 {
            let cv = v[k];
            let id = *local.entry(cv).or_insert_with(|| {
                let (uv, p) = match vdata.get(&cv) {
                    Some(&d) => d,
                    None => {
                        let uv = from_c(tri.cdt.pts[cv as usize]);
                        let e = wrap_eval(uv);
                        (uv, surf.eval(e.x, e.y))
                    }
                };
                let e = wrap_eval(uv);
                out.uv.push(uv);
                out.pts.push(p);
                out.nrm.push(FaceMesh::outward(surf, sense, e));
                (out.pts.len() - 1) as u32
            });
            ids[k] = id;
        }
        let (pa, pb, pc) = (out.pts[ids[0] as usize], out.pts[ids[1] as usize], out.pts[ids[2] as usize]);
        if (pb - pa).cross(pc - pa).len2() <= 0.0 {
            continue; // degenerate (pole)
        }
        let em = std::env::var("CK_EMIT").ok().and_then(|x| x.parse::<u32>().ok()).unwrap_or(0);
        out.tris.push(if sense || mir { ids } else { match em { 1 => [ids[1], ids[0], ids[2]], 2 => [ids[2], ids[1], ids[0]], _ => [ids[0], ids[2], ids[1]] } });
    }
    if out.tris.is_empty() {
        return Err("face produced no triangles".into());
    }
    Ok(out)
}

/// Per-axis scale of the centred CDT frame (original f3573, with the cone special case of f3325):
/// plane (1,1), cylinder (r,1), cone (max radius over the v range, 1/cos), sphere (r,r), torus (R+r,r), else |Su|,|Sv| at the domain centre.
fn metric_scale(surf: &Surface, ur: (f64, f64), vr: (f64, f64)) -> (f64, f64) {
    match surf {
        Surface::Plane { .. } if std::env::var("CK_PL").is_err() => (1.0, 1.0),
        Surface::Cylinder { r, .. } if std::env::var("CK_PL").is_err() => (*r, 1.0),
        Surface::Cone { r, semi_angle, .. } => {
            let sv = 1.0 / semi_angle.cos();
            if vr.0.is_finite() && vr.1.is_finite() {
                let t = semi_angle.tan();
                let a = (vr.1 * t + r).abs();
                let b = (vr.0 * t + r).abs();
                let m = if a.is_nan() { b } else if b.is_nan() { a } else { a.max(b) };
                (if m.is_nan() { 0.001 } else { m.max(0.001) }, sv)
            } else {
                (if r.is_nan() { 0.001 } else { r.abs() }, sv)
            }
        }
        Surface::Sphere { r, .. } if std::env::var("CK_PL").is_err() => (*r, *r),
        Surface::Torus { major, minor, .. } => (minor + major, *minor),
        _ => {
            let (du, dv) = surf.domain();
            let mid = |a: (f64, f64)| if a.0.is_finite() && a.1.is_finite() { (a.0 + a.1) * 0.5 } else { 0.0 };
            let _ = (ur, vr);
            let (a, b) = surf.partials(mid(du), mid(dv));
            let f = |x: f64| if x.is_nan() { 1e-9 } else { x.max(1e-9) };
            (f(a.len()), f(b.len()))
        }
    }
}

fn same_uv(a: Vec2, b: Vec2) -> bool {
    a.dist(b) <= 1e-9 * (1.0 + a.x.abs() + a.y.abs())
}

/// Connects the end `a` of one coedge to the start `b` of the next: a pole
/// (same 3D point, different uv) is sampled along uv with the angle step.
fn join(poly: &mut Vec<BPt>, a: BPt, b: BPt, tol: &Tol) {
    if same_uv(a.uv, b.uv) || a.p.dist(b.p) > 1e-9 * (1.0 + b.p.len()) {
        return;
    }
    let n = (a.uv.dist(b.uv) / tol.angle).ceil().max(1.0) as usize;
    for i in 1..n {
        poly.push(BPt { uv: a.uv.lerp(b.uv, i as f64 / n as f64), p: b.p });
    }
}

/// Parity of boundary crossings of a ray from `p` towards −v.
fn inside_parity(segs: &[(BPt, BPt)], p: Vec2) -> bool {
    let mut odd = false;
    for (a, b) in segs {
        let (x0, x1) = (a.uv.x, b.uv.x);
        if (x0 > p.x) != (x1 > p.x) {
            let t = (p.x - x0) / (x1 - x0);
            let y = a.uv.y + t * (b.uv.y - a.uv.y);
            if y < p.y {
                odd = !odd;
            }
        }
    }
    odd
}

/// Parity of boundary crossings of a ray from `p` towards −u.
fn inside_parity_v(segs: &[(BPt, BPt)], p: Vec2) -> bool {
    let mut odd = false;
    for (a, b) in segs {
        let (y0, y1) = (a.uv.y, b.uv.y);
        if (y0 > p.y) != (y1 > p.y) {
            let t = (p.y - y0) / (y1 - y0);
            let x = a.uv.x + t * (b.uv.x - a.uv.x);
            if x < p.x {
                odd = !odd;
            }
        }
    }
    odd
}
