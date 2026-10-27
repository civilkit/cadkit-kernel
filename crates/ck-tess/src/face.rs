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
//! 4. Refinement: insert the (u,v) centroid of every triangle whose surface
//!    depth at the centroid exceeds the chord tolerance. Pinned by A/B on the
//!    sphere: the oracle's max accepted centroid depth == chord tol exactly,
//!    with no normal-angle term (36.9deg observed vs 17.2deg tol) and no
//!    edge-midpoint chord term.
//!
//! Triangles are CCW about the OUTWARD normal (flipped for `sense == false`).
use crate::cdt::{self, Pt};
use crate::curvature::{self, Tol};
use ck_geom::surface::Surface;
use ck_geom::{Vec2, Vec3};
use ck_topo::body::{Body, FaceId};
use ck_core::fxhash::{FxHashMap as HashMap, FxHashSet as HashSet};

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
    /// Cut windows `(start, period)` of the periodic directions (a strip point at `start + period` evaluates at `start`).
    pub win: [Option<(f64, f64)>; 2],
    /// The boundary graph handed to the CDT had a vertex of odd degree (an unclosed loop: the CDT then fills
    /// only part of the face). `mesh_face` retries a periodic face on the universal cover.
    pub open: bool,
    /// Boundary segments the CDT could not recover (the parity flood may then leak across them).
    pub unrecovered: u32,
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
    /// Junction of a merged circle chain: dropped from the polygon once the loops are split into chains.
    virt: bool,
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
        self.pts.push(BPt { uv, p, virt: false });
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
    /// Reorders the points so that, when the segments form one simple cycle, the ring starts at the (u, v)-lexicographically
    /// smallest point and runs counter-clockwise (after the first point reversed for a reversed-sense face).
    fn canonical_ring(&mut self, sense: bool) {
        let n = self.pts.len();
        if n < 3 || self.segs.len() != n {
            return;
        }
        let mut adj: Vec<Vec<usize>> = vec![vec![]; n];
        for &(x, y) in &self.segs {
            adj[x].push(y);
            adj[y].push(x);
        }
        if adj.iter().any(|a| a.len() != 2) {
            return;
        }
        let start = (0..n)
            .min_by(|&i, &j| self.pts[i].uv.x.total_cmp(&self.pts[j].uv.x).then(self.pts[i].uv.y.total_cmp(&self.pts[j].uv.y)))
            .unwrap();
        let mut ring = vec![start];
        let (mut prev, mut cur) = (usize::MAX, start);
        loop {
            let nx = if adj[cur][0] != prev { adj[cur][0] } else { adj[cur][1] };
            if nx == start {
                break;
            }
            ring.push(nx);
            prev = cur;
            cur = nx;
            if ring.len() > n {
                return;
            }
        }
        if ring.len() != n {
            return;
        }
        let area: f64 = (0..n)
            .map(|i| {
                let (p, q) = (self.pts[ring[i]].uv, self.pts[ring[(i + 1) % n]].uv);
                p.x * q.y - q.x * p.y
            })
            .sum();
        if area < 0.0 {
            ring[1..].reverse();
        }
        let _ = sense;
        self.pts = ring.iter().map(|&i| self.pts[i]).collect();
        self.segs = (0..n).map(|i| (i, (i + 1) % n)).collect();
        self.index.clear();
        for (i, q) in self.pts.iter().enumerate() {
            self.index.insert((q.uv.x.to_bits(), q.uv.y.to_bits()), i);
        }
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
///
/// Periodic faces are normally opened at a cut window (the strip approach of
/// the original kernel); when that cannot close the region (a torus
/// fragment bounded by intersection loops that cross both seams) the face is
/// retried in the universal cover: loops are unwrapped continuously through
/// their seam coedges and triangulated as ordinary closed polygons.
pub fn mesh_face(body: &Body, f: FaceId, params: &[Vec<f64>], virt: &[(bool, bool)], ssi: &[f64], cut: FaceCut, tol: &Tol) -> Result<FaceMesh, String> {
    let first = match mesh_face_strict(body, f, params, virt, ssi, cut, tol) {
        Ok(m) => Ok(m),
        Err(e) => {
            // Coedge ends that miss each other by a few 1e-9 of the parameter range (fitted trim curves: a notch
            // of a spline face closes 8e-9 apart) leave a sliver segment the CDT cannot take. Weld them at 1e-6
            // and retry; every face that meshed before is untouched.
            let r = with_same_uv(1e-6, || mesh_face_strict(body, f, params, virt, ssi, cut, tol));
            r.map_err(|e2| format!("{e}; welded: {e2}"))
        }
    };
    // A face that meshed cleanly is done (no exact-area integral: it costs more than the tessellation itself).
    if matches!(&first, Ok(m) if !m.open && m.unrecovered == 0 && !m.tris.is_empty()) {
        return first;
    }
    escalate(body, f, params, virt, ssi, cut, tol, first)
}

/// Runs `g` with trim ends welded within `rel` of the parameter range.
fn with_same_uv<T>(rel: f64, g: impl FnOnce() -> T) -> T {
    SAME_UV_REL.with(|c| c.set(rel));
    let r = g();
    SAME_UV_REL.with(|c| c.set(1e-9));
    r
}

fn mesh_area(m: &FaceMesh) -> f64 {
    m.tris.iter().map(|t| 0.5 * (m.pts[t[1] as usize] - m.pts[t[0] as usize]).cross(m.pts[t[2] as usize] - m.pts[t[0] as usize]).len()).sum()
}

/// The first attempt left the face open, with unrecovered trims, or empty. Try every other way the face can be meshed (the
/// strip with welded trim ends, the universal cover with and without the weld), judge each by the exact face area, and keep
/// the one that covers it best: a face is never dropped while any attempt produced triangles.
#[allow(clippy::too_many_arguments)]
fn escalate(body: &Body, f: FaceId, params: &[Vec<f64>], virt: &[(bool, bool)], ssi: &[f64], cut: FaceCut, tol: &Tol, first: Result<FaceMesh, String>) -> Result<FaceMesh, String> {
    let exact = ck_topo::mass::face_integral::<1>(body, f, 1, &|_, n| [n.len()])[0].abs();
    let periodic = body.face(f).surface.periods() != (None, None);
    let score = |m: &FaceMesh| -> f64 {
        if m.tris.is_empty() || !(exact > 0.0) {
            return f64::INFINITY;
        }
        // |log ratio|: an over-covering mesh is as bad as an under-covering one
        (mesh_area(m) / exact).ln().abs() + if m.unrecovered > 0 { 0.02 } else { 0.0 }
    };
    let good = |m: &FaceMesh| -> bool { score(m) < 0.05 };
    let mut best: Option<FaceMesh> = None;
    let mut err = String::new();
    let mut offer = |r: Result<FaceMesh, String>, best: &mut Option<FaceMesh>, err: &mut String| {
        match r {
            Ok(m) => {
                if best.as_ref().map_or(true, |b| score(&m) < score(b)) {
                    *best = Some(m);
                }
            }
            Err(e) => {
                if !err.is_empty() {
                    err.push_str("; ");
                }
                err.push_str(&e);
            }
        }
    };
    offer(first, &mut best, &mut err);
    if !best.as_ref().map_or(false, |m| good(m)) {
        let tries: [(bool, f64); 3] = [(false, 1e-6), (true, 1e-9), (true, 1e-6)];
        for (cover, rel) in tries {
            if cover && !periodic {
                continue;
            }
            let r = with_same_uv(rel, || mesh_face_impl(body, f, params, virt, ssi, cut, tol, cover));
            offer(r, &mut best, &mut err);
            if best.as_ref().map_or(false, |m| good(m)) {
                break;
            }
        }
    }
    match best {
        Some(m) => Ok(m),
        None => Err(err),
    }
}

thread_local! {
    static SAME_UV_REL: std::cell::Cell<f64> = const { std::cell::Cell::new(1e-9) };
}

fn mesh_face_strict(body: &Body, f: FaceId, params: &[Vec<f64>], virt: &[(bool, bool)], ssi: &[f64], cut: FaceCut, tol: &Tol) -> Result<FaceMesh, String> {
    match mesh_face_impl(body, f, params, virt, ssi, cut, tol, false) {
        Ok(m) if m.open && body.face(f).surface.periods() != (None, None) => {
            // an unclosed boundary (a rim edge cut by the window, its ends not meeting): the cut-open strip lost
            // part of the face. Retry on the universal cover and keep the retry only when its area is within 5
            // percent of the exact face area and closer to it than the strip's.
            let area = |m: &FaceMesh| -> f64 {
                m.tris.iter().map(|t| 0.5 * (m.pts[t[1] as usize] - m.pts[t[0] as usize]).cross(m.pts[t[2] as usize] - m.pts[t[0] as usize]).len()).sum()
            };
            let exact = ck_topo::mass::face_integral::<1>(body, f, 1, &|_, n| [n.len()])[0].abs();
            match mesh_face_impl(body, f, params, virt, ssi, cut, tol, true) {
                Ok(m2) if (area(&m2) - exact).abs() < 0.05 * exact && (area(&m2) - exact).abs() < (area(&m) - exact).abs() => Ok(m2),
                _ => Ok(m),
            }
        }
        Ok(m) => Ok(m),
        Err(e) => {
            let periodic = body.face(f).surface.periods() != (None, None);
            if periodic {
                mesh_face_impl(body, f, params, virt, ssi, cut, tol, true).map_err(|e2| format!("{e}; cover: {e2}"))
            } else {
                Err(e)
            }
        }
    }
}

#[allow(clippy::too_many_arguments)]
fn mesh_face_impl(body: &Body, f: FaceId, params: &[Vec<f64>], virt: &[(bool, bool)], ssi: &[f64], cut: FaceCut, tol: &Tol, cover: bool) -> Result<FaceMesh, String> {
    let face = body.face(f);
    crate::tr!(9, 0, 0, f.idx(), 0);
    let surf = &face.surface;
    let (pu, pv) = surf.periods();
    // The original opens a periodic direction at the window [cut - p, cut] (its uv bounds are exactly that window).
    // A cylinder wall bounded by the original's first SSI circle opens the other way: window [cut, cut + p].
    let ssi_hole = face.loops.iter().any(|&l| body.lp(l).coedges.iter().any(|&c| body.coedge(c).alive && ssi.get(body.coedge(c).edge.idx()).map_or(false, |&o| o != 0.0)));
    let wsh = if ssi_hole { 0.0 } else { 1.0 };
    // full spheres / tori are opened at their parameter domain [0, p] (oracle bounds)
    let full_dom = matches!(surf, Surface::Sphere { .. } | Surface::Torus { .. });
    let cut0 = cut;
    let mut cut = if cover {
        FaceCut::default()
    } else if full_dom {
        FaceCut { u: pu, v: pv }
    } else {
        cut
    };
    let (mut cu, mut cv) = if cover {
        (None, None)
    } else {
        (pu.map(|p| cut.u.unwrap_or(p) - p * wsh), pv.map(|p| cut.v.unwrap_or(p) - p * wsh))
    };

    let mut b = Builder { surf, pts: vec![], index: HashMap::default(), segs: vec![] };
    let eps_u = pu.map(|p| p * 1e-9).unwrap_or(0.0);
    let eps_v = pv.map(|p| p * 1e-9).unwrap_or(0.0);

    // ---- 1. boundary polylines (unwrapped along each loop)
    let mut v_lo = f64::INFINITY;
    let mut v_hi = f64::NEG_INFINITY;
    let mut u_lo = f64::INFINITY;
    let mut u_hi = f64::NEG_INFINITY;
    let mut polys: Vec<Vec<BPt>> = vec![];
    // v of every seam end (a seam that reaches a pole is the only boundary there)
    let mut seam_v: Vec<f64> = vec![];
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
            // (a B-spline patch closed on itself is an ordinary rectangle in uv: its seam edge
            // bounds the polygon on both sides, there is no cut window to open)
            let seam = body.is_seam(e) && !matches!(surf, Surface::BSpline(_) | Surface::Screw { .. });
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
                // A seam edge's pcurve copy is deliberate: the face's two sides of the seam sit
                // one period apart (a wrapping trim loop). Unwrapping it against the previous
                // point would collapse the pair onto one copy and lose the face's trim.
                if !seam {
                    if let (Some(per), Some(pr)) = (pu, prev) {
                        uv.x = ck_geom::pcurve::unwrap_near(uv.x, pr.x, per);
                    }
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
                pl.push(BPt { uv, p, virt: false });
            }
            if std::env::var("CK_DUMP3").is_ok() { eprintln!("COEDGE f{:?} c={:?} rev={} seam={} n={} first={:?} last={:?}", f, c, ce.rev, seam, pl.len(), pl.first().map(|q| q.uv), pl.last().map(|q| q.uv)); }
            // junctions of a merged circle chain are not vertices of the original's single edge: mark, drop later
            if let Some(&(v0, v1)) = virt.get(e.idx()) {
                let (sv, ev) = if ce.rev { (v1, v0) } else { (v0, v1) };
                if ev {
                    pl.last_mut().unwrap().virt = true;
                }
                if sv {
                    pl[0].virt = true;
                }
            }
            if seam && !cover {
                seam_v.extend([pl[0].uv.y, pl[pl.len() - 1].uv.y]);
                // seams are interior once the face is cut open; they only bound the v/u range
                poly.push(BPt { uv: Vec2::new(f64::NAN, f64::NAN), p: Vec3::ZERO, virt: false });
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
                // A gap that is a whole number of periods is not a failure on the cover: a face
                // whose trim wraps the seam (a sphere minus a patch: the seam edge is walked both
                // ways) closes in 3D one period over in uv, and its closing segment is the seam /
                // pole line, which `join` samples. Any other uv gap means the chain does not close.
                let winds = |d: f64, p: Option<f64>| match p {
                    Some(p) => ((d / p) - (d / p).round()).abs() <= 1e-9,
                    // a non-periodic direction cannot wind: a small gap is a tangency
                    // closure error (the pcurves meet at one vertex but their fits differ)
                    None => d.abs() <= 1e-3 * (1.0 + d.abs()),
                };
                let whole = winds(first.uv.x - last.uv.x, pu) && winds(first.uv.y - last.uv.y, pv);
                join(&mut poly, last, first, tol);
                if cover && !whole && first.uv.dist(poly.last().unwrap().uv) > 1e-6 && first.p.dist(poly.last().unwrap().p) <= 1e-6 * (1.0 + first.p.len()) {
                    return Err("loop winds a period".into());
                }
            }
        }
        // split at NaN markers into open chains; a loop without markers is closed
        polys.push(poly);
    }

    // A partial sphere / torus (a fillet band) whose unwrapped boundary spills out of the oracle's [0, p] window (a torus fillet
    // band spans v = 3pi/2 .. 2pi + a) cannot be mapped into it: the segments beyond the window edge would be shifted a
    // period down and the strip folds over itself (mesh area/volume wrong). Open such a direction at the face's own cut.
    if full_dom && !cover {
        let spill = |lo: f64, hi: f64, p: f64| lo < -1e-9 || hi > p + 1e-9;
        if let (Some(p), Some(c)) = (pu, cut0.u) {
            if spill(u_lo, u_hi, p) {
                cut.u = Some(c);
                cu = Some(c - p * wsh);
            }
        }
        if let (Some(p), Some(c)) = (pv, cut0.v) {
            if spill(v_lo, v_hi, p) {
                cut.v = Some(c);
                cv = Some(c - p * wsh);
            }
        }
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
    // drop the junction points of merged circle chains; a chain that began and ended on one closes over it
    //
    // A face without a period (a flange ring with a slit) is cut into open chains at the slit edges, which are dropped
    // as seams: a dropped junction at a chain end is where the chain on the other side of the slit attaches. Remember
    // those junctions, and connect the new chain ends over them once the points are dropped (the chord the neighbouring
    // faces, which drop the same junction, have too).
    let flat = pu.is_none() && pv.is_none();
    let mut junctions: HashMap<(u64, u64), Vec<(usize, bool)>> = HashMap::default();
    if flat {
        for (ci, (ch, closed)) in chains.iter().enumerate() {
            if !*closed && ch.len() >= 2 {
                if ch[0].virt {
                    junctions.entry((ch[0].uv.x.to_bits(), ch[0].uv.y.to_bits())).or_default().push((ci, true));
                }
                if ch[ch.len() - 1].virt {
                    junctions.entry((ch[ch.len() - 1].uv.x.to_bits(), ch[ch.len() - 1].uv.y.to_bits())).or_default().push((ci, false));
                }
            }
        }
    }
    for (ch, closed) in chains.iter_mut() {
        if ch.iter().any(|q| q.virt) {
            // (both ends are the same junction: the same point in space. A chain between two different junctions
            // of a slit face stays open: the chains on the far side of the slits close it.)
            let ends = !*closed && ch.len() >= 2 && ch[0].virt && ch[ch.len() - 1].virt && ch[0].p.dist(ch[ch.len() - 1].p) <= 1e-9 * (1.0 + ch[0].p.len());
            // a ring over a junction that sits on the cut line keeps the junction: the chord that would replace it
            // straddles the window edge and cannot be mapped into the strip
            let on_cut = |x: f64, c: Option<f64>, p: Option<f64>| match (c, p) {
                (Some(c), Some(p)) => ((x - c) / p - ((x - c) / p).round()).abs() <= 1e-9,
                _ => false,
            };
            let keep = if ends && (on_cut(ch[0].uv.x, cu, pu) || on_cut(ch[0].uv.y, cv, pv)) { Some(ch[0]) } else { None };
            // Drop a junction only where the boundary continues smoothly along one carrier arc (the
            // pieces of an arc split at the seam). A seam that is itself part of the trim loop turns at
            // the junction (a sphere face whose trim wraps the seam): dropping it leaves the seam's two
            // copies with different ends and the display mesh gets a hole.
            let n = ch.len();
            let mut keep_corner = vec![false; n];
            if *closed {
                for i in 0..n {
                    if !ch[i].virt {
                        continue;
                    }
                    let (a, b) = (ch[(i + n - 1) % n].uv, ch[(i + 1) % n].uv);
                    let (d1, d2) = (ch[i].uv - a, b - ch[i].uv);
                    keep_corner[i] = !(d1.len() > 1e-12 && d2.len() > 1e-12 && d1.dot(d2) > 0.9 * d1.len() * d2.len());
                }
            }
            let mut kept: Vec<BPt> = Vec::with_capacity(n);
            for (i, &q) in ch.iter().enumerate() {
                if q.virt && !keep_corner[i] {
                    continue;
                }
                let mut q = q;
                q.virt = false;
                kept.push(q);
            }
            *ch = kept;
            if let Some(mut j) = keep {
                j.virt = false;
                ch.insert(0, j);
            }
            if ends && ch.len() >= 3 {
                *closed = true;
            }
        }
    }
    for (_, group) in junctions {
        if let [(ci, si), (cj, sj)] = group[..] {
            if ci != cj && !chains[ci].1 && !chains[cj].1 && !chains[ci].0.is_empty() && !chains[cj].0.is_empty() {
                let end = |c: &(Vec<BPt>, bool), at_start: bool| if at_start { c.0[0] } else { c.0[c.0.len() - 1] };
                let (p, q) = (end(&chains[ci], si), end(&chains[cj], sj));
                if !same_uv(p.uv, q.uv) {
                    chains.push((vec![p, q], false));
                }
            }
        }
    }
    // A face whose outer loop (largest area) runs clockwise is walked backwards: every closed chain keeps its first vertex and
    // is reversed after it.
    {
        let area = |ch: &Vec<BPt>| -> f64 {
            let n = ch.len();
            (0..n).map(|i| ch[i].uv.x * ch[(i + 1) % n].uv.y - ch[(i + 1) % n].uv.x * ch[i].uv.y).sum::<f64>()
        };
        let outer = chains.iter().filter(|c| c.1 && c.0.len() > 2).map(|c| area(&c.0)).fold(0.0f64, |m, a| if a.abs() > m.abs() { a } else { m });
        if outer < 0.0 && !cover {
            for (ch, closed) in chains.iter_mut() {
                if *closed && ch.len() > 1 {
                    ch[1..].reverse();
                }
            }
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
    // A segment lying exactly on the window's upper edge `c + p` is that edge's boundary (not the lower edge a period
    // down) when the whole face sits below it: lo > c and hi == c + p (a partial torus band ending on its outer equator).
    let on_top = |ya: f64, yb: f64, lo: f64, hi: f64, c: f64, p: f64| -> bool {
        let eps = 1e-9 * (1.0 + p);
        let top = c + p;
        (ya - top).abs() <= eps && (yb - top).abs() <= eps && (hi - top).abs() <= eps && lo > c + eps
    };
    // The mirror case: a segment lying exactly on the window's lower edge `c` when the whole face sits below it (a torus
    // band ending on its outer equator, opened at a window above it) is that edge's boundary a period up.
    let on_floor = |ya: f64, yb: f64, lo: f64, hi: f64, c: f64, p: f64| -> bool {
        let eps = 1e-9 * (1.0 + p);
        (ya - c).abs() <= eps && (yb - c).abs() <= eps && (hi - c).abs() <= eps && lo < c - eps
    };
    if std::env::var("CK_DBG").map_or(false, |v| v == body.face_id_str(f)) {
        for (ch, closed) in &chains {
            let xs: Vec<f64> = ch.iter().map(|q| (q.uv.x * 1e4).round() / 1e4).collect();
            let n = xs.len();
            eprintln!("DBGCHAIN closed={} n={} head={:?} tail={:?} cu={:?} u_lo={} u_hi={}", closed, n, &xs[..n.min(4)], &xs[n.saturating_sub(4)..], cu, u_lo, u_hi);
        }
    }
    let mut seg_list: Vec<(BPt, BPt)> = vec![];
    for (chain, closed) in &chains {
        let n = chain.len();
        let m = if *closed { n } else { n - 1 };
        for i in 0..m {
            let (mut a, mut q) = (chain[i], chain[(i + 1) % n]);
            if *closed {
                // the closing segment of a ring: the first point is the period copy nearest the last one. A ring that was
                // walked backwards (first vertex kept, the rest reversed) has that jump at its first segment instead, so
                // any step of more than half a period is a wrap.
                if let Some(per) = pu {
                    if i + 1 == n || (q.uv.x - a.uv.x).abs() > 0.5 * per {
                        q.uv.x = ck_geom::pcurve::unwrap_near(q.uv.x, a.uv.x, per);
                    }
                }
                if let Some(per) = pv {
                    if i + 1 == n || (q.uv.y - a.uv.y).abs() > 0.5 * per {
                        q.uv.y = ck_geom::pcurve::unwrap_near(q.uv.y, a.uv.y, per);
                    }
                }
            }
            if a.uv.dist(q.uv) == 0.0 {
                continue;
            }
            if let (Some(c), Some(p)) = (cu, pu) {
                let mid = 0.5 * (a.uv.x + q.uv.x);
                if on_floor(a.uv.x, q.uv.x, u_lo, u_hi, c, p) {
                    a.uv.x += p;
                    q.uv.x += p;
                } else if !on_top(a.uv.x, q.uv.x, u_lo, u_hi, c, p) {
                    a.uv.x = shift_into(a.uv.x, mid, c, p);
                    q.uv.x = shift_into(q.uv.x, mid, c, p);
                }
            }
            if let (Some(c), Some(p)) = (cv, pv) {
                let mid = 0.5 * (a.uv.y + q.uv.y);
                if on_floor(a.uv.y, q.uv.y, v_lo, v_hi, c, p) {
                    a.uv.y += p;
                    q.uv.y += p;
                } else if !on_top(a.uv.y, q.uv.y, v_lo, v_hi, c, p) {
                    a.uv.y = shift_into(a.uv.y, mid, c, p);
                    q.uv.y = shift_into(q.uv.y, mid, c, p);
                }
            }
            a.uv.x = snap(a.uv.x, cu, pu, eps_u);
            q.uv.x = snap(q.uv.x, cu, pu, eps_u);
            a.uv.y = snap(a.uv.y, cv, pv, eps_v);
            q.uv.y = snap(q.uv.y, cv, pv, eps_v);
            seg_list.push((a, q));
        }
    }
    weld_dangling_ends(&mut seg_list, 1e-8);
    weld_close_points(&mut seg_list);
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
                // a seam ending at a pole (a hemisphere cap bounded by one circle): the pole is not a
                // boundary point, so the cut lines must reach it and the pole line closes the strip
                let mut poles: Vec<f64> = vec![];
                if pv.is_none() {
                    for &sv in &seam_v {
                        if surf.is_pole(c, sv) && !poles.iter().any(|&x| (x - sv).abs() < 1e-12) {
                            poles.push(sv);
                            on.push(sv);
                        }
                    }
                }
                on.sort_by(|x, y| x.total_cmp(y));
                on.dedup();
                for &pv0 in &poles {
                    iso(&mut b, Vec2::new(c, pv0), Vec2::new(c + p, pv0), Vec2::ZERO);
                }
                // The pole line is a boundary of the strip: the parity test (a ray towards -v) must count it, or a pole
                // below the loop (a sphere cap whose pole is at v = -pi/2) leaves the strip's side lines out.
                let mut with_poles = seg_list.clone();
                for &pv0 in &poles {
                    let pt = |x: f64| BPt { uv: Vec2::new(x, pv0), p: Vec3::ZERO, virt: false };
                    with_poles.push((pt(c), pt(c + p)));
                }
                for w in on.windows(2) {
                    let vm = 0.5 * (w[0] + w[1]);
                    if inside_parity(&with_poles, Vec2::new(c + 0.5 * p * 1e-6, vm)) {
                        iso(&mut b, Vec2::new(c, w[0]), Vec2::new(c, w[1]), Vec2::ZERO);
                        iso(&mut b, Vec2::new(c + p, w[0]), Vec2::new(c + p, w[1]), Vec2::new(p, 0.0));
                    }
                }
                // a torus fragment is cut open in v as well: boundary points on its v cut lines bound inside intervals there
                if let (Some(cv1), Some(qv1)) = (cv, pv) {
                    let mut onv: Vec<f64> = b.pts.iter().filter(|q| q.uv.y == cv1).map(|q| q.uv.x).collect();
                    onv.extend(b.pts.iter().filter(|q| q.uv.y == cv1 + qv1).map(|q| q.uv.x));
                    onv.sort_by(|x, y| x.total_cmp(y));
                    onv.dedup();
                    for w in onv.windows(2) {
                        let um = 0.5 * (w[0] + w[1]);
                        if inside_parity_v(&seg_list, Vec2::new(um, cv1 + 0.5 * qv1 * 1e-6)) {
                            iso(&mut b, Vec2::new(w[0], cv1), Vec2::new(w[1], cv1), Vec2::ZERO);
                            iso(&mut b, Vec2::new(w[0], cv1 + qv1), Vec2::new(w[1], cv1 + qv1), Vec2::new(0.0, qv1));
                        }
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
    // A cut-open periodic strip whose boundary is a single simple cycle: the original walks it counter-clockwise from the
    // corner with the smallest (u, v) (the reversed-sense rule: first vertex kept, the rest reversed).
    let full_u = pu.map_or(false, |p| u_hi - u_lo >= p * (1.0 - 1e-6));
    let full_v = pv.map_or(false, |p| v_hi - v_lo >= p * (1.0 - 1e-6));
    if (full_u || full_v) && !cover {
        b.canonical_ring(face.sense);
    }

    // ---- 4. CDT in a metric-scaled frame
    // uv bounds of the face (original f3578): the sample bbox grown by 1e-3 of its extent on each side; a periodic direction
    // is exactly its window.
    let infl = |lo: f64, hi: f64| -> (f64, f64) {
        let e = (hi - lo) * 1e-3;
        (lo - e, hi + e)
    };
    let ubnd = match (pu, cut.u) {
        (Some(p), Some(c)) => (c - p * wsh, c + p * (1.0 - wsh)),
        _ => infl(u_lo, u_hi),
    };
    let vbnd = match (pv, cut.v) {
        (Some(p), Some(c)) => (c - p * wsh, c + p * (1.0 - wsh)),
        _ => infl(v_lo, v_hi),
    };
    let cuv = Vec2::new((ubnd.0 + ubnd.1) * 0.5, (vbnd.0 + vbnd.1) * 0.5);
    crate::tr!(20, ubnd.0, vbnd.0, ubnd.1, vbnd.1);
    let (su, sv) = {
        let (su, sv) = metric_scale(surf, ubnd, vbnd);
        (if su.is_nan() { 1e-12 } else { su.max(1e-12) }, if sv.is_nan() { 1e-12 } else { sv.max(1e-12) })
    };
    let to_c = |uv: Vec2| -> Pt { [(uv.x - cuv.x) * su, (uv.y - cuv.y) * sv] };
    let from_c = |p: Pt| -> Vec2 { Vec2::new(p[0] / su + cuv.x, p[1] / sv + cuv.y) };
    let cpts: Vec<Pt> = b.pts.iter().map(|q| to_c(q.uv)).collect();
    let open = {
        let mut deg: HashMap<usize, u32> = HashMap::default();
        for &(a, c) in &b.segs {
            if a != c {
                *deg.entry(a).or_insert(0) += 1;
                *deg.entry(c).or_insert(0) += 1;
            }
        }
        if std::env::var("CK_DBG").map_or(false, |v| v == body.face_id_str(f)) {
            for (k, d) in &deg {
                if d % 2 == 1 {
                    eprintln!("DBGODD pt {} {:?} p={:?} deg {}", k, b.pts[*k].uv, b.pts[*k].p, d);
                }
            }
        }
        deg.values().any(|d| d % 2 == 1)
    };
    let mut tri = cdt::triangulate(&cpts, &b.segs)?;
    if std::env::var("CK_DBG").map_or(false, |v| v == "1" || v == body.face_id_str(f)) {
        eprintln!("DBG face {:?} {} cu={:?} cv={:?} npts={} nsegs={} inside={} failed={}", f, body.face_id_str(f), cu, cv, b.pts.len(), b.segs.len(), tri.cdt.inside_tris().len(), tri.cdt.failed);
        for (i, q) in b.pts.iter().enumerate().take(400) { eprintln!("DBGPT {} {:?} P {:.5} {:.5} {:.5}", i, q.uv, q.p.x, q.p.y, q.p.z); }
        eprintln!("DBGSEGS {:?}", b.segs);
    }
    #[cfg(not(target_arch = "wasm32"))]
    if tri.cdt.failed > 0 && std::env::var("CK_TESS_DBG").is_ok() {
        eprintln!("tessellate: face {} has {} unrecovered trim segments ({} points, {} segments)", body.face_id_str(f), tri.cdt.failed, b.pts.len(), b.segs.len());
    }
    // vertex data indexed by CDT vertex
    let mut vdata: HashMap<u32, (Vec2, Vec3)> = HashMap::default();
    // Boundary vertices keep the edge curve's 3D point (the neighbours share it), but an edge curve can lie
    // well off the surface (an approximated cut that grazes it: the uv of a near-tangent point is ill
    // conditioned). Refinement must judge the surface, not that offset, or it never converges there.
    let mut off_surf: HashMap<u32, Vec3> = HashMap::default();
    for (i, &v) in tri.vmap.iter().enumerate() {
        vdata.entry(v).or_insert((b.pts[i].uv, b.pts[i].p));
    }
    // Other surfaces only count a gross offset (an invalid body whose trim is far off its face): a triangle with such a
    // vertex never comes within the tolerance however often it is split, and refinement would spend its whole budget.
    let off_limit = if matches!(surf, Surface::BSpline(_) | Surface::Screw { .. }) { 0.25 * tol.chord } else { 100.0 * tol.chord };
    for (&v, &(uv, p)) in vdata.iter() {
        let sp = surf.eval(uv.x, uv.y);
        if sp.dist(p) > off_limit {
            off_surf.insert(v, sp);
        }
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

    // ---- 4b. lattice seeding of big smooth spline faces
    // Centroid insertion alone grows a long thin face by one triangle layer per pass (64 passes max, each
    // re-testing the whole mesh): a swept coil face came out half empty and 100k triangles heavy. Seed the
    // interior with the (u,v) lattice the surface needs along its iso-lines, then let refinement mop up.
    let lattice_n0 = match surf {
        Surface::BSpline(bs) => Some(((knot_spans(&bs.knots_u) * 4).clamp(64, 4096), (knot_spans(&bs.knots_v) * 4).clamp(64, 4096))),
        // a screw patch: the density of the rows its interpolated stand-in had (48 per turn, 24 along arcs)
        Surface::Screw { urange, .. } => Some((((48.0 * (urange.1 - urange.0) / std::f64::consts::TAU).ceil() as usize * 4).clamp(64, 4096), 96)),
        _ => None,
    };
    if let (Some(n0), None, None) = (lattice_n0, cu, cv) {
        if u_hi > u_lo && v_hi > v_lo && b.pts.len() >= 64 {
            seed_lattice(n0, surf, &mut tri, &mut vdata, (u_lo, u_hi), (v_lo, v_hi), (su, sv), tol, &to_c);
        }
    }

    // ---- 4c. surface-aware diagonals
    // The Delaunay triangulation of a long thin strip (a cylinder wall between two trimmed ends) fans one end's points to a few
    // points of the other end: its diagonals span half the circumference and fail the chord test, and the refinement below
    // then subdivides them into thousands of triangles. Flip an edge whose midpoint deviates from the surface by more than the
    // tolerance when the other diagonal of its quadrilateral deviates clearly less.
    if !matches!(surf, Surface::Plane { .. }) {
        let cpts = tri.cdt.pts.clone();
        let dev_of = |a: u32, b: u32| -> f64 {
            let at = |k: u32| -> (Vec2, Vec3) {
                match vdata.get(&k) {
                    Some(&d) => (d.0, off_surf.get(&k).copied().unwrap_or(d.1)),
                    None => {
                        let uv = from_c(cpts[k as usize]);
                        let e = wrap_eval(uv);
                        (uv, surf.eval(e.x, e.y))
                    }
                }
            };
            let (pa, pb) = (at(a), at(b));
            let w = wrap_eval((pa.0 + pb.0) * 0.5);
            surf.eval(w.x, w.y).dist((pa.1 + pb.1) * 0.5)
        };
        let mut want = |w: u32, u: u32, v: u32, x: u32| -> bool {
            let cur = dev_of(u, v);
            cur > tol.chord && dev_of(w, x) < 0.8 * cur
        };
        tri.cdt.optimize_edges(&mut want, 16);
    }

    // ---- 5. refinement
    let budget = 20000 + 50 * b.pts.len();
    let mut inserted = 0usize;
    let mut front: Vec<usize> = vec![];
    let sense = face.sense;
    let is_spline = matches!(surf, Surface::BSpline(_) | Surface::Screw { .. });
    // Triangles that passed keep their vertices (and so their verdict) across
    // passes: remember them instead of re-evaluating the surface 4x each pass.
    let mut accepted: HashSet<[u32; 3]> = HashSet::default();
    for _pass in 0..64 {
        let mut cands: Vec<(Pt, u32)> = vec![];
        for (ti, v) in tri.cdt.inside_tris() {
            let ids = [v[0], v[1], v[2]];
            let key = {
                let mut k = ids;
                k.sort_unstable();
                k
            };
            if accepted.contains(&key) {
                continue;
            }
            let get = |k: u32| -> (Vec2, Vec3) {
                match vdata.get(&k) {
                    Some(&d) => (d.0, off_surf.get(&k).copied().unwrap_or(d.1)),
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
            let cuv3 = (a.0 + bb.0 + c.0) * (1.0 / 3.0);
            let flat3 = !(area2 > 1e-24 * (1.0 + a.1.len2()));
            if flat3 && !is_spline {
                accepted.insert(key);
                continue;
            }
            // A triangle flat in 3D but not in uv (it spans a closing seam, e.g. (0,v0) (2pi,v0) (0,v1) on a
            // tube) has no plane to measure depth against: it must still be refined, else the face keeps a
            // hole there. Judge it by the distance of the surface at the centroid to the 3D centroid.
            let nn = if flat3 { Vec3::ZERO } else { n / area2 };
            let e = wrap_eval(cuv3);
            let sp = surf.eval(e.x, e.y);
            // Pinned (sphere A/B): the oracle's interior acceptance test is the
            // surface depth at the triangle centroid alone — it saturates at
            // exactly the chord tolerance (0.01384 vs tol 0.01386 on R=10
            // sphere) and does NOT bound the centroid normal angle (36.9deg
            // observed > 17.2deg angle tol) or the edge-midpoint chords.
            let mut dev = if flat3 { sp.dist((a.1 + bb.1 + c.1) * (1.0 / 3.0)) } else { (sp - a.1).dot(nn).abs() };
            for (x, y) in [(a, bb), (bb, c), (c, a)] {
                let w = wrap_eval((x.0 + y.0) * 0.5);
                let sm = surf.eval(w.x, w.y);
                dev = dev.max(sm.dist((x.1 + y.1) * 0.5));
            }
            if dev > tol.chord {
                let cp = to_c(cuv3);
                cands.push((cp, ti));
            } else {
                accepted.insert(key);
            }
        }
        if cands.is_empty() {
            break;
        }
        // Every pass halves the triangle size along a front that is resolving, and a feature a thousandth of the part is
        // resolved in about ten passes, so after fourteen the front must be shrinking. One that is no smaller than it was six
        // passes ago is not converging: its triangles have a vertex the
        // surface cannot reach (a trim of a failed boolean), and splitting them again changes nothing. Stop here instead
        // of spending the whole budget.
        front.push(cands.len());
        if front.len() > 14 && cands.len() >= front[front.len() - 7] {
            break;
        }
        let mut any = false;
        // Candidates come out in triangle-id order, which is spatially random:
        // on a large face the locate walk from the last insertion is O(sqrt n)
        // per point. Start it from the candidate's own triangle instead (the
        // result does not depend on the start except on exact ties, so small
        // faces keep the original's walk).
        let hinted = cands.len() > 4000;
        for (p, hint) in cands {
            if inserted >= budget {
                break;
            }
            if hinted && (hint as usize) < tri.cdt.tris.len() {
                tri.cdt.hint = hint;
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
    let mut out = FaceMesh { face: Some(f), win: [cu.zip(pu), cv.zip(pv)], open, unrecovered: tri.cdt.failed, ..Default::default() };
    let mut local: HashMap<u32, u32> = HashMap::default();
    // The parity flood can leak across a boundary segment: triangles bridge a hole or spill outside the trims (a sliver
    // across a bolt hole of a flange face). On a plane the trim polygon is exact in uv, so drop every triangle whose
    // centroid the segments say is outside.
    let strict_plane = matches!(surf, Surface::Plane { .. }) && !seg_list.is_empty() && tri.cdt.tris.len() * seg_list.len() <= 50_000_000;
    if std::env::var("CK_DBG_TRI").map_or(false, |v| v == body.face_id_str(f)) {
        let at: Vec<f64> = std::env::var("CK_DBG_AT").unwrap_or_default().split(',').filter_map(|x| x.parse().ok()).collect();
        let near = |uv: Vec2| at.len() == 3 && (uv.x - at[0]).abs() < at[2] && (uv.y - at[1]).abs() < at[2];
        for (ti, v) in tri.cdt.all_tris_dbg() {
            let uvs: Vec<Vec2> = (0..3).map(|k| from_c(tri.cdt.pts[v[k] as usize])).collect();
            if uvs.iter().any(|&u| near(u)) {
                eprintln!("DBGTRI {} inside={} {:?} ids {:?}", ti, tri.cdt.inside_tris().iter().any(|x| x.0 == ti), uvs.iter().map(|u| ((u.x * 1e5).round() / 1e5, (u.y * 1e5).round() / 1e5)).collect::<Vec<_>>(), v);
            }
        }
    }
    // snap lookups: a boundary sample and the refinement point beside it can land within a hair
    // of each other (the refitted pcurve puts a straight side chain a few ulps off its
    // isoparametric line), and the wire only carries f32, so two points the f32 grid cannot tell
    // apart give a triangle with no area and a meaningless facet normal. Join each new vertex to
    // an already-emitted one within `snap_tol`, looked up for every vertex as it is emitted: every
    // triangle mentioning either of a near pair then gets the same id, which keeps the mesh closed
    // (merging only where the two co-occur in one triangle leaves the other edges unmatched).
    let snap_tol: f64 = std::env::var("CK_SNAP_TOL").ok().and_then(|v| v.parse().ok()).unwrap_or(1e-4);
    // ...and only when the parameters agree: two copies of one point either side of a periodic
    // cut (u and u + period) are coincident in 3D but joining them leaves edges whose uv midpoint
    // lands a half period away, which the shading correction then measures as a huge normal step.
    let snap_uv = 1e-3;
    let cell = snap_tol.max(1e-12);
    let mut cells: HashMap<(i64, i64, i64), Vec<u32>> = HashMap::default();
    let key = |p: Vec3| -> (i64, i64, i64) { ((p.x / cell).floor() as i64, (p.y / cell).floor() as i64, (p.z / cell).floor() as i64) };
    for (_, v) in tri.cdt.inside_tris() {
        if strict_plane {
            let c = (0..3).fold(Vec2::ZERO, |acc, k| acc + from_c(tri.cdt.pts[v[k] as usize]) * (1.0 / 3.0));
            if !inside_parity(&seg_list, c) {
                continue;
            }
        }
        let mut ids = [0u32; 3];
        for k in 0..3 {
            let cv = v[k];
            ids[k] = match local.get(&cv) {
                Some(&id) => id,
                None => {
                    let (uv, p) = match vdata.get(&cv) {
                        Some(&d) => d,
                        None => {
                            let uv = from_c(tri.cdt.pts[cv as usize]);
                            let e = wrap_eval(uv);
                            (uv, surf.eval(e.x, e.y))
                        }
                    };
                    let k0 = key(p);
                    let mut joined = None;
                    'near: for dx in -1i64..=1 {
                        for dy in -1i64..=1 {
                            for dz in -1i64..=1 {
                                if let Some(here) = cells.get(&(k0.0 + dx, k0.1 + dy, k0.2 + dz)) {
                                    for &id in here {
                                        let u = out.uv[id as usize];
                                        if (out.pts[id as usize] - p).len() < snap_tol
                                            && (u.x - uv.x).abs() <= snap_uv
                                            && (u.y - uv.y).abs() <= snap_uv
                                        {
                                            joined = Some(id);
                                            break 'near;
                                        }
                                    }
                                }
                            }
                        }
                    }
                    let id = match joined {
                        Some(id) => id,
                        None => {
                            let e = wrap_eval(uv);
                            out.uv.push(uv);
                            out.pts.push(p);
                            out.nrm.push(FaceMesh::outward(surf, sense, e));
                            let id = (out.pts.len() - 1) as u32;
                            cells.entry(key(p)).or_default().push(id);
                            id
                        }
                    };
                    local.insert(cv, id);
                    id
                }
            };
        }
        if ids[0] == ids[1] || ids[1] == ids[2] || ids[0] == ids[2] {
            continue;
        }
        let (pa, pb, pc) = (out.pts[ids[0] as usize], out.pts[ids[1] as usize], out.pts[ids[2] as usize]);
        if (pb - pa).cross(pc - pa).len2() <= 0.0 {
            continue; // degenerate (pole)
        }
        out.tris.push(if sense { ids } else { [ids[0], ids[2], ids[1]] });
    }
    if out.tris.is_empty() {
        // The CDT classified nothing as inside: a sliver face (a crescent a tenth of a millimetre wide between a circle
        // and an intersection curve that touch at both ends) whose boundary polylines cross by a few 1e-6. A single simple
        // cycle of a plane is still a polygon: clip its ears.
        {
            if let Some(ring) = single_cycle(&b.segs) {
                let uv: Vec<Vec2> = ring.iter().map(|&i| b.pts[i].uv).collect();
                let clip = ear_clip(&uv);
                // the ring of a sliver whose polylines cross is not a simple polygon: its ears can fill the wrong region
                // (a whole disc for a crescent). Keep the clip only when it covers the exact face area.
                let area: f64 = clip.iter().map(|t| 0.5 * (b.pts[ring[t[1]]].p - b.pts[ring[t[0]]].p).cross(b.pts[ring[t[2]]].p - b.pts[ring[t[0]]].p).len()).sum();
                let exact = ck_topo::mass::face_integral::<1>(body, f, 1, &|_, n| [n.len()])[0].abs();
                if exact > 0.0 && area >= 0.8 * exact && area <= 1.25 * exact {
                    for t in clip {
                        let mut ids = [0u32; 3];
                        for k in 0..3 {
                            let q = &b.pts[ring[t[k]]];
                            out.uv.push(q.uv);
                            out.pts.push(q.p);
                            out.nrm.push(FaceMesh::outward(surf, sense, q.uv));
                            ids[k] = (out.pts.len() - 1) as u32;
                        }
                        out.tris.push(if sense { ids } else { [ids[0], ids[2], ids[1]] });
                    }
                }
            }
        }
        if out.tris.is_empty() {
            return Err("face produced no triangles".into());
        }
    }
    Ok(out)
}

/// The vertex order of `segs` when they form exactly one simple closed cycle (every vertex used twice), else `None`.
fn single_cycle(segs: &[(usize, usize)]) -> Option<Vec<usize>> {
    let mut adj: HashMap<usize, Vec<usize>> = HashMap::default();
    for &(a, c) in segs {
        if a == c {
            continue;
        }
        adj.entry(a).or_default().push(c);
        adj.entry(c).or_default().push(a);
    }
    if adj.len() < 3 || adj.values().any(|v| v.len() != 2) {
        return None;
    }
    let start = *adj.keys().min()?;
    let mut ring = vec![start];
    let (mut prev, mut cur) = (start, adj[&start][0]);
    while cur != start {
        ring.push(cur);
        let n = &adj[&cur];
        let next = if n[0] != prev { n[0] } else { n[1] };
        prev = cur;
        cur = next;
        if ring.len() > adj.len() {
            return None;
        }
    }
    (ring.len() == adj.len()).then_some(ring)
}

/// Ear clipping of the polygon `p` (either orientation); triangles come out counter-clockwise as index triples into `p`.
/// An ear that another vertex blocks is clipped anyway when nothing else is left, so a sliver always gets covered.
fn ear_clip(p: &[Vec2]) -> Vec<[usize; 3]> {
    let cross = |a: Vec2, b: Vec2, c: Vec2| (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
    let area2: f64 = (0..p.len()).map(|i| cross(Vec2::ZERO, p[i], p[(i + 1) % p.len()])).sum();
    let mut idx: Vec<usize> = (0..p.len()).collect();
    if area2 < 0.0 {
        idx.reverse();
    }
    let mut out = vec![];
    while idx.len() > 3 {
        let n = idx.len();
        let ear = |strict: bool| -> Option<usize> {
            (0..n).find(|&i| {
                let (a, b, c) = (p[idx[(i + n - 1) % n]], p[idx[i]], p[idx[(i + 1) % n]]);
                if cross(a, b, c) <= 0.0 {
                    return false;
                }
                !strict
                    || !idx.iter().any(|&j| {
                        let q = p[j];
                        q != a && q != b && q != c && cross(a, b, q) >= 0.0 && cross(b, c, q) >= 0.0 && cross(c, a, q) >= 0.0
                    })
            })
        };
        let i = ear(true).or_else(|| ear(false)).unwrap_or(0);
        out.push([idx[(i + n - 1) % n], idx[i], idx[(i + 1) % n]]);
        idx.remove(i);
    }
    if idx.len() == 3 {
        out.push([idx[0], idx[1], idx[2]]);
    }
    out.retain(|t| cross(p[t[0]], p[t[1]], p[t[2]]).abs() > 0.0);
    out
}

/// Per-axis scale of the centred CDT frame (original f3573, with the cone special case of f3325):
/// plane (1,1), cylinder (r,1), cone (max radius over the v range, 1/cos), sphere (r,r), torus (R+r,r), else |Su|,|Sv| at the domain centre.
fn metric_scale(surf: &Surface, ur: (f64, f64), vr: (f64, f64)) -> (f64, f64) {
    match surf {
        Surface::Plane { .. } => (1.0, 1.0),
        Surface::Cylinder { r, .. } => (*r, 1.0),
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
        Surface::Sphere { .. } | Surface::Torus { .. } if std::env::var("X").is_ok() => (1.0, 1.0),
        Surface::Sphere { r, .. } => (*r, *r),
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

/// Open chains that should meet (the two halves of a hole straddling a periodic seam) can end a few ulps apart because each
/// edge evaluates its own pcurve; the CDT keys points by exact bits, so the halves would stay unjoined and the hole would
/// be filled. Snap every dangling segment end (degree one in uv) onto another dangling end within `eps`.
fn weld_dangling_ends(segs: &mut [(BPt, BPt)], eps: f64) {
    let key = |v: Vec2| (v.x.to_bits(), v.y.to_bits());
    let mut deg: HashMap<(u64, u64), u32> = HashMap::default();
    for (a, q) in segs.iter() {
        *deg.entry(key(a.uv)).or_insert(0) += 1;
        *deg.entry(key(q.uv)).or_insert(0) += 1;
    }
    let mut dang: Vec<Vec2> = vec![];
    for (a, q) in segs.iter() {
        for v in [a.uv, q.uv] {
            if deg[&key(v)] == 1 {
                dang.push(v);
            }
        }
    }
    if dang.len() < 2 {
        return;
    }
    let mut snap: HashMap<(u64, u64), Vec2> = HashMap::default();
    for i in 0..dang.len() {
        if snap.contains_key(&key(dang[i])) {
            continue;
        }
        for j in (i + 1)..dang.len() {
            if !snap.contains_key(&key(dang[j])) && (dang[i].x - dang[j].x).abs() <= eps && (dang[i].y - dang[j].y).abs() <= eps && key(dang[i]) != key(dang[j]) {
                snap.insert(key(dang[j]), dang[i]);
                break;
            }
        }
    }
    if snap.is_empty() {
        return;
    }
    for (a, q) in segs.iter_mut() {
        if let Some(&v) = snap.get(&key(a.uv)) {
            a.uv = v;
        }
        if let Some(&v) = snap.get(&key(q.uv)) {
            q.uv = v;
        }
    }
}

/// Snaps trim points that differ by a few ulps (the same vertex reached through two coedges, or two samples of a closing
/// curve) onto one point, so no zero-length constraint or sub-ulp sliver reaches the CDT.
fn weld_close_points(segs: &mut [(BPt, BPt)]) {
    let mut pts: Vec<(f64, f64)> = Vec::with_capacity(segs.len() * 2);
    for (a, q) in segs.iter() {
        pts.push((a.uv.x, a.uv.y));
        pts.push((q.uv.x, q.uv.y));
    }
    pts.sort_by(|a, b| a.0.total_cmp(&b.0));
    let tol = |p: (f64, f64)| 4e-15 * (1.0 + p.0.abs() + p.1.abs());
    let mut snap: HashMap<(u64, u64), Vec2> = HashMap::default();
    for i in 0..pts.len() {
        let key_i = (pts[i].0.to_bits(), pts[i].1.to_bits());
        if snap.contains_key(&key_i) {
            continue;
        }
        let mut j = i + 1;
        while j < pts.len() && pts[j].0 - pts[i].0 <= tol(pts[i]) {
            let key_j = (pts[j].0.to_bits(), pts[j].1.to_bits());
            if key_j != key_i && (pts[j].1 - pts[i].1).abs() <= tol(pts[i]) && !snap.contains_key(&key_j) {
                snap.insert(key_j, Vec2::new(pts[i].0, pts[i].1));
            }
            j += 1;
        }
    }
    if snap.is_empty() {
        return;
    }
    for (a, q) in segs.iter_mut() {
        if let Some(&v) = snap.get(&(a.uv.x.to_bits(), a.uv.y.to_bits())) {
            a.uv = v;
        }
        if let Some(&v) = snap.get(&(q.uv.x.to_bits(), q.uv.y.to_bits())) {
            q.uv = v;
        }
    }
}

fn same_uv(a: Vec2, b: Vec2) -> bool {
    a.dist(b) <= SAME_UV_REL.with(|c| c.get()) * (1.0 + a.x.abs() + a.y.abs())
}

/// Connects the end `a` of one coedge to the start `b` of the next: a pole
/// (same 3D point, different uv) is sampled along uv with the angle step.
fn join(poly: &mut Vec<BPt>, a: BPt, b: BPt, tol: &Tol) {
    if same_uv(a.uv, b.uv) || a.p.dist(b.p) > 1e-9 * (1.0 + b.p.len()) {
        return;
    }
    let n = (a.uv.dist(b.uv) / tol.angle).ceil().max(1.0) as usize;
    for i in 1..n {
        poly.push(BPt { uv: a.uv.lerp(b.uv, i as f64 / n as f64), p: b.p, virt: false });
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

/// Distinct knot spans of a clamped knot vector.
fn knot_spans(k: &[f64]) -> usize {
    let mut n = 0;
    for w in k.windows(2) {
        if w[1] > w[0] {
            n += 1;
        }
    }
    n
}

/// Interior parameter fractions in (0,1) for the lattice lines of one direction. Along each of a few section
/// iso-lines the polyline is sampled at `m` uniform steps; every step needs `max(len * sqrt(k / 8 chord), turn / angle)`
/// nodes (k = turning angle per length), the sections' needs are maxed, and the nodes are spread to equalise the
/// cumulative need (continuous, unlike bisection which jumps by powers of two).
fn lattice_fractions(surf: &Surface, rng: (f64, f64), other: (f64, f64), along_u: bool, tol: &Tol, m: usize) -> Vec<f64> {
    let mut need = vec![0.0f64; m];
    for f in [0.1, 0.3, 0.5, 0.7, 0.9] {
        let o = other.0 + (other.1 - other.0) * f;
        let pts: Vec<Vec3> = (0..=m)
            .map(|k| {
                let t = rng.0 + (rng.1 - rng.0) * k as f64 / m as f64;
                if along_u { surf.eval(t, o) } else { surf.eval(o, t) }
            })
            .collect();
        let len: Vec<f64> = pts.windows(2).map(|w| w[0].dist(w[1])).collect();
        // turning angle at interior sample k (between interval k-1 and k)
        let turn = |k: usize| -> f64 {
            if len[k - 1] > 1e-12 && len[k] > 1e-12 {
                curvature::normal_angle((pts[k] - pts[k - 1]) / len[k - 1], (pts[k + 1] - pts[k]) / len[k])
            } else {
                0.0
            }
        };
        let th: Vec<f64> = (0..=m).map(|k| if k == 0 || k == m { 0.0 } else { turn(k) }).collect();
        for k in 0..m {
            // the larger turn at either end of the interval, spread over the interval
            let t = th[k].max(th[k + 1]);
            let l = len[k].max(1e-300);
            let kappa = t / l;
            // A micro-kink (a spline pulled through two points 1e-4 apart) turns by an angle inside one sampling step of a
            // length far below the chord tolerance: `t / angle` nodes there are a fan of slivers a few 1e-4 wide that no
            // display can show, and that a welded copy of the mesh collapses unevenly (open edges). No node closer than a
            // tenth of the display chord tolerance (the lattice tolerance is a third of it).
            let n = (l * (kappa / (8.0 * tol.chord)).sqrt()).max(t / tol.angle).min(l / (0.3 * tol.chord));
            if n > need[k] {
                need[k] = n;
            }
        }
    }
    let total: f64 = need.iter().sum();
    let nlines = total.ceil() as usize;
    if nlines < 2 || nlines > 100_000 {
        return vec![];
    }
    let mut out = Vec::with_capacity(nlines);
    let (mut acc, mut j) = (0.0, 1usize);
    for k in 0..m {
        let nk = need[k];
        while j < nlines && acc + nk >= total * j as f64 / nlines as f64 && nk > 0.0 {
            let want = total * j as f64 / nlines as f64;
            out.push((k as f64 + (want - acc) / nk) / m as f64);
            j += 1;
        }
        acc += nk;
    }
    out.retain(|&s| s > 0.0 && s < 1.0);
    out
}

#[allow(clippy::too_many_arguments)]
fn seed_lattice(
    n0: (usize, usize),
    surf: &Surface,
    tri: &mut cdt::Triangulation,
    vdata: &mut HashMap<u32, (Vec2, Vec3)>,
    ur: (f64, f64),
    vr: (f64, f64),
    sc: (f64, f64),
    tol: &Tol,
    to_c: &dyn Fn(Vec2) -> Pt,
) {
    let (n0u, n0v) = n0;
    // the two iso-line sags add up on a cell diagonal: sample each direction to a third of the chord tolerance
    let tol = &Tol { chord: tol.chord / 3.0, angle: tol.angle };
    let fu = lattice_fractions(surf, ur, vr, true, tol, n0u);
    let fv = lattice_fractions(surf, vr, ur, false, tol, n0v);
    if fu.is_empty() || fv.is_empty() || fu.len().saturating_mul(fv.len()) > 400_000 {
        return;
    }
    let us: Vec<f64> = fu.iter().map(|s| ur.0 + (ur.1 - ur.0) * s).collect();
    let vs: Vec<f64> = fv.iter().map(|s| vr.0 + (vr.1 - vr.0) * s).collect();
    let gap = |x: &[f64], i: usize, lo: f64, hi: f64| {
        let a = if i == 0 { lo } else { x[i - 1] };
        let b = if i + 1 == x.len() { hi } else { x[i + 1] };
        (x[i] - a).min(b - x[i])
    };
    for i in 0..us.len() {
        let hu = gap(&us, i, ur.0, ur.1) * sc.0;
        for j in 0..vs.len() {
            let hv = gap(&vs, j, vr.0, vr.1) * sc.1;
            let uv = Vec2::new(us[i], vs[j]);
            let p = to_c(uv);
            let cdt::Loc::In(t) = tri.cdt.locate(p) else { continue };
            if tri.cdt.tris[t as usize].region != cdt::INSIDE {
                continue;
            }
            let h2 = (0.3 * hu.min(hv)).powi(2);
            let near = tri.cdt.tris[t as usize].v.iter().any(|&k| {
                let q = tri.cdt.pts[k as usize];
                (q[0] - p[0]).powi(2) + (q[1] - p[1]).powi(2) < h2
            });
            if near {
                continue;
            }
            let v = tri.cdt.insert_in_tri(t, p);
            vdata.insert(v, (uv, surf.eval(uv.x, uv.y)));
        }
    }
}
