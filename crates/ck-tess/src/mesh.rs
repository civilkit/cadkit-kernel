//! Body display mesh, as the wire `body.mesh` / `body.silhouette` expect it.
//!
//! Wire layout (pinned against oracle buffers of box / cylinder / sphere /
//! torus / revolve probes, decoded with tools/meshdump.mjs):
//! - `positions` f32, 9 per triangle (UNWELDED: 3 vertices × xyz).
//! - `normals`   f32, 9 per triangle: exact outward surface normal at each
//!   vertex (planes: the face normal).
//! - `mids`      f32, 9 per triangle: for the triangle edges (v0v1, v1v2,
//!   v2v0) the correction `n(mid) − (n_a + n_b)/2`, where n(mid) is the unit
//!   surface normal at the edge's (u,v) midpoint. The frontend shader uses
//!   `normalize(n + 4(M0·b0b1 + M1·b1b2 + M2·b2b0))` (app.pretty.js line
//!   3648). Pinned: cylinder r=1 → (0.003627035766839981, 0, −0.00174668…),
//!   planes → 0.
//! - `triFace`   i32, 1 per triangle: index into `body.faces[]`.
//! - `silhouette` f32, 12 per candidate edge: `[a xyz, b xyz, nA xyz, nB xyz]`
//!   for every mesh edge shared by two triangles of the SAME face (welded by
//!   position) whose dihedral is strictly convex; `a→b` runs in the later
//!   triangle's direction, `nA` = facet normal of the earlier triangle, `nB`
//!   of the later. Pinned: cylinder = one record per ruling (36 for r=1),
//!   a cylindrical HOLE has none (concave), planes none, sphere 11841 of
//!   13293 edges (all convex ones). GUESS: cross-face smooth edges excluded.
//! - `box` = bounds of the f32 positions (oracle boxes are f32-rounded).
use crate::curvature::{self, Tol, MIN_CHORD};
use crate::face::{self, FaceCut, FaceMesh};
use ck_geom::{Vec2, Vec3};
use ck_topo::body::{Body, EdgeId, FaceId};
use ck_core::fxhash::FxHashMap as HashMap;
use std::cell::RefCell;
use std::collections::BTreeMap;
use std::rc::Rc;

/// Shared edge samples: `params[e]` = sorted parameter list of edge `e`.
#[derive(Debug, Clone, Default)]
pub struct EdgeSampling {
    pub params: Vec<Vec<f64>>,
    /// Per edge `(start, end)`: that end is a seam-only junction inside a merged circle chain. The original has ONE edge there, so the
    /// junction vertex is not a sample of the polyline (wire `pts`) nor a vertex of the face polygons.
    pub virt: Vec<(bool, bool)>,
    /// Per edge: u offset of its pcurve on the cylinder in the original. Closed SSI circles (negative start parameter) alternate
    /// -2pi / +2pi in edge order (pinned on the cutHole probe: both give the original's cut). HEURISTIC.
    pub ssi: Vec<f64>,
    /// Periodic cut of every face (indexed by face id).
    pub cuts: Vec<FaceCut>,
}

/// Display mesh with typed arrays ready for `$buf` emission.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Mesh {
    pub positions: Vec<f32>,
    pub normals: Vec<f32>,
    pub mids: Vec<f32>,
    pub tri_face: Vec<i32>,
    pub silhouette: Vec<f32>,
    /// Wire face indices whose triangulation failed (not in the arrays).
    pub failed: Vec<usize>,
    /// Why each entry of `failed` failed (same order; empty text when the failure came out of the face cache).
    pub failed_why: Vec<String>,
    /// Faces whose triangulation is suspect: `(wire face index, text)` (unrecovered trim segments).
    pub notes: Vec<(usize, String)>,
}

impl Mesh {
    pub fn num_tris(&self) -> usize {
        self.tri_face.len()
    }
    /// Bounds of the f32 positions as f64 (wire `box`). None for an empty mesh.
    pub fn bbox(&self) -> Option<(Vec3, Vec3)> {
        if self.positions.is_empty() {
            return None;
        }
        let mut lo = [f64::INFINITY; 3];
        let mut hi = [f64::NEG_INFINITY; 3];
        for (i, &x) in self.positions.iter().enumerate() {
            let k = i % 3;
            lo[k] = lo[k].min(x as f64);
            hi[k] = hi[k].max(x as f64);
        }
        Some((Vec3::new(lo[0], lo[1], lo[2]), Vec3::new(hi[0], hi[1], hi[2])))
    }
}

/// Full tessellation: edge samples (for `body.edges[].pts`) + the mesh.
#[derive(Debug, Clone, Default)]
pub struct Tess {
    pub tol: Option<Tol>,
    pub edges: EdgeSampling,
    pub mesh: Mesh,
    /// Per wire face, the face triangulation (None when it failed).
    pub faces: Vec<Option<Rc<FaceMesh>>>,
}

impl Tess {
    /// Polyline of edge `e` (endpoints = its vertices).
    pub fn edge_polyline(&self, body: &Body, e: EdgeId) -> Vec<Vec3> {
        polyline(body, e, &self.edges.params[e.idx()], self.edges.virt[e.idx()])
    }
}

fn polyline(body: &Body, e: EdgeId, ps: &[f64], virt: (bool, bool)) -> Vec<Vec3> {
    let ed = body.edge(e);
    let (a, b) = (body.vertex(ed.v0).p, body.vertex(ed.v1).p);
    if ps.len() < 2 {
        return vec![a, b];
    }
    let n = ps.len();
    let mut pl: Vec<Vec3> = ps
        .iter()
        .enumerate()
        .map(|(i, &t)| if i == 0 { a } else if i + 1 == n { b } else { ed.curve.eval(t) })
        .collect();
    if virt.1 {
        // the next chain member's first point is skipped by the wire concatenation, so drop the junction here
        pl.pop();
    }
    pl
}

/// Regen tolerance of a body (chord from its bbox, angle 0.3).
pub fn body_tol(body: &Body) -> Tol {
    let (lo, hi) = body.bbox();
    let mut t = Tol::for_bbox(lo, hi);
    // The box of a body with curved faces moves in the last bits when an unrelated feature changes, and with it every
    // sample (ties between equal gaps in the periodic cuts are decided by rounding noise): round the chord DOWN to four
    // significant figures so that unchanged faces mesh bit-identically (and the face cache can find them).
    if t.chord.is_finite() && t.chord > 0.0 {
        let q = 10f64.powi(t.chord.log10().floor() as i32 - 3);
        t.chord = ((t.chord / q).floor() * q).max(MIN_CHORD.min(t.chord));
    }
    t
}

/// Samples every live edge: curve params, per-coedge uv refinement, per-face
/// normal refinement, then the periodic-face cut params (see curvature.rs).
pub fn sample_edges(body: &Body, tol: &Tol) -> EdgeSampling {
    let mut params: Vec<Vec<f64>> = vec![vec![]; body.edges.len()];
    let mut virt: Vec<(bool, bool)> = vec![(false, false); body.edges.len()];
    // Circle arcs chained through seam-only vertices are ONE oracle edge: one sample grid over the whole run.
    let mut base: Vec<Option<Vec<f64>>> = vec![None; body.edges.len()];
    for chain in body.wire_chains() {
        if chain.len() < 2 || !chain.iter().all(|&e| matches!(body.edge(e).curve, ck_geom::curve::Curve::Circle { .. })) {
            continue;
        }
        let first = body.edge(chain[0]);
        let p0 = first.t0;
        let total: f64 = chain.iter().map(|&e| body.edge(e).t1 - body.edge(e).t0).sum();
        let grid = curvature::curve_params(&first.curve, p0, p0 + total, tol);
        let span = total.abs().max(1e-300);
        let mut a = p0; // chain parameter where the member starts
        for (k, &e) in chain.iter().enumerate() {
            let ed = body.edge(e);
            let s = ed.t1 - ed.t0;
            let b = a + s;
            let (lo, hi) = (a.min(b), a.max(b));
            let eps = 1e-12 * span;
            let mut ps = vec![ed.t0];
            let mut on_end = false;
            for &g in &grid {
                if g > lo + eps && g < hi - eps {
                    ps.push(g - a + ed.t0);
                } else if k + 1 < chain.len() && (g - b).abs() <= eps {
                    on_end = true;
                }
            }
            ps.push(ed.t1);
            if s < 0.0 {
                // members run against increasing parameter: keep the list ascending in traversal order
                ps.sort_by(|x, y| y.total_cmp(x));
            }
            base[e.idx()] = Some(ps);
            if k + 1 < chain.len() && !on_end {
                virt[e.idx()].1 = true;
                virt[chain[k + 1].idx()].0 = true;
            }
            a = b;
        }
    }
    let mut ssi = vec![0.0f64; body.edges.len()];
    {
        let mut k = 0usize;
        for e in body.edge_ids() {
            let ed = body.edge(e);
            if ed.t0 < 0.0 && matches!(ed.curve, ck_geom::curve::Curve::Circle { .. }) && !body.is_seam(e) {
                ssi[e.idx()] = if k % 2 == 0 { -std::f64::consts::TAU } else { std::f64::consts::TAU };
                k += 1;
            }
        }
    }
    for e in body.edge_ids() {
        let ed = body.edge(e);
        let mut ps = base[e.idx()].take().unwrap_or_else(|| curvature::curve_params(&ed.curve, ed.t0, ed.t1, tol));
        let cos = body.edge_coedges(e);
        // uv chord refinement, coedge by coedge
        for &c in &cos {
            if let Some(pc) = &body.coedge(c).pcurve {
                let surf = &body.face(body.coedge_face(c)).surface;
                let uvt = curvature::uv_tol(surf, pc.eval(ps[0]), tol);
                ps = curvature::refine_uv(pc, &ps, uvt);
            }
        }
        // normal-angle refinement, coedge by coedge
        for &c in &cos {
            let f = body.coedge_face(c);
            let surf = &body.face(f).surface;
            let uv_at = |t: f64| body.coedge_uv(c, t, None);
            ps = curvature::refine_normals(&uv_at, surf, &ps, tol.angle);
        }
        params[e.idx()] = ps;
    }
    // periodic cuts (computed from the refined samples of every face, then merged)
    let mut cuts = vec![FaceCut::default(); body.faces.len()];
    let mut extra: Vec<Vec<f64>> = vec![vec![]; body.edges.len()];
    for f in body.face_ids() {
        let surf = &body.face(f).surface;
        let (pu, pv) = surf.periods();
        if pu.is_none() && pv.is_none() {
            continue;
        }
        let mut us = vec![];
        let mut vs = vec![];
        let mut cos = vec![];
        for &l in &body.face(f).loops {
            for &c in &body.lp(l).coedges {
                if !body.coedge(c).alive {
                    continue;
                }
                cos.push(c);
                let e = body.coedge(c).edge;
                let (v0, v1) = virt[e.idx()];
                let pe = &params[e.idx()];
                for (k, &t) in pe.iter().enumerate() {
                    if (k == 0 && v0) || (k + 1 == pe.len() && v1) {
                        continue; // junction of a merged chain: not a sample of the original's single edge
                    }
                    let uv = body.coedge_uv(c, t, None);
                        let sh = ssi[e.idx()];
                    us.push(uv.x + sh);
                    vs.push(uv.y);
                }
            }
        }
        let mut cut = FaceCut::default();
        if let Some(p) = pu {
            cut.u = Some(curvature::periodic_cut(&us, p).unwrap_or(0.0));
        }
        if let Some(p) = pv {
            cut.v = Some(curvature::periodic_cut(&vs, p).unwrap_or(0.0));
        }
        cuts[f.idx()] = cut;
        // crossing params of every non-seam coedge
        for &c in &cos {
            let e = body.coedge(c).edge;
            if body.is_seam(e) {
                continue;
            }
            let ps = &params[e.idx()];
            for (dir, per, cv) in [(0usize, pu, cut.u), (1usize, pv, cut.v)] {
                let (Some(p), Some(cv)) = (per, cv) else { continue };
                for t in crossings(body, c, ps, dir, cv, p, &ssi) {
                    extra[e.idx()].push(t);
                }
            }
        }
    }
    for (e, ex) in extra.into_iter().enumerate() {
        if ex.is_empty() {
            continue;
        }
        let ps = &mut params[e];
        ps.extend(ex);
        ps.sort_by(|a, b| a.total_cmp(b));
        let span = (ps[ps.len() - 1] - ps[0]).abs().max(1e-300);
        ps.dedup_by(|a, b| (*a - *b).abs() <= 1e-12 * span);
    }
    EdgeSampling { params, virt, ssi, cuts }
}

/// Params in the open sample range where coordinate `dir` of the coedge's
/// (u,v) equals `cut + k·p`. Lines solved exactly, other pcurves by bisection.
fn crossings(body: &Body, c: ck_topo::body::CoedgeId, ps: &[f64], dir: usize, cut: f64, p: f64, ssi: &[f64]) -> Vec<f64> {
    let mut out = vec![];
    if ps.len() < 2 {
        return out;
    }
    let comp = |v: Vec2| if dir == 0 { v.x } else { v.y };
    let (lo, hi) = (ps[0].min(ps[ps.len() - 1]), ps[0].max(ps[ps.len() - 1]));
    let tiny = 1e-12 * (hi - lo).abs().max(1e-300);
    if let Some(ck_geom::curve2::Curve2::Line { o, d }) = &body.coedge(c).pcurve {
        let (mut o, d) = (comp(*o), comp(*d));
        if dir == 0 && ssi[body.coedge(c).edge.idx()] < 0.0 {
            o += ssi[body.coedge(c).edge.idx()]; // the first SSI circle of a pair sits a period lower in the original
        }
        if d == 0.0 {
            return out;
        }
        // u(t) = o + d t ; u in [u(lo), u(hi)]
        let (ua, ub) = ((o + d * lo).min(o + d * hi), (o + d * lo).max(o + d * hi));
        let k0 = ((ua - cut) / p).floor() as i64;
        let k1 = ((ub - cut) / p).ceil() as i64;
        for k in k0..=k1 {
            let target = cut + k as f64 * p;
            let t = (target - o) / d;
            if t > lo + tiny && t < hi - tiny {
                out.push(t);
            }
        }
        return out;
    }
    // general: scan the (unwrapped) samples
    let mut prev = body.coedge_uv(c, ps[0], None);
    for w in ps.windows(2) {
        let q = body.coedge_uv(c, w[1], Some(prev));
        let (a, b) = (comp(prev), comp(q));
        let (mn, mx) = (a.min(b), a.max(b));
        let k = ((mx - cut) / p).floor();
        let target = cut + k * p;
        if target > mn && target < mx {
            let (mut t0, mut t1) = (w[0], w[1]);
            let f0 = a - target;
            for _ in 0..60 {
                let tm = 0.5 * (t0 + t1);
                let fm = comp(body.coedge_uv(c, tm, Some(prev))) - target;
                if (fm < 0.0) == (f0 < 0.0) {
                    t0 = tm;
                } else {
                    t1 = tm;
                }
            }
            let t = 0.5 * (t0 + t1);
            if t > lo + tiny && t < hi - tiny {
                out.push(t);
            }
        }
        prev = q;
    }
    out
}

/// Polyline of one edge (runs the whole edge-sampling stage; use [`tessellate`]
/// + [`Tess::edge_polyline`] when exporting many edges).
pub fn edge_polyline(body: &Body, e: EdgeId) -> Vec<Vec3> {
    let tol = body_tol(body);
    let s = sample_edges(body, &tol);
    polyline(body, e, &s.params[e.idx()], s.virt[e.idx()])
}

/// Tessellates the body with the regen tolerance.
pub fn tessellate(body: &Body) -> Tess {
    tessellate_with(body, &body_tol(body))
}

/// Tessellates the body with an explicit tolerance.
pub fn tessellate_with(body: &Body, tol: &Tol) -> Tess {
    tessellate_impl(body, tol, false)
}

/// Like [`tessellate`], but every face that was meshed before with identical inputs (surface, trim loops, edge samples,
/// tolerance) comes out of the per-thread face cache ([`face_cache_stats`]) instead of being meshed again. The mesh is
/// bit-identical to the uncached one.
pub fn tessellate_memo(body: &Body) -> Tess {
    tessellate_impl(body, &body_tol(body), true)
}

fn tessellate_impl(body: &Body, tol: &Tol, memo: bool) -> Tess {
    let edges = sample_edges(body, tol);
    let mut outs: Vec<Option<Rc<FaceOut>>> = vec![];
    let mut why: HashMap<usize, String> = HashMap::default();
    for f in body.face_ids() {
        let key = if memo { Some(face_key(body, f, &edges, tol)) } else { None };
        if let Some(k) = &key {
            if let Some(hit) = FACE_CACHE.with(|c| c.borrow_mut().get(k)) {
                outs.push(hit);
                continue;
            }
        }
        let r = face::mesh_face(body, f, &edges.params, &edges.virt, &edges.ssi, edges.cuts[f.idx()], tol);
        #[cfg(not(target_arch = "wasm32"))]
        if let (Err(e), true) = (&r, std::env::var("CK_TESS_DBG").is_ok()) {
            eprintln!("tessellate: face {} dropped: {e}", body.face_id_str(f));
        }
        if let Err(e) = &r {
            why.insert(outs.len(), e.clone());
        }
        let out = r.ok().map(|fm| Rc::new(assemble_face(body, f, fm)));
        if let Some(k) = key {
            FACE_CACHE.with(|c| c.borrow_mut().put(k, out.clone()));
        }
        outs.push(out);
    }
    let mut mesh = assemble(&outs);
    for (wi, o) in outs.iter().enumerate() {
        if let Some(o) = o {
            if o.mesh.open {
                mesh.notes.push((wi, "boundary graph has an unclosed loop (odd-degree vertex)".to_string()));
            }
            if o.mesh.unrecovered > 0 {
                mesh.notes.push((wi, format!("{} trim segments not recovered", o.mesh.unrecovered)));
            }
        }
    }
    mesh.failed_why = mesh.failed.iter().map(|i| why.get(i).cloned().unwrap_or_default()).collect();
    let faces = outs.iter().map(|o| o.as_ref().map(|o| o.mesh.clone())).collect();
    Tess { tol: Some(*tol), edges, mesh, faces }
}

/// `body_mesh(&Body, tol)`: the wire mesh only.
pub fn body_mesh(body: &Body, tol: &Tol) -> Mesh {
    tessellate_with(body, tol).mesh
}

fn push3(v: &mut Vec<f32>, p: Vec3) {
    v.push(p.x as f32);
    v.push(p.y as f32);
    v.push(p.z as f32);
}

/// One face's contribution to the display mesh (everything but the wire face index): the triangulation and its
/// f32 buffers. Depends only on the face's own geometry, so it is what the face cache stores.
#[derive(Debug)]
pub struct FaceOut {
    pub mesh: Rc<FaceMesh>,
    pub positions: Vec<f32>,
    pub normals: Vec<f32>,
    pub mids: Vec<f32>,
    pub silhouette: Vec<f32>,
}

impl FaceOut {
    pub fn num_tris(&self) -> usize {
        self.mesh.tris.len()
    }
    fn bytes(&self) -> usize {
        let fm = &self.mesh;
        4 * (self.positions.len() + self.normals.len() + self.mids.len() + self.silhouette.len())
            + fm.uv.len() * 16
            + fm.pts.len() * 24
            + fm.nrm.len() * 24
            + fm.tris.len() * 12
            + 256
    }
}

fn assemble(outs: &[Option<Rc<FaceOut>>]) -> Mesh {
    let mut m = Mesh::default();
    let n: usize = outs.iter().flatten().map(|o| o.num_tris()).sum();
    m.positions.reserve(9 * n);
    m.normals.reserve(9 * n);
    m.mids.reserve(9 * n);
    m.tri_face.reserve(n);
    for (wi, o) in outs.iter().enumerate() {
        let Some(o) = o else {
            m.failed.push(wi);
            continue;
        };
        m.positions.extend_from_slice(&o.positions);
        m.normals.extend_from_slice(&o.normals);
        m.mids.extend_from_slice(&o.mids);
        m.silhouette.extend_from_slice(&o.silhouette);
        m.tri_face.resize(m.tri_face.len() + o.num_tris(), wi as i32);
    }
    m
}

fn assemble_face(body: &Body, f: FaceId, fm: FaceMesh) -> FaceOut {
    let mut m = Mesh::default();
    let face = body.face(f);
    let surf = &face.surface;
    let planar = matches!(surf, ck_geom::surface::Surface::Plane { .. });
    let cut = |uv: Vec2| -> Vec2 {
        // the same wrap the vertex normals use: the far copy of a strip point evaluates at the near edge
        let mut e = uv;
        if let Some((c, p)) = fm.win[0] {
            if e.x >= c + p {
                e.x -= p;
            }
        }
        if let Some((c, p)) = fm.win[1] {
            if e.y >= c + p {
                e.y -= p;
            }
        }
        e
    };
    let mut fnorm: Vec<Vec3> = Vec::with_capacity(fm.tris.len());
    for t in &fm.tris {
        let (a, b, c) = (t[0] as usize, t[1] as usize, t[2] as usize);
        for &k in &[a, b, c] {
            push3(&mut m.positions, fm.pts[k]);
            push3(&mut m.normals, fm.nrm[k]);
        }
        for &(i, j) in &[(a, b), (b, c), (c, a)] {
            let mid = if planar {
                Vec3::ZERO
            } else {
                let uv = cut((fm.uv[i] + fm.uv[j]) * 0.5);
                let nm = FaceMesh::outward(surf, face.sense, uv);
                nm - (fm.nrm[i] + fm.nrm[j]) * 0.5
            };
            push3(&mut m.mids, mid);
        }
        // facet normals come from the f32 display positions (pinned: oracle silhouette normals)
        let r = |p: Vec3| Vec3::new(p.x as f32 as f64, p.y as f32 as f64, p.z as f32 as f64);
        fnorm.push((r(fm.pts[b]) - r(fm.pts[a])).cross(r(fm.pts[c]) - r(fm.pts[a])).normalize());
    }
    if !planar {
        // silhouette candidates: same-face edges welded by position, strictly convex
        let key = |p: Vec3| (p.x.to_bits(), p.y.to_bits(), p.z.to_bits());
        let mut seen: HashMap<((u64, u64, u64), (u64, u64, u64)), usize> = HashMap::default();
        for (ti, t) in fm.tris.iter().enumerate() {
            for k in 0..3 {
                let (ia, ib) = (t[k] as usize, t[(k + 1) % 3] as usize);
                let (pa, pb) = (fm.pts[ia], fm.pts[ib]);
                let (ka, kb) = (key(pa), key(pb));
                if ka == kb {
                    continue;
                }
                // the earlier triangle stored this edge in its own (opposite) direction
                if let Some(&tj) = seen.get(&(kb, ka)) {
                    let (n1, n2) = (fnorm[tj], fnorm[ti]);
                    let opp = fm.pts[t[(k + 2) % 3] as usize];
                    let a0 = fm.pts[fm.tris[tj][0] as usize];
                    let s = n1.dot(opp - a0);
                    if s < 0.0 && n1 != n2 {
                        push3(&mut m.silhouette, pa);
                        push3(&mut m.silhouette, pb);
                        push3(&mut m.silhouette, n1);
                        push3(&mut m.silhouette, n2);
                    }
                } else {
                    seen.insert((ka, kb), ti);
                }
            }
        }
    }
    FaceOut { mesh: Rc::new(fm), positions: m.positions, normals: m.normals, mids: m.mids, silhouette: m.silhouette }
}

// ------------------------------------------------------------------------------------------------ face cache

/// Everything `mesh_face` reads for face `f`, as the exact bits of its inputs: equal keys mean an identical triangulation.
/// Every variable-length part is prefixed with its length, so two different inputs never give the same words.
type FaceKey = Rc<[u64]>;

fn key_f64s(out: &mut Vec<u64>, v: &[f64]) {
    out.push(v.len() as u64);
    out.extend(v.iter().map(|x| x.to_bits()));
}

/// Exact content of a surface (unlike `Surface::push_key`, which stands a B-spline net in by its hash).
fn key_surface(out: &mut Vec<u64>, s: &ck_geom::surface::Surface) {
    use ck_geom::surface::Surface;
    match s {
        Surface::BSpline(b) => {
            out.extend([18, b.deg_u as u64, b.deg_v as u64, b.nu as u64, b.nv as u64]);
            key_f64s(out, &b.knots_u);
            key_f64s(out, &b.knots_v);
            out.push(b.ctrl.len() as u64);
            for c in &b.ctrl {
                out.extend([c.x.to_bits(), c.y.to_bits(), c.z.to_bits()]);
            }
            match &b.weights {
                None => out.push(0),
                Some(w) => {
                    out.push(1);
                    key_f64s(out, w);
                }
            }
        }
        Surface::Offset { base, dist } => {
            out.push(20);
            key_surface(out, base);
            out.push(dist.to_bits());
        }
        other => other.push_key(out),
    }
}

fn key_vec2(out: &mut Vec<u64>, p: Vec2) {
    out.extend([p.x.to_bits(), p.y.to_bits()]);
}

fn key_curve2(out: &mut Vec<u64>, c: &Option<ck_geom::curve2::Curve2>) {
    use ck_geom::curve2::Curve2;
    match c {
        None => out.push(0),
        Some(Curve2::Line { o, d }) => {
            out.push(1);
            key_vec2(out, *o);
            key_vec2(out, *d);
        }
        Some(Curve2::Circle { c, x, r, ccw }) => {
            out.push(2);
            key_vec2(out, *c);
            key_vec2(out, *x);
            out.extend([r.to_bits(), *ccw as u64]);
        }
        Some(Curve2::Ellipse { c, x, a, b, ccw }) => {
            out.push(3);
            key_vec2(out, *c);
            key_vec2(out, *x);
            out.extend([a.to_bits(), b.to_bits(), *ccw as u64]);
        }
        Some(Curve2::BSpline(b)) => {
            out.extend([4, b.degree as u64]);
            key_f64s(out, &b.knots);
            out.push(b.ctrl.len() as u64);
            for p in &b.ctrl {
                key_vec2(out, *p);
            }
            match &b.weights {
                None => out.push(0),
                Some(w) => {
                    out.push(1);
                    key_f64s(out, w);
                }
            }
        }
    }
}

fn face_key(body: &Body, f: FaceId, edges: &EdgeSampling, tol: &Tol) -> FaceKey {
    let mut k: Vec<u64> = Vec::with_capacity(256);
    let face = body.face(f);
    key_surface(&mut k, &face.surface);
    let cuts = edges.cuts[f.idx()];
    let cut = |c: Option<f64>| [c.is_some() as u64, c.map_or(0, f64::to_bits)];
    k.push(face.sense as u64);
    k.extend(cut(cuts.u));
    k.extend(cut(cuts.v));
    k.extend([tol.chord.to_bits(), tol.angle.to_bits()]);
    for &l in &face.loops {
        let lp = body.lp(l);
        if !lp.alive {
            continue;
        }
        k.push(u64::MAX);
        for &c in &lp.coedges {
            let ce = body.coedge(c);
            if !ce.alive {
                continue;
            }
            let e = ce.edge;
            let ed = body.edge(e);
            k.extend([ce.rev as u64 | (body.is_seam(e) as u64) << 1, edges.virt[e.idx()].0 as u64 | (edges.virt[e.idx()].1 as u64) << 1]);
            ed.curve.push_key(&mut k);
            key_curve2(&mut k, &ce.pcurve);
            k.extend([ed.t0.to_bits(), ed.t1.to_bits(), edges.ssi[e.idx()].to_bits()]);
            key_f64s(&mut k, &edges.params[e.idx()]);
            for p in [body.vertex(body.coedge_start(c)).p, body.vertex(body.coedge_end(c)).p] {
                k.extend([p.x.to_bits(), p.y.to_bits(), p.z.to_bits()]);
            }
        }
    }
    k.into()
}

/// Byte cap of the per-thread face cache.
pub const FACE_CACHE_CAP: usize = 128 << 20;

#[derive(Default, Clone, Copy, Debug)]
pub struct FaceCacheStats {
    pub lookups: u64,
    pub hits: u64,
    pub entries: usize,
    pub bytes: usize,
    pub evicted: u64,
}

struct FaceCache {
    map: HashMap<FaceKey, (Option<Rc<FaceOut>>, u64)>,
    /// last_used tick -> key, oldest first
    lru: BTreeMap<u64, FaceKey>,
    tick: u64,
    bytes: usize,
    cap: usize,
    stats: FaceCacheStats,
}

impl Default for FaceCache {
    fn default() -> Self {
        FaceCache { map: HashMap::default(), lru: BTreeMap::new(), tick: 0, bytes: 0, cap: FACE_CACHE_CAP, stats: FaceCacheStats::default() }
    }
}

impl FaceCache {
    fn get(&mut self, k: &FaceKey) -> Option<Option<Rc<FaceOut>>> {
        self.stats.lookups += 1;
        self.tick += 1;
        let t = self.tick;
        let e = self.map.get_mut(k)?;
        self.lru.remove(&e.1);
        e.1 = t;
        self.lru.insert(t, k.clone());
        self.stats.hits += 1;
        Some(e.0.clone())
    }
    fn put(&mut self, k: FaceKey, v: Option<Rc<FaceOut>>) {
        if self.map.contains_key(&k) {
            return;
        }
        self.tick += 1;
        self.bytes += k.len() * 8 + v.as_ref().map_or(64, |o| o.bytes());
        self.lru.insert(self.tick, k.clone());
        self.map.insert(k, (v, self.tick));
        while self.bytes > self.cap && self.map.len() > 1 {
            let Some((&t, _)) = self.lru.iter().next() else { break };
            let key = self.lru.remove(&t).unwrap();
            if let Some((o, _)) = self.map.remove(&key) {
                self.bytes -= key.len() * 8 + o.as_ref().map_or(64, |o| o.bytes());
                self.stats.evicted += 1;
            }
        }
    }
}

thread_local! {
    static FACE_CACHE: RefCell<FaceCache> = RefCell::new(FaceCache::default());
}

pub fn face_cache_stats() -> FaceCacheStats {
    FACE_CACHE.with(|c| {
        let c = c.borrow();
        FaceCacheStats { entries: c.map.len(), bytes: c.bytes, ..c.stats }
    })
}

pub fn set_face_cache_cap(cap: usize) {
    FACE_CACHE.with(|c| c.borrow_mut().cap = cap);
}

pub fn clear_face_cache() {
    FACE_CACHE.with(|c| {
        let mut c = c.borrow_mut();
        c.map.clear();
        c.lru.clear();
        c.bytes = 0;
    });
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn empty_mesh_bbox() {
        assert!(Mesh::default().bbox().is_none());
    }
}
