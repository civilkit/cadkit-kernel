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
use crate::curvature::{self, Tol};
use crate::face::{self, FaceCut, FaceMesh};
use ck_geom::{Vec2, Vec3};
use ck_topo::body::{Body, EdgeId, FaceId};
use std::collections::HashMap;

/// Shared edge samples: `params[e]` = sorted parameter list of edge `e`.
#[derive(Debug, Clone, Default)]
pub struct EdgeSampling {
    pub params: Vec<Vec<f64>>,
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
    pub faces: Vec<Option<FaceMesh>>,
}

impl Tess {
    /// Polyline of edge `e` (endpoints = its vertices).
    pub fn edge_polyline(&self, body: &Body, e: EdgeId) -> Vec<Vec3> {
        polyline(body, e, &self.edges.params[e.idx()])
    }
}

fn polyline(body: &Body, e: EdgeId, ps: &[f64]) -> Vec<Vec3> {
    let ed = body.edge(e);
    let (a, b) = (body.vertex(ed.v0).p, body.vertex(ed.v1).p);
    if ps.len() < 2 {
        return vec![a, b];
    }
    let n = ps.len();
    ps.iter()
        .enumerate()
        .map(|(i, &t)| if i == 0 { a } else if i + 1 == n { b } else { ed.curve.eval(t) })
        .collect()
}

/// Regen tolerance of a body (chord from its bbox, angle 0.3).
pub fn body_tol(body: &Body) -> Tol {
    let (lo, hi) = body.bbox();
    Tol::for_bbox(lo, hi)
}

/// Samples every live edge: curve params, per-coedge uv refinement, per-face
/// normal refinement, then the periodic-face cut params (see curvature.rs).
pub fn sample_edges(body: &Body, tol: &Tol) -> EdgeSampling {
    let mut params: Vec<Vec<f64>> = vec![vec![]; body.edges.len()];
    for e in body.edge_ids() {
        let ed = body.edge(e);
        let mut ps = curvature::curve_params(&ed.curve, ed.t0, ed.t1, tol);
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
                for &t in &params[e.idx()] {
                    let uv = body.coedge_uv(c, t, None);
                    us.push(uv.x);
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
                for t in crossings(body, c, ps, dir, cv, p) {
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
    EdgeSampling { params, cuts }
}

/// Params in the open sample range where coordinate `dir` of the coedge's
/// (u,v) equals `cut + k·p`. Lines solved exactly, other pcurves by bisection.
fn crossings(body: &Body, c: ck_topo::body::CoedgeId, ps: &[f64], dir: usize, cut: f64, p: f64) -> Vec<f64> {
    let mut out = vec![];
    if ps.len() < 2 {
        return out;
    }
    let comp = |v: Vec2| if dir == 0 { v.x } else { v.y };
    let (lo, hi) = (ps[0].min(ps[ps.len() - 1]), ps[0].max(ps[ps.len() - 1]));
    let tiny = 1e-12 * (hi - lo).abs().max(1e-300);
    if let Some(ck_geom::curve2::Curve2::Line { o, d }) = &body.coedge(c).pcurve {
        let (o, d) = (comp(*o), comp(*d));
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
    polyline(body, e, &s.params[e.idx()])
}

/// Tessellates the body with the regen tolerance.
pub fn tessellate(body: &Body) -> Tess {
    tessellate_with(body, &body_tol(body))
}

/// Tessellates the body with an explicit tolerance.
pub fn tessellate_with(body: &Body, tol: &Tol) -> Tess {
    let edges = sample_edges(body, tol);
    let mut faces = vec![];
    for f in body.face_ids() {
        faces.push(face::mesh_face(body, f, &edges.params, edges.cuts[f.idx()], tol).ok());
    }
    let mesh = assemble(body, &faces);
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

fn assemble(body: &Body, faces: &[Option<FaceMesh>]) -> Mesh {
    let mut m = Mesh::default();
    let fids: Vec<FaceId> = body.face_ids().collect();
    for (wi, fm) in faces.iter().enumerate() {
        let Some(fm) = fm else {
            m.failed.push(wi);
            continue;
        };
        let face = body.face(fids[wi]);
        let surf = &face.surface;
        let planar = matches!(surf, ck_geom::surface::Surface::Plane { .. });
        let (pu, pv) = surf.periods();
        let cut = |uv: Vec2| -> Vec2 {
            // evaluation parameter of a strip point (wrap the far copy back)
            let mut e = uv;
            if let Some(p) = pu {
                if e.x.is_finite() {
                    e.x = e.x.rem_euclid(p);
                }
            }
            if let Some(p) = pv {
                if e.y.is_finite() {
                    e.y = e.y.rem_euclid(p);
                }
            }
            e
        };
        let base_tri = m.tri_face.len();
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
            m.tri_face.push(wi as i32);
            fnorm.push((fm.pts[b] - fm.pts[a]).cross(fm.pts[c] - fm.pts[a]).normalize());
        }
        let _ = base_tri;
        if planar {
            continue;
        }
        // silhouette candidates: same-face edges welded by position, strictly convex
        let key = |p: Vec3| (p.x.to_bits(), p.y.to_bits(), p.z.to_bits());
        let mut seen: HashMap<((u64, u64, u64), (u64, u64, u64)), usize> = HashMap::new();
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
    m
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn empty_mesh_bbox() {
        assert!(Mesh::default().bbox().is_none());
    }
}
