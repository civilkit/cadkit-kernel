//! Imprint (ck-ops/src/boolean/imprint.rs): intersect every face of body A
//! with every face of body B, trim the intersection curves to both faces,
//! and build a shared edge arrangement:
//! - one welded vertex pool for both bodies (tolerance `tol`),
//! - original edges split at every pool point lying on them,
//! - cut segments (trimmed SSI curves) split the same way,
//! - coincident pieces de-duplicated (a cut lying on an existing edge, or
//!   coplanar boundary edges of A and B, become ONE arrangement edge).
//! Each face then lists its boundary pieces (in original loop order) and
//! the cut edges strictly inside it ([`ArrFace`]). Fragment loops are
//! walked in assemble.rs.
//!
//! Coincident face pairs (same plane / same analytic carrier) imprint each
//! other's boundary edges instead of an SSI curve.

use super::classify::{point_in_face, Where};
use crate::bvh::{face_box, Aabb, Bvh};
use crate::intersect::ss::{self, Bounds, SsResult};
use crate::intersect::{cc, fit_param, param_slack};
use ck_geom::curve::Curve;
use ck_geom::surface::Surface;
use ck_geom::Vec3;
use ck_topo::body::{Body, EdgeId, FaceId, FaceName};

/// Arrangement edge.
#[derive(Debug, Clone, PartialEq)]
pub struct ArrEdge {
    pub curve: Curve,
    pub t0: f64,
    pub t1: f64,
    pub v0: usize,
    pub v1: usize,
    /// (side, original edge) for pieces of an existing edge (first source
    /// wins when A and B pieces coincide).
    pub src: Option<(usize, EdgeId)>,
}

/// A face of A (side 0) or B (side 1) with its arrangement pieces.
#[derive(Debug, Clone, PartialEq)]
pub struct ArrFace {
    pub side: usize,
    pub face: FaceId,
    pub surface: Surface,
    pub sense: bool,
    pub name: FaceName,
    /// Per original loop: boundary pieces in traversal order (edge, rev).
    pub bound: Vec<Vec<(usize, bool)>>,
    /// Cut edges strictly inside the face (both directions are used).
    pub inner: Vec<usize>,
    /// Original coedge count (to detect untouched faces).
    pub n_coedges: usize,
}

impl ArrFace {
    pub fn touched(&self) -> bool {
        !self.inner.is_empty() || self.bound.iter().map(|l| l.len()).sum::<usize>() != self.n_coedges
    }
}

#[derive(Debug, Clone, Default, PartialEq)]
pub struct Arrangement {
    pub verts: Vec<Vec3>,
    pub edges: Vec<ArrEdge>,
    pub faces: Vec<ArrFace>,
    pub tol: f64,
    /// Number of face pairs that produced cut segments or coincidences.
    pub interacting_pairs: usize,
}

/// A trimmed intersection curve before splitting.
#[derive(Debug, Clone)]
struct CutSeg {
    curve: Curve,
    t0: f64,
    t1: f64,
    /// faces (side index into `faces` of `Arrangement` via (side, FaceId))
    faces: Vec<(usize, FaceId)>,
}

impl Arrangement {
    /// Weld `p` into the pool.
    fn vertex(&mut self, p: Vec3) -> usize {
        let tol = self.tol;
        if let Some(i) = self.verts.iter().position(|q| q.dist(p) <= tol) {
            return i;
        }
        self.verts.push(p);
        self.verts.len() - 1
    }

    /// Add an edge piece or return the coincident existing one. Returns
    /// (edge index, same direction as given).
    fn edge(&mut self, curve: &Curve, t0: f64, t1: f64, v0: usize, v1: usize, src: Option<(usize, EdgeId)>) -> (usize, bool) {
        let tol = self.tol;
        for (i, e) in self.edges.iter().enumerate() {
            let same_ends = (e.v0 == v0 && e.v1 == v1) || (e.v0 == v1 && e.v1 == v0);
            if !same_ends {
                continue;
            }
            // samples of the new piece on the existing one
            let on = [0.31, 0.5, 0.73].iter().all(|&f| {
                let p = curve.eval(t0 + (t1 - t0) * f);
                dist_in_range(&e.curve, (e.t0, e.t1), p).1 <= tol * 10.0
            });
            if !on {
                continue;
            }
            let same = if v0 != v1 {
                e.v0 == v0
            } else {
                // closed: compare tangents
                let a = curve.deriv(t0 + (t1 - t0) * 0.25);
                let p = curve.eval(t0 + (t1 - t0) * 0.25);
                let (t, _) = dist_in_range(&e.curve, (e.t0, e.t1), p);
                a.dot(e.curve.deriv(t)) > 0.0
            };
            return (i, same);
        }
        self.edges.push(ArrEdge { curve: curve.clone(), t0, t1, v0, v1, src });
        (self.edges.len() - 1, true)
    }

    pub fn face_index(&self, side: usize, f: FaceId) -> Option<usize> {
        self.faces.iter().position(|af| af.side == side && af.face == f)
    }
}

/// Closest parameter (within range, periodic-aware) and distance.
pub fn dist_in_range(c: &Curve, r: (f64, f64), p: Vec3) -> (f64, f64) {
    let t = c.closest_param(p);
    if let Some(tt) = fit_param(c, t, r, 0.0) {
        return (tt, c.eval(tt).dist(p));
    }
    let (d0, d1) = (c.eval(r.0).dist(p), c.eval(r.1).dist(p));
    if d0 <= d1 {
        (r.0, d0)
    } else {
        (r.1, d1)
    }
}

/// Build the arrangement of `a` (side 0) and `b` (side 1).
pub fn imprint(a: &Body, b: &Body, tol: f64) -> Arrangement {
    let bodies = [a, b];
    let mut arr = Arrangement { tol, ..Default::default() };
    // ---- 1. face pairs and cut segments
    let fa: Vec<FaceId> = a.face_ids().collect();
    let fb: Vec<FaceId> = b.face_ids().collect();
    let boxes_a: Vec<Aabb> = fa.iter().map(|&f| face_box(a, f).padded(tol * 10.0)).collect();
    let boxes_b: Vec<Aabb> = fb.iter().map(|&f| face_box(b, f).padded(tol * 10.0)).collect();
    let (ta, tb) = (Bvh::build(&boxes_a), Bvh::build(&boxes_b));
    let mut cuts: Vec<CutSeg> = vec![];
    for (i, j) in ta.overlap_pairs(&tb) {
        let (f, g) = (fa[i], fb[j]);
        let (sf, sg) = (&a.face(f).surface, &b.face(g).surface);
        let bx = boxes_a[i].union(&boxes_b[j]);
        let bounds = Bounds { lo: bx.min, hi: bx.max };
        let before = cuts.len();
        match ss::intersect(sf, sg, Some(bounds), tol) {
            SsResult::Disjoint => {}
            SsResult::Coincident => {
                // imprint each face's boundary onto the other
                for (src, sf_id, dst, df_id) in [(1usize, g, 0usize, f), (0usize, f, 1usize, g)] {
                    let sb = bodies[src];
                    for e in sb.face_edges(sf_id) {
                        let ed = sb.edge(e);
                        for (t0, t1) in trim(&ed.curve, (ed.t0, ed.t1), &[(bodies[dst], df_id)], &bx, tol, true) {
                            cuts.push(CutSeg { curve: ed.curve.clone(), t0, t1, faces: vec![(dst, df_id)] });
                        }
                    }
                }
                arr.interacting_pairs += 1;
            }
            SsResult::Curves(cs) => {
                for c in cs {
                    let curve = c.curve().clone();
                    let r = curve.range();
                    for (t0, t1) in trim(&curve, r, &[(a, f), (b, g)], &bx, tol, false) {
                        cuts.push(CutSeg { curve: curve.clone(), t0, t1, faces: vec![(0, f), (1, g)] });
                    }
                }
            }
        }
        if cuts.len() > before {
            arr.interacting_pairs += 1;
        }
    }
    // ---- 2. vertex pool: original vertices, then cut ends
    let mut pts: Vec<Vec3> = vec![];
    for body in bodies {
        for v in body.vertex_ids() {
            pts.push(body.vertex(v).p);
        }
    }
    for c in &cuts {
        pts.push(c.curve.eval(c.t0));
        pts.push(c.curve.eval(c.t1));
    }
    for &p in &pts {
        arr.vertex(p);
    }
    let pool = arr.verts.clone();
    // ---- 3. split original edges and cut segments at pool points on them
    let split_params = |c: &Curve, t0: f64, t1: f64| -> Vec<f64> {
        let mut ts = vec![t0, t1];
        for &p in &pool {
            let (t, d) = dist_in_range(c, (t0, t1), p);
            if d <= tol {
                let s = param_slack(c, t, tol);
                if t > t0 + s && t < t1 - s {
                    ts.push(t);
                }
            }
        }
        ts.sort_by(|x, y| x.partial_cmp(y).unwrap());
        ts.dedup_by(|x, y| (*x - *y).abs() <= 1e-12);
        ts
    };
    // original edge pieces: (side, edge) -> [(arr edge, same_dir)]
    let mut pieces: Vec<Vec<Vec<(usize, bool)>>> = vec![vec![], vec![]];
    for (s, body) in bodies.iter().enumerate() {
        pieces[s] = vec![vec![]; body.edges.len()];
        for e in body.edge_ids() {
            let ed = body.edge(e);
            let ts = split_params(&ed.curve, ed.t0, ed.t1);
            let mut out = vec![];
            for w in ts.windows(2) {
                let v0 = if w[0] == ed.t0 { arr.vertex(body.vertex(ed.v0).p) } else { arr.vertex(ed.curve.eval(w[0])) };
                let v1 = if w[1] == ed.t1 { arr.vertex(body.vertex(ed.v1).p) } else { arr.vertex(ed.curve.eval(w[1])) };
                out.push(arr.edge(&ed.curve, w[0], w[1], v0, v1, Some((s, e))));
            }
            pieces[s][e.idx()] = out;
        }
    }
    // cut pieces
    let mut cut_edges: Vec<(usize, Vec<(usize, FaceId)>)> = vec![];
    for c in &cuts {
        let ts = split_params(&c.curve, c.t0, c.t1);
        for w in ts.windows(2) {
            let v0 = arr.vertex(c.curve.eval(w[0]));
            let v1 = arr.vertex(c.curve.eval(w[1]));
            if v0 == v1 && c.curve.eval(w[0]).dist(c.curve.eval(0.5 * (w[0] + w[1]))) <= tol {
                continue; // degenerate
            }
            let (ei, _) = arr.edge(&c.curve, w[0], w[1], v0, v1, None);
            cut_edges.push((ei, c.faces.clone()));
        }
    }
    // ---- 4. faces
    for (s, body) in bodies.iter().enumerate() {
        for f in body.face_ids() {
            let face = body.face(f);
            let mut bound = vec![];
            let mut n_co = 0;
            for &l in &face.loops {
                let mut lp = vec![];
                for &c in &body.lp(l).coedges {
                    n_co += 1;
                    let ce = body.coedge(c);
                    let ps = &pieces[s][ce.edge.idx()];
                    if ce.rev {
                        for &(ei, same) in ps.iter().rev() {
                            lp.push((ei, same)); // rev XOR !same == same
                        }
                    } else {
                        for &(ei, same) in ps {
                            lp.push((ei, !same));
                        }
                    }
                }
                bound.push(lp);
            }
            let mut inner = vec![];
            for (ei, fs) in &cut_edges {
                if !fs.contains(&(s, f)) {
                    continue;
                }
                if bound.iter().flatten().any(|&(x, _)| x == *ei) || inner.contains(ei) {
                    continue;
                }
                inner.push(*ei);
            }
            arr.faces.push(ArrFace {
                side: s,
                face: f,
                surface: face.surface.clone(),
                sense: face.sense,
                name: face.name.clone(),
                bound,
                inner,
                n_coedges: n_co,
            });
        }
    }
    arr
}

/// Trim curve `c` over `r` to the parts inside all `faces`. Breakpoints:
/// crossings with every boundary edge of every face (+ overlap ends).
/// Kept: intervals whose midpoint is not Out of any face and (unless
/// `allow_all_on`) not On the boundary of all of them.
fn trim(c: &Curve, r: (f64, f64), faces: &[(&Body, FaceId)], bx: &Aabb, tol: f64, allow_all_on: bool) -> Vec<(f64, f64)> {
    let r = crate::intersect::finite_range(c, r, bx.min, bx.max);
    if !(r.0.is_finite() && r.1.is_finite()) || r.1 - r.0 <= 0.0 {
        return vec![];
    }
    let mut ts = vec![r.0, r.1];
    for &(body, f) in faces {
        for e in body.face_edges(f) {
            let ed = body.edge(e);
            let res = cc::intersect(c, r, &ed.curve, (ed.t0, ed.t1), tol);
            ts.extend(res.hits.iter().map(|h| h.t1));
            for o in res.overlaps {
                ts.push(o.a.0);
                ts.push(o.a.1);
            }
        }
    }
    ts.sort_by(|x, y| x.partial_cmp(y).unwrap());
    ts.dedup_by(|x, y| (*x - *y).abs() <= 1e-12 || (*x - *y).abs() <= param_slack(c, *y, tol * 0.5));
    let mut out: Vec<(f64, f64)> = vec![];
    for w in ts.windows(2) {
        let (t0, t1) = (w[0], w[1]);
        if c.eval(t0).dist(c.eval(t1)) <= tol && c.eval(t0).dist(c.eval(0.5 * (t0 + t1))) <= tol {
            continue;
        }
        let m = c.eval(0.5 * (t0 + t1));
        let mut all_on = true;
        let mut out_any = false;
        for &(body, f) in faces {
            match point_in_face(body, f, m, tol) {
                Where::Out => out_any = true,
                Where::In => all_on = false,
                Where::On => {}
            }
        }
        if out_any || (all_on && !allow_all_on) {
            continue;
        }
        out.push((t0, t1));
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::boolean::tests::{box_body, cyl_body};

    #[test]
    fn box_cylinder_arrangement() {
        let a = box_body("F2", 0.0, 0.0, 40.0, 20.0, 0.0, 10.0);
        let b = cyl_body("F4", 20.0, 10.0, 5.0, -1.0, 11.0);
        let arr = imprint(&a, &b, 1e-6);
        // C0 and C1 of the box get one inner circle each; the tool
        // cylinder gets both circles inside
        let c0 = arr.faces.iter().find(|f| f.side == 0 && f.name.id() == "F2.C0").unwrap();
        assert_eq!(c0.inner.len(), 1);
        let side = arr.faces.iter().find(|f| f.side == 1 && f.name.id() == "F4.S.e1").unwrap();
        assert_eq!(side.inner.len(), 2);
        assert!(side.touched());
        let s1 = arr.faces.iter().find(|f| f.side == 0 && f.name.id() == "F2.S.e1").unwrap();
        assert!(!s1.touched());
    }
}

