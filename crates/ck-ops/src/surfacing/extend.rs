//! Surface extension.
//!
//! Extend a sheet face across one of its open boundary edges. Pinned by
//! oracle probes:
//! - planar square, extend the y=0 edge by 5 (probe ext1): the face grows
//!   to the exact rectangle y ∈ [−5,10]; the face keeps its name `F2.P`;
//!   ALL the face's edges are rebuilt (fresh edge objects). Edge order =
//!   the original loop rotated to the moved edge; the moved edge and the
//!   edge at its far end are stored ALONG the loop walk, every further
//!   edge AGAINST it (probe: `[(0,-5)->(10,-5), (10,-5)->(10,10),
//!   (0,10)->(10,10), (0,-5)->(0,10)]`).
//! - half-cylinder strip, extend the z=0 rim arc by 3 (probe ext2): the
//!   rim moves to z=−3 (the profile translated along the sweep direction),
//!   the rails stretch to the new rim, the far rim is unchanged; same
//!   stored-direction pattern.
//! - extend `end:"surface"` to a target plane (probe ext3b): the face
//!   grows exactly to the target plane y=−7 (see `reach`).
//!
//! Errors: `extendNotBoundary` ("only open boundary edges extend"),
//! `extendNoTarget`, `extendPointBehind`, `extendUnsupported` ("the face
//! is degenerate at the edge"), `extendFolds` ("the surface folds where it
//! continues").
//!
//! HOW TO WIRE (ck-model, Gate B): feature `extend {edges:[{faces,at}…],
//! shape, end, distance, point, target}` → resolve the edges of the
//! current body (`missingReference`), then
//! `extend::extend(&body, &edges, shape, end, &feat_id, 1e-6)`; the result
//! body registers under the feature id. `end` = Distance(d) or
//! Surface(plane) — resolve the target ref first. `shape`
//! same/linear/soft/reflective all extend naturally on analytic surfaces
//! (see extend/shape.rs).

mod reach;
pub mod shape;

use crate::surfacing::knit::BodyExt;
use ck_geom::curve::Curve;
use ck_geom::surface::Surface;
use ck_geom::{Frame, Vec3};
use ck_topo::body::{Body, EdgeId, FaceId};

pub use reach::{reach_distance, ReachError};
pub use shape::Shape;

#[derive(Debug, Clone)]
pub enum End {
    Distance(f64),
    /// extend until the boundary reaches the target surface
    Surface(Surface),
}

#[derive(Debug, Clone)]
pub struct ExtendError {
    pub code: &'static str,
    pub detail: String,
}

impl ExtendError {
    fn code(code: &'static str) -> ExtendError {
        ExtendError { code, detail: String::new() }
    }
    fn detail(code: &'static str, detail: impl Into<String>) -> ExtendError {
        ExtendError { code, detail: detail.into() }
    }
}

impl std::fmt::Display for ExtendError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        if self.detail.is_empty() {
            write!(f, "{}", self.code)
        } else {
            write!(f, "{}: {}", self.code, self.detail)
        }
    }
}

/// Extend each given open boundary edge of its face.
pub fn extend(body: &Body, edges: &[EdgeId], shape: Shape, end: &End, feature: &str, tol: f64) -> Result<Body, ExtendError> {
    let _ = feature;
    if edges.is_empty() {
        return Err(ExtendError::code("missingReference"));
    }
    for &e in edges {
        let ok = e.idx() < body.edges.len()
            && body.edge(e).alive
            && body.edge_coedges(e).iter().filter(|&&c| body.coedge(c).alive).count() == 1;
        if !ok {
            return Err(ExtendError::detail("extendNotBoundary", "only open boundary edges extend"));
        }
    }
    let mut out = body.clone();
    let mut done: Vec<FaceId> = Vec::new();
    for &e in edges {
        let f = out
            .edge_coedges(e)
            .iter()
            .map(|&c| out.coedge_face(c))
            .find(|&f| !done.contains(&f))
            .ok_or_else(|| ExtendError::code("extendNotBoundary"))?;
        done.push(f);
        extend_face(&mut out, f, e, shape, end, tol)?;
    }
    Ok(out)
}

fn map_reach(e: ReachError) -> ExtendError {
    ExtendError { code: e.code, detail: e.detail }
}

/// The face's loop as (edge, coedge-rev) in loop order.
fn face_loop(b: &Body, f: FaceId) -> Vec<(EdgeId, bool)> {
    let mut out = Vec::new();
    for l in &b.face(f).loops {
        if !b.lp(*l).alive {
            continue;
        }
        for &c in &b.lp(*l).coedges {
            if b.coedge(c).alive {
                out.push((b.coedge(c).edge, b.coedge(c).rev));
            }
        }
    }
    out
}

/// Rebuild `f` extended across `e`.
fn extend_face(out: &mut Body, f: FaceId, e: EdgeId, shape: Shape, end: &End, tol: f64) -> Result<(), ExtendError> {
    let _ = shape;
    let loop0 = face_loop(out, f);
    if loop0.iter().any(|&(x, _)| out.edge_coedges(x).len() > 1) {
        // shared edges would need splitting in neighbours — single-face
        // sheets only in this pass
        return Err(ExtendError::code("extendUnsupported"));
    }
    let pos = loop0.iter().position(|&(x, _)| x == e).expect("edge in face");
    let face = out.face(f);
    let surface = face.surface.clone();
    let sense = face.sense;
    let name = face.name.clone();
    let shell = face.shell;

    // the moved edge's curve per surface type
    let moved: Curve = match &surface {
        Surface::Plane { frame } => {
            let ed = out.edge(e);
            let Curve::Line { o, d: du } = &ed.curve else {
                return Err(ExtendError::code("extendUnsupported"));
            };
            let u = du.normalize();
            let ext = frame.z.cross(u).normalize();
            // away from the face interior (loop centroid)
            let mut c = Vec3::ZERO;
            let mut n = 0.0;
            for &(x, _) in &loop0 {
                let ed2 = out.edge(x);
                c = c + ed2.curve.eval((ed2.t0 + ed2.t1) * 0.5);
                n += 1.0;
            }
            c = c.scale(1.0 / n);
            let mid = ed.curve.eval((ed.t0 + ed.t1) * 0.5);
            let ext = if ext.dot(c - mid) > 0.0 { ext.scale(-1.0) } else { ext };
            let dist = match end {
                End::Distance(d) => *d,
                End::Surface(s) => reach_distance(mid, ext, s, face_size(out, f)).map_err(map_reach)?,
            };
            Curve::Line { o: *o + ext.scale(dist), d: *du }
        }
        Surface::Cylinder { frame, r } => {
            // rim edges (circles/arcs in a plane perpendicular to the axis)
            // move along the axis
            let ed = out.edge(e);
            let Curve::Circle { frame: cf, .. } = &ed.curve else {
                return Err(ExtendError::code("extendUnsupported"));
            };
            if cf.z.cross(frame.z).len() > 1e-9 {
                return Err(ExtendError::code("extendUnsupported"));
            }
            let p = ed.curve.eval(ed.t0);
            let q = ed.curve.eval(ed.t1);
            let v0 = (p - frame.o).dot(frame.z);
            let v1 = (q - frame.o).dot(frame.z);
            if (v0 - v1).abs() > tol {
                return Err(ExtendError::code("extendUnsupported"));
            }
            let v = v0;
            // the face's other rim-like edge (constant height)
            let other_v = loop0
                .iter()
                .find_map(|&(x, _)| {
                    if x == e {
                        return None;
                    }
                    let ed2 = out.edge(x);
                    let a = ed2.curve.eval(ed2.t0);
                    let b = ed2.curve.eval(ed2.t1);
                    let (h0, h1) = ((a - frame.o).dot(frame.z), (b - frame.o).dot(frame.z));
                    ((h0 - h1).abs() <= tol).then_some(h0)
                })
                .ok_or_else(|| ExtendError::code("extendUnsupported"))?;
            let dir = if other_v > v { -1.0 } else { 1.0 };
            let dist = match end {
                End::Distance(dd) => *dd,
                End::Surface(s) => {
                    let mid = ed.curve.eval(ed.t0);
                    reach_distance(mid, frame.z.scale(dir), s, face_size(out, f)).map_err(map_reach)?
                }
            };
            let new_v = v + dir * dist;
            let centre = frame.o + frame.z.scale(new_v);
            Curve::Circle { frame: Frame { o: centre, ..cf.clone() }, r: *r }
        }
        _ => return Err(ExtendError::code("extendUnsupported")),
    };

    // the moved edge's endpoints (images of the original's v0/v1 ends)
    let (ov0, ov1) = (out.edge(e).v0, out.edge(e).v1);
    // the moved edge keeps the original's parameter range
    let (mt0, mt1) = (out.edge(e).t0, out.edge(e).t1);
    let corner_start = moved.eval(mt0); // image of the original v0 corner
    let corner_end = moved.eval(mt1); // image of the original v1 corner

    // kill the old edges (single-face bodies only — checked above)
    let old_edges: Vec<EdgeId> = loop0.iter().map(|&(x, _)| x).collect();

    // new edges in walk order (the loop rotated to the moved edge);
    // stored along the walk for k <= 1, against for k >= 2 (probe-pinned)
    let m = loop0.len();
    let mut created: Vec<(EdgeId, bool)> = Vec::new();
    for k in 0..m {
        let idx = (pos + k) % m;
        let (oe, orev) = loop0[idx];
        let ed = out.edge(oe);
        if idx == pos {
            let va = out.add_vertex(corner_start);
            let vb = out.add_vertex(corner_end);
            created.push((out.add_edge(moved.clone(), mt0, mt1, va, vb), false));
            continue;
        }
        if k == 1 {
            // stretched end-side: from the moved end corner to the far one
            let far_v = if ed.v0 == ov1 { ed.v1 } else { ed.v0 };
            let far_p = out.vertex(far_v).p;
            let va = out.add_vertex(corner_end);
            let vb = out.add_vertex(far_p);
            created.push((out.add_edge(Curve::line_through(corner_end, far_p), 0.0, (far_p - corner_end).len(), va, vb), false));
            continue;
        }
        if k == m - 1 {
            // stretched start-side: stored against the walk (moved corner
            // -> far), the walk runs far -> moved corner
            let far_v = if ed.v0 == ov0 { ed.v1 } else { ed.v0 };
            let far_p = out.vertex(far_v).p;
            let va = out.add_vertex(corner_start);
            let vb = out.add_vertex(far_p);
            created.push((out.add_edge(Curve::line_through(corner_start, far_p), 0.0, (far_p - corner_start).len(), va, vb), false));
            continue;
        }
        // unchanged middle edge: stored AGAINST the walk (the walk's
        // reverse direction)
        let (curve, t0, t1, q0, q1) = if orev {
            // walk runs the curve backwards; stored = the curve forwards
            (ed.curve.clone(), ed.t0, ed.t1, ed.curve.eval(ed.t0), ed.curve.eval(ed.t1))
        } else {
            match &ed.curve {
                Curve::Line { o, d } => {
                    let len = ed.t1 - ed.t0;
                    let p0 = *o + d.scale(len);
                    (Curve::Line { o: p0, d: -*d }, 0.0, len, p0, *o)
                }
                _ => (ed.curve.reversed(), -ed.t1, -ed.t0, ed.curve.eval(ed.t1), ed.curve.eval(ed.t0)),
            }
        };
        let va = out.add_vertex(q0);
        let vb = out.add_vertex(q1);
        created.push((out.add_edge(curve, t0, t1, va, vb), false));
    }
    for oe in old_edges {
        out.edge_mut(oe).alive = false;
    }
    // rebuild the face (new id, appended) and chain its loop
    let nf = out.add_face(shell, surface, sense, name);
    out.face_mut(f).alive = false;
    let nloop = out.add_loop(nf);
    weld_vertices(out, tol);
    let mut chain: Vec<(EdgeId, bool)> = Vec::new();
    let mut placed = vec![false; created.len()];
    chain.push(created[0]);
    placed[0] = true;
    while chain.len() < created.len() {
        let last = *chain.last().unwrap();
        let tail = if last.1 { out.edge(last.0).v0 } else { out.edge(last.0).v1 };
        let nxt = (0..created.len())
            .find(|&i| {
                !placed[i]
                    && (out.edge(created[i].0).v0 == tail || out.edge(created[i].0).v1 == tail)
            })
            .or_else(|| {
                if std::env::var("CK_EXT_DEBUG").is_ok() {
                    for (i, &(ne, _)) in created.iter().enumerate() {
                        let ed = out.edge(ne);
                        eprintln!("DBG edge[{i}] v0={:?} v1={:?} tail={:?}", out.vertex(ed.v0).p, out.vertex(ed.v1).p, out.vertex(tail).p);
                    }
                }
                None
            })
            .ok_or_else(|| ExtendError::code("extendFolds"))?;
        let (ne, _) = created[nxt];
        let rev = out.edge(ne).v0 != tail;
        chain.push((ne, rev));
        placed[nxt] = true;
    }
    for (ne, rev) in chain {
        out.add_coedge(nloop, ne, rev, None);
    }
    Ok(())
}

fn face_size(b: &Body, f: FaceId) -> f64 {
    let mut lo = Vec3::ZERO;
    let mut hi = Vec3::ZERO;
    let mut first = true;
    for e in b.face_edges(f) {
        let ed = b.edge(e);
        for t in [ed.t0, (ed.t0 + ed.t1) * 0.5, ed.t1] {
            let p = ed.curve.eval(t);
            if first {
                lo = p;
                hi = p;
                first = false;
            } else {
                lo = lo.min(p);
                hi = hi.max(p);
            }
        }
    }
    (hi - lo).len()
}

/// Merge coincident vertices.
fn weld_vertices(b: &mut Body, tol: f64) {
    let ids: Vec<ck_topo::body::VertexId> = b.vertex_ids().collect();
    let n = ids.len();
    let mut parent: Vec<usize> = (0..n).collect();
    for i in 0..n {
        for j in (i + 1)..n {
            if b.vertex(ids[i]).p.dist(b.vertex(ids[j]).p) <= tol {
                let (mut a, mut c) = (i, j);
                while parent[a] != a {
                    a = parent[a];
                }
                while parent[c] != c {
                    c = parent[c];
                }
                if a != c {
                    parent[c] = a;
                }
            }
        }
    }
    let root = |mut x: usize, p: &[usize]| {
        while p[x] != x {
            x = p[x];
        }
        x
    };
    let roots: Vec<usize> = (0..n).map(|i| root(i, &parent)).collect();
    for i in 0..n {
        if roots[i] == i {
            continue;
        }
        let v = ids[i];
        let target = ids[roots[i]];
        for e in b.edge_ids().collect::<Vec<_>>() {
            if b.edge(e).v0 == v {
                b.edge_mut(e).v0 = target;
            }
            if b.edge(e).v1 == v {
                b.edge_mut(e).v1 = target;
            }
        }
        b.vertex_mut(v).alive = false;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use ck_topo::body::{FaceName, Role};

    fn nm(feat: &str, role: &str) -> FaceName {
        FaceName { feature: feat.into(), role: Role::Other(role.into()), region: None, suffix: String::new() }
    }

    /// Square planar sheet x,y ∈ [0,10] at z=0, natural +z (probe ext1).
    fn square_sheet() -> Body {
        let mut b = Body::new();
        let lump = b.add_lump();
        let shell = b.add_shell(lump);
        let f = b.add_face(shell, Surface::plane(Vec3::ZERO, Vec3::Z), true, nm("F2", "P"));
        let p = [Vec3::ZERO, Vec3::new(10.0, 0.0, 0.0), Vec3::new(10.0, 10.0, 0.0), Vec3::new(0.0, 10.0, 0.0)];
        let lp = b.add_loop(f);
        let vs: Vec<_> = p.iter().map(|&q| b.add_vertex(q)).collect();
        for i in 0..4 {
            let e = b.add_edge(Curve::line_through(p[i], p[(i + 1) % 4]), 0.0, (p[(i + 1) % 4] - p[i]).len(), vs[i], vs[(i + 1) % 4]);
            b.add_coedge(lp, e, false, None);
        }
        b
    }

    /// Half-cylinder strip r=5, angle 0..π, z ∈ [0,10] (probe ext2).
    fn half_cyl() -> Body {
        let mut b = Body::new();
        let lump = b.add_lump();
        let shell = b.add_shell(lump);
        let f = b.add_face(
            shell,
            Surface::Cylinder { frame: Frame { o: Vec3::ZERO, x: Vec3::X, y: Vec3::Y, z: Vec3::Z }, r: 5.0 },
            true,
            FaceName { feature: "F2".into(), role: Role::Side("e1".into()), region: None, suffix: String::new() },
        );
        let arc = |z: f64| Curve::circle_x(Vec3::new(0.0, 0.0, z), Vec3::Z, Vec3::X, 5.0);
        let p0 = Vec3::new(5.0, 0.0, 0.0);
        let p1 = Vec3::new(-5.0, 0.0, 0.0);
        let p2 = Vec3::new(-5.0, 0.0, 10.0);
        let p3 = Vec3::new(5.0, 0.0, 10.0);
        let vs = [b.add_vertex(p0), b.add_vertex(p1), b.add_vertex(p2), b.add_vertex(p3)];
        let rim0 = b.add_edge(arc(0.0), 0.0, std::f64::consts::PI, vs[0], vs[1]);
        let rail1 = b.add_edge(Curve::line_through(p1, p2), 0.0, 10.0, vs[1], vs[2]);
        let rim1 = b.add_edge(arc(10.0), 0.0, std::f64::consts::PI, vs[3], vs[2]);
        let rail0 = b.add_edge(Curve::line_through(p0, p3), 0.0, 10.0, vs[0], vs[3]);
        // loop: rim0 fwd, rail1 fwd, rim1 rev, rail0 rev (CCW in uv)
        let lp = b.add_loop(f);
        b.add_coedge(lp, rim0, false, None);
        b.add_coedge(lp, rail1, false, None);
        b.add_coedge(lp, rim1, true, None);
        b.add_coedge(lp, rail0, true, None);
        b
    }

    fn ends(b: &Body, e: EdgeId) -> (Vec3, Vec3) {
        let ed = b.edge(e);
        (ed.curve.eval(ed.t0), ed.curve.eval(ed.t1))
    }

    #[test]
    fn extend_sheet_edge_oracle() {
        // probe ext1: extend the y=0 edge (at (5,0,0)) by 5
        let body = square_sheet();
        let e0 = b_edge_at(&body, 0.0);
        let b = extend(&body, &[e0], Shape::Same, &End::Distance(5.0), "F3", 1e-6).unwrap();
        let f = b.face_ids().find(|&f| b.face(f).name.id() == "F2.P").unwrap();
        assert_eq!(b.face(f).name.id(), "F2.P");
        let eids: Vec<EdgeId> = b.edge_ids().collect();
        assert_eq!(eids.len(), 4);
        let want: &[([f64; 3], [f64; 3])] = &[
            ([0.0, -5.0, 0.0], [10.0, -5.0, 0.0]),
            ([10.0, -5.0, 0.0], [10.0, 10.0, 0.0]),
            ([0.0, 10.0, 0.0], [10.0, 10.0, 0.0]),
            ([0.0, -5.0, 0.0], [0.0, 10.0, 0.0]),
        ];
        for (i, (a, c)) in want.iter().enumerate() {
            let (p, q) = ends(&b, eids[i]);
            assert!(p.dist(Vec3::new(a[0], a[1], a[2])) < 1e-9 && q.dist(Vec3::new(c[0], c[1], c[2])) < 1e-9, "edge {i} {p:?} {q:?}");
        }
    }

    #[test]
    fn extend_cyl_rim_oracle() {
        // probe ext2: extend the z=0 rim arc by 3 -> rim at z=-3
        let body = half_cyl();
        let rim0 = b_rim_low(&body);
        let b = extend(&body, &[rim0], Shape::Same, &End::Distance(3.0), "F3", 1e-6).unwrap();
        let eids: Vec<EdgeId> = b.edge_ids().collect();
        assert_eq!(eids.len(), 4);
        match &b.edge(eids[0]).curve {
            Curve::Circle { frame, r } => {
                assert!((*r - 5.0).abs() < 1e-9);
                assert!(frame.o.dist(Vec3::new(0.0, 0.0, -3.0)) < 1e-9);
            }
            _ => panic!("edge 0 not a circle"),
        }
        let (p, q) = ends(&b, eids[0]);
        eprintln!("DBG rim0 {p:?} -> {q:?}");
        assert!(p.dist(Vec3::new(5.0, 0.0, -3.0)) < 1e-9 && q.dist(Vec3::new(-5.0, 0.0, -3.0)) < 1e-9, "{p:?} {q:?}");
        let (p, q) = ends(&b, eids[1]);
        assert!(p.dist(Vec3::new(-5.0, 0.0, -3.0)) < 1e-9 && q.dist(Vec3::new(-5.0, 0.0, 10.0)) < 1e-9, "{p:?} {q:?}");
        let (p, q) = ends(&b, eids[2]);
        assert!(p.dist(Vec3::new(5.0, 0.0, 10.0)) < 1e-9 && q.dist(Vec3::new(-5.0, 0.0, 10.0)) < 1e-9, "{p:?} {q:?}");
        let (p, q) = ends(&b, eids[3]);
        assert!(p.dist(Vec3::new(5.0, 0.0, -3.0)) < 1e-9 && q.dist(Vec3::new(5.0, 0.0, 10.0)) < 1e-9, "{p:?} {q:?}");
    }

    #[test]
    fn extend_to_target_plane_oracle() {
        // probe ext3b: extend the y=0 edge to the plane y=-7
        let body = square_sheet();
        let e0 = b_edge_at(&body, 0.0);
        let target = Surface::Plane {
            frame: Frame { o: Vec3::new(0.0, -7.0, 0.0), x: Vec3::X, y: Vec3::Z, z: Vec3::Y },
        };
        let b = extend(&body, &[e0], Shape::Same, &End::Surface(target), "F6", 1e-6).unwrap();
        let (mut lo, mut hi) = (f64::INFINITY, f64::NEG_INFINITY);
        for e in b.edge_ids() {
            let (p, q) = ends(&b, e);
            lo = lo.min(p.y).min(q.y);
            hi = hi.max(p.y).max(q.y);
        }
        assert!((lo + 7.0).abs() < 1e-9 && (hi - 10.0).abs() < 1e-9, "y range {lo}..{hi}");
    }

    #[test]
    fn extend_errors() {
        let body = square_sheet();
        let dead = ck_topo::body::EdgeId(u32::MAX);
        let e = extend(&body, &[dead], Shape::Same, &End::Distance(5.0), "F3", 1e-6).unwrap_err();
        assert_eq!(e.code, "extendNotBoundary");
        assert_eq!(e.detail, "only open boundary edges extend");
        let e0 = b_edge_at(&body, 0.0);
        let far = Surface::Plane {
            frame: Frame { o: Vec3::new(0.0, -300.0, 0.0), x: Vec3::X, y: Vec3::Z, z: Vec3::Y },
        };
        let e = extend(&body, &[e0], Shape::Same, &End::Surface(far), "F3", 1e-6).unwrap_err();
        assert_eq!(e.code, "extendNoTarget");
        assert!(e.detail.contains("10 times the face's size"), "{}", e.detail);
    }

    fn b_edge_at(b: &Body, y: f64) -> EdgeId {
        let f = b.face_ids().next().unwrap();
        b.face_edges(f)
            .into_iter()
            .find(|&e| {
                let ed = b.edge(e);
                let m = ed.curve.eval((ed.t0 + ed.t1) * 0.5);
                (m.y - y).abs() < 1e-9
            })
            .unwrap()
    }

    fn b_rim_low(b: &Body) -> EdgeId {
        let f = b.face_ids().next().unwrap();
        b.face_edges(f)
            .into_iter()
            .find(|&e| matches!(b.edge(e).curve, Curve::Circle { .. }) && b.edge(e).curve.eval(b.edge(e).t0).z < 1e-9)
            .unwrap()
    }
}
