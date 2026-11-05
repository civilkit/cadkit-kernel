//! Vertex chamfer (`chamferType: "vertex"`, kernel.strings `chamferVertexEdges`,
//! f2988) on a straight-edged 3-valent vertex of planar faces: the corner is
//! cut off by the plane through the points at distances `d1, d2, d3` along
//! the vertex's three edges. RECONSTRUCTED from oracle probes on a box
//! (tools/probe_vertex_chamfer.mjs), not a line-by-line port:
//!
//! * the vertex is the one nearest to `at` (feature `at`, else the first
//!   selected edge's pick point); the selected faces are ignored;
//! * the vertex's edges, in arena (id) order, take `distance`, `distance2`,
//!   `distance3` (the last two default to `distance`);
//! * `distance*` <= 0 -> `invalidValue`; `distance* >= edge length` ->
//!   `blendTooLarge`; no vertex edges besides three -> `chamferVertexEdges`;
//! * result: one planar blend face `B0`; faces ordered untouched, the three
//!   corner faces counter-clockwise about the new face's normal starting at
//!   the face after the first edge, then `B0`; edges numbered in place with
//!   three slots reserved after the input's for the trimmed edges (slot =
//!   input count + the edge's index among the vertex edges) and the three
//!   `B0` edges after them in the same face order.

use super::build::{apply_tool, in_place_layout_opts, BlendRef, LayoutOpts};
use super::BlendError;
use crate::boolean::Op;
use ck_geom::curve::Curve;
use ck_geom::surface::Surface;
use ck_geom::Vec3;
use ck_topo::body::{Body, EdgeId, FaceName, VertexId};

/// Closed polyhedron from planar polygons given counter-clockwise about
/// their outward normals.
pub fn polyhedron(polys: &[(Vec<Vec3>, FaceName)]) -> Body {
    let mut b = Body::new();
    let lump = b.add_lump();
    let shell = b.add_shell(lump);
    let mut verts: Vec<(Vec3, VertexId)> = vec![];
    let mut edges: Vec<((usize, usize), EdgeId)> = vec![];
    for (pts, name) in polys {
        // Newell normal
        let mut n = Vec3::ZERO;
        for i in 0..pts.len() {
            let (p, q) = (pts[i], pts[(i + 1) % pts.len()]);
            n = n + Vec3::new((p.y - q.y) * (p.z + q.z), (p.z - q.z) * (p.x + q.x), (p.x - q.x) * (p.y + q.y));
        }
        let frame = ck_geom::frame_from(pts[0], n.normalize(), pts[1] - pts[0]);
        let f = b.add_face(shell, Surface::Plane { frame }, true, name.clone());
        let lp = b.add_loop(f);
        for i in 0..pts.len() {
            let (p, q) = (pts[i], pts[(i + 1) % pts.len()]);
            let mut vid = |b: &mut Body, p: Vec3| -> (usize, VertexId) {
                if let Some(k) = verts.iter().position(|(x, _)| x.dist(p) < 1e-12) {
                    return (k, verts[k].1);
                }
                let v = b.add_vertex(p);
                verts.push((p, v));
                (verts.len() - 1, v)
            };
            let (ka, va) = vid(&mut b, p);
            let (kq, vq) = vid(&mut b, q);
            let key = (ka.min(kq), ka.max(kq));
            let rev = ka > kq;
            let e = match edges.iter().find(|(k, _)| *k == key) {
                Some((_, e)) => *e,
                None => {
                    let (lo, hi, vlo, vhi) = if rev { (q, p, vq, va) } else { (p, q, va, vq) };
                    let len = lo.dist(hi);
                    let e = b.add_edge(Curve::Line { o: lo, d: (hi - lo) / len }, 0.0, len, vlo, vhi);
                    edges.push((key, e));
                    e
                }
            };
            let ed = b.edge(e).clone();
            let Curve::Line { o, d } = ed.curve else { unreachable!() };
            let pc = ck_topo::make::plane_line_pc(&frame, o, d);
            b.add_coedge(lp, e, rev, Some(pc));
        }
    }
    ck_topo::region::rebuild_lumps(&mut b);
    b
}

/// `chamferType: "vertex"` at the vertex nearest `at`; `dist` = (distance,
/// distance2, distance3).
pub fn chamfer_vertex(b: &Body, at: Vec3, dist: [f64; 3], feature: &str, tol: f64) -> Result<Body, BlendError> {
    if dist.iter().any(|d| !(*d > 0.0) || !d.is_finite()) {
        return Err(BlendError::code("invalidValue"));
    }
    let v = b
        .vertex_ids()
        .min_by(|&x, &y| b.vertex(x).p.dist(at).total_cmp(&b.vertex(y).p.dist(at)))
        .ok_or_else(|| BlendError::code("missingReference"))?;
    let vp = b.vertex(v).p;
    let mut inc: Vec<EdgeId> = vec![];
    for e in b.edge_ids() {
        let ed = b.edge(e);
        if (ed.v0 == v || ed.v1 == v) && !inc.contains(&e) {
            inc.push(e);
        }
    }
    if inc.len() != 3 || inc.iter().any(|&e| b.edge(e).v0 == b.edge(e).v1) {
        return Err(BlendError::code("chamferVertexEdges"));
    }
    // points along the edges
    let mut pts = [Vec3::ZERO; 3];
    for (i, &e) in inc.iter().enumerate() {
        let ed = b.edge(e);
        if !matches!(ed.curve, Curve::Line { .. }) {
            return Err(BlendError::code("blendUnsupported"));
        }
        let other = if ed.v0 == v { b.vertex(ed.v1).p } else { b.vertex(ed.v0).p };
        let len = vp.dist(other);
        if dist[i] >= len - 1e-9 {
            return Err(BlendError::code("blendTooLarge"));
        }
        pts[i] = vp + (other - vp) / len * dist[i];
    }
    // the face shared by edges i and j
    let face_of = |i: usize, j: usize| {
        let (fi, fj) = (b.edge_faces(inc[i]), b.edge_faces(inc[j]));
        fi.iter().copied().find(|f| fj.contains(f))
    };
    let mut cf = vec![];
    for (i, j) in [(0, 1), (1, 2), (2, 0)] {
        let f = face_of(i, j).ok_or_else(|| BlendError::code("chamferVertexEdges"))?;
        if !matches!(b.face(f).surface, Surface::Plane { .. }) {
            return Err(BlendError::code("blendUnsupported"));
        }
        cf.push(f);
    }
    // normal of the cut plane pointing at the vertex
    let mut n = (pts[1] - pts[0]).cross(pts[2] - pts[0]).normalize();
    if n.dot(vp - pts[0]) < 0.0 {
        n = -n;
    }
    // counter-clockwise order of the points about n, starting at point 0
    let ccw = (pts[1] - pts[0]).cross(pts[2] - pts[0]).dot(n) > 0.0;
    let order: [usize; 3] = if ccw { [0, 1, 2] } else { [0, 2, 1] };
    let face_between = |a: usize, c: usize| match (a, c) {
        (0, 1) | (1, 0) => cf[0],
        (1, 2) | (2, 1) => cf[1],
        _ => cf[2],
    };
    let hint: Vec<String> = (0..3).map(|k| b.face_id_str(face_between(order[k], order[(k + 1) % 3]))).collect();
    // the tetrahedron V, P0, P1, P2
    let name = FaceName::blend(feature, 0);
    let centre = (vp + pts[0] + pts[1] + pts[2]) * 0.25;
    let outward = |poly: Vec<Vec3>| -> Vec<Vec3> {
        let nn = (poly[1] - poly[0]).cross(poly[2] - poly[0]);
        if nn.dot(centre - poly[0]) > 0.0 {
            poly.into_iter().rev().collect()
        } else {
            poly
        }
    };
    let polys = vec![
        (outward(vec![pts[0], pts[1], pts[2]]), name.clone()),
        (outward(vec![vp, pts[0], pts[1]]), FaceName::side(feature, "v01")),
        (outward(vec![vp, pts[1], pts[2]]), FaceName::side(feature, "v12")),
        (outward(vec![vp, pts[2], pts[0]]), FaceName::side(feature, "v20")),
    ];
    let tool = polyhedron(&polys);
    let (raw, _) = apply_tool(b, &tool, Op::Cut, &name, hint.clone(), tol)?;
    // in-place numbering
    let (slots, n_in) = b.wire_numbered();
    let slot_of = |e: EdgeId| slots.iter().find(|(_, ch)| ch.contains(&e)).map(|(s, _)| *s);
    let mut piece_slots = vec![];
    for (i, &e) in inc.iter().enumerate() {
        if let Some(s) = slot_of(e) {
            piece_slots.push((s, n_in + i));
        }
    }
    let refs = [BlendRef { name: name.id(), faces: [hint[0].clone(), hint[1].clone()], ends: [vp, vp] }];
    let out = in_place_layout_opts(b, &raw, &refs, tol, &LayoutOpts { reserve: Some(3), piece_slots, ..Default::default() });
    super::validity::check_vertex_cut(b, out, dist, tol)
}
