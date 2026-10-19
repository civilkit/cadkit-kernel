//! Validity checks: open / non-manifold edges, loop chaining, vertex-curve
//! consistency, pcurve-curve consistency, coedge pairing, degenerate edges.

use crate::body::{Body, CoedgeId, EdgeId, FaceId, LoopId, VertexId};

#[derive(Debug, Clone, PartialEq)]
pub enum Issue {
    /// Edge used by a single coedge.
    OpenEdge(EdgeId),
    /// Edge used by more than two coedges.
    NonManifoldEdge(EdgeId, usize),
    /// Edge with no live coedge (orphan).
    UnusedEdge(EdgeId),
    /// Manifold edge whose two coedges run the same way.
    BadPairing(EdgeId),
    /// end(coedge) != start(next coedge) in a loop.
    BrokenLoop(LoopId, CoedgeId),
    /// Vertex not on its edge's curve end (distance).
    VertexOffCurve(EdgeId, VertexId, f64),
    /// pcurve maps off the 3D curve (max deviation).
    PcurveOff(CoedgeId, f64),
    /// Zero-length edge.
    DegenerateEdge(EdgeId),
    /// Face without loops.
    EmptyFace(FaceId),
    /// t0 >= t1.
    BadRange(EdgeId),
}

/// Run all checks; `tol` is the model tolerance (mm).
pub fn check(b: &Body, tol: f64) -> Vec<Issue> {
    let mut out = vec![];
    for e in b.edge_ids() {
        let ed = b.edge(e);
        let cs = b.edge_coedges(e);
        match cs.len() {
            0 => out.push(Issue::UnusedEdge(e)),
            1 => out.push(Issue::OpenEdge(e)),
            2 => {
                if b.coedge(cs[0]).rev == b.coedge(cs[1]).rev {
                    out.push(Issue::BadPairing(e));
                }
            }
            n => out.push(Issue::NonManifoldEdge(e, n)),
        }
        if ed.t1 <= ed.t0 {
            out.push(Issue::BadRange(e));
        }
        let scale = 1.0 + ed.curve.eval(ed.t0).len();
        for (v, t) in [(ed.v0, ed.t0), (ed.v1, ed.t1)] {
            let d = b.vertex(v).p.dist(ed.curve.eval(t));
            if d > tol * scale * 10.0 {
                out.push(Issue::VertexOffCurve(e, v, d));
            }
        }
        // (an edge whose end points are 10 tolerances apart is longer than a tolerance, whatever the quadrature
        // would say of its length: only the short and the closed ones are measured)
        let chord = ed.curve.eval(ed.t0).dist(ed.curve.eval(ed.t1));
        if !(chord >= 10.0 * tol) && ed.curve.length_between(ed.t0, ed.t1) < tol {
            out.push(Issue::DegenerateEdge(e));
        }
        for &c in &cs {
            if let Some(pc) = &b.coedge(c).pcurve {
                let surf = &b.face(b.coedge_face(c)).surface;
                let mut dev: f64 = 0.0;
                for k in 0..=8 {
                    let t = ed.t0 + (ed.t1 - ed.t0) * k as f64 / 8.0;
                    let uv = pc.eval(t);
                    dev = dev.max(surf.eval(uv.x, uv.y).dist(ed.curve.eval(t)));
                }
                if dev > tol * scale * 10.0 {
                    out.push(Issue::PcurveOff(c, dev));
                }
            }
        }
    }
    for f in b.face_ids() {
        let face = b.face(f);
        if face.loops.is_empty() {
            out.push(Issue::EmptyFace(f));
        }
        for &l in &face.loops {
            let cs = &b.lp(l).coedges;
            for i in 0..cs.len() {
                let (c, n) = (cs[i], cs[(i + 1) % cs.len()]);
                let (pe, ps) = (b.vertex(b.coedge_end(c)).p, b.vertex(b.coedge_start(n)).p);
                if b.coedge_end(c) != b.coedge_start(n) && pe.dist(ps) > tol {
                    out.push(Issue::BrokenLoop(l, c));
                }
            }
        }
    }
    out
}

/// Euler-Poincaré characteristic V - E + F - (L - F) per body (= 2·(S - G)
/// for closed manifold shells). Seams and pole vertices count as usual.
pub fn euler(b: &Body) -> i64 {
    let v = b.vertex_ids().count() as i64;
    let e = b.edge_ids().count() as i64;
    let f = b.face_ids().count() as i64;
    let l: i64 = b.face_ids().map(|f| b.face(f).loops.len() as i64).sum();
    v - e + f - (l - f)
}

/// True when the body is a closed, consistently paired 2-manifold.
pub fn is_closed_manifold(b: &Body) -> bool {
    check(b, 1e-7).iter().all(|i| !matches!(i, Issue::OpenEdge(_) | Issue::NonManifoldEdge(..) | Issue::BadPairing(_)))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::make::tests::box_body;

    #[test]
    fn box_is_valid() {
        let b = box_body();
        assert!(check(&b, 1e-9).is_empty());
        assert!(is_closed_manifold(&b));
        assert_eq!(euler(&b), 2);
    }

    #[test]
    fn detects_open_edges() {
        let mut b = box_body();
        crate::edit::delete_face(&mut b, crate::body::FaceId(0));
        let iss = check(&b, 1e-9);
        assert_eq!(iss.iter().filter(|i| matches!(i, Issue::OpenEdge(_))).count(), 4);
        assert!(!is_closed_manifold(&b));
    }
}
