// Adapted from brepkit v2.129.15 (MIT OR Apache-2.0), crates/check/src/validate/edge.rs
//! Vertex-on-curve and edge-on-surface (pcurve vs 3D curve) deviation.

use crate::{edge_ent, face_ent, Issue};
use ck_topo::body::Body;

/// Largest distance from the edge curve to the faces that use it, over `n`+1
/// samples per coedge (pcurve evaluation when the coedge has one, else
/// closest-point projection).
pub fn face_deviation(b: &Body, e: ck_topo::body::EdgeId, n: usize) -> f64 {
    let ed = b.edge(e);
    let mut dev: f64 = 0.0;
    for c in b.edge_coedges(e) {
        let surf = &b.face(b.coedge_face(c)).surface;
        let mut prev = None;
        for k in 0..=n {
            let t = ed.t0 + (ed.t1 - ed.t0) * k as f64 / n as f64;
            let uv = b.coedge_uv(c, t, prev);
            prev = Some(uv);
            dev = dev.max(surf.eval(uv.x, uv.y).dist(ed.curve.eval(t)));
        }
    }
    dev
}

/// Distance of the edge's two vertices to its curve ends.
pub fn vertex_deviation(b: &Body, e: ck_topo::body::EdgeId) -> [f64; 2] {
    let ed = b.edge(e);
    [b.vertex(ed.v0).p.dist(ed.curve.eval(ed.t0)), b.vertex(ed.v1).p.dist(ed.curve.eval(ed.t1))]
}

pub fn check(b: &Body, tol: f64, cap: f64, out: &mut Vec<Issue>) {
    for e in b.edge_ids() {
        let ed = b.edge(e);
        if !(ed.t1 > ed.t0) {
            out.push(Issue { code: "EDGE_RANGE", entity: edge_ent(b, e), detail: format!("t0={} >= t1={}", ed.t0, ed.t1) });
            continue;
        }
        let scale = 1.0 + ed.curve.eval(ed.t0).len();
        // a tolerant edge (set by ck_heal) may deviate up to its own tolerance
        if ed.tol > cap {
            out.push(Issue { code: "EDGE_TOLERANCE_EXCEEDED", entity: edge_ent(b, e), detail: format!("edge tolerance {:.3e} exceeds the cap {cap:.3e}", ed.tol) });
        }
        let lim = (tol * scale * 10.0).max(ed.tol);
        for (v, t, w) in [(ed.v0, ed.t0, "start"), (ed.v1, ed.t1, "end")] {
            let vt = b.vertex(v).tol;
            if vt > cap {
                out.push(Issue { code: "VERTEX_TOLERANCE_EXCEEDED", entity: edge_ent(b, e), detail: format!("vertex tolerance {vt:.3e} exceeds the cap {cap:.3e}") });
            }
            let d = b.vertex(v).p.dist(ed.curve.eval(t));
            if d > lim.max(vt) {
                out.push(Issue { code: "VERTEX_OFF_CURVE", entity: edge_ent(b, e), detail: format!("{w} vertex {d:.3e} off curve") });
            }
        }
        if b.edge_length(e) < tol {
            out.push(Issue { code: "EDGE_DEGENERATE", entity: edge_ent(b, e), detail: "zero-length edge".into() });
        }
        for c in b.edge_coedges(e) {
            let f = b.coedge_face(c);
            let surf = &b.face(f).surface;
            let mut dev: f64 = 0.0;
            let mut prev = None;
            for k in 0..=8 {
                let t = ed.t0 + (ed.t1 - ed.t0) * k as f64 / 8.0;
                let uv = b.coedge_uv(c, t, prev);
                prev = Some(uv);
                dev = dev.max(surf.eval(uv.x, uv.y).dist(ed.curve.eval(t)));
            }
            if dev > lim.max(1e-6 * scale).max(ed.tol) {
                let what = if b.coedge(c).pcurve.is_some() { "pcurve" } else { "projection" };
                out.push(Issue {
                    code: "EDGE_OFF_SURFACE",
                    entity: face_ent(b, f),
                    detail: format!("{} {what} deviates {dev:.3e}", edge_ent(b, e)),
                });
            }
        }
    }
}
