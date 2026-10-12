// Adapted from brepkit v2.129.15 (MIT OR Apache-2.0), crates/check/src/validate/edge.rs
//! Vertex-on-curve and edge-on-surface (pcurve vs 3D curve) deviation.

use crate::{edge_ent, face_ent, Issue};
use ck_topo::body::Body;

pub fn check(b: &Body, tol: f64, out: &mut Vec<Issue>) {
    for e in b.edge_ids() {
        let ed = b.edge(e);
        if !(ed.t1 > ed.t0) {
            out.push(Issue { code: "EDGE_RANGE", entity: edge_ent(b, e), detail: format!("t0={} >= t1={}", ed.t0, ed.t1) });
            continue;
        }
        let scale = 1.0 + ed.curve.eval(ed.t0).len();
        let lim = tol * scale * 10.0;
        for (v, t, w) in [(ed.v0, ed.t0, "start"), (ed.v1, ed.t1, "end")] {
            let d = b.vertex(v).p.dist(ed.curve.eval(t));
            if d > lim {
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
            if dev > lim.max(1e-6 * scale) {
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
