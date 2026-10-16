//! Tolerant geometry: instead of moving an exporter's edge curve off the
//! faces it was fitted to, record how far it is from them. An edge whose
//! curve lies farther from a face than the model tolerance gets
//! `tol = measured max deviation`; a vertex farther from its edge curve end
//! gets the same treatment. Deviations above the cap are left alone (the
//! validator then reports them). Entities that are already within the model
//! tolerance are never touched, so a valid body is a fixed point.

use crate::Fixed;
use ck_check::{face_deviation, vertex_deviation};
use ck_topo::body::Body;

/// Margin on a measured deviation so a re-measurement at other samples
/// still passes.
const MARGIN: f64 = 1.05;

/// Returns the number of entities given a tolerance and the number left
/// off-tolerance because the deviation exceeds the cap.
pub fn set_tolerances(b: &mut Body, tol: f64, cap: f64, out: &mut Vec<Fixed>) -> (usize, usize) {
    let (mut n, mut over) = (0, 0);
    for e in b.edge_ids().collect::<Vec<_>>() {
        let ed = b.edge(e);
        if !(ed.t1 > ed.t0) {
            continue;
        }
        let scale = 1.0 + ed.curve.eval(ed.t0).len();
        let lim = tol * scale * 10.0;
        // the validator's 8 samples first; finer only for the edges that need it
        let mut dev = face_deviation(b, e, 8);
        if dev > lim.max(1e-6 * scale).max(b.edge(e).tol) {
            dev = dev.max(face_deviation(b, e, 32));
        }
        if dev > lim.max(1e-6 * scale).max(b.edge(e).tol) && dev * MARGIN <= cap {
            b.edges[e.idx()].tol = dev * MARGIN;
            n += 1;
            out.push(Fixed { code: "EDGE_TOLERANCE", entity: format!("edge#{}", e.idx()), detail: format!("edge tolerance set to {:.3e}", dev * MARGIN) });
        } else if dev > lim.max(1e-6 * scale).max(b.edge(e).tol) {
            over += 1;
        }
        let ed = b.edge(e);
        let (v0, v1) = (ed.v0, ed.v1);
        let lim = lim.max(ed.tol);
        for (v, d) in [v0, v1].into_iter().zip(vertex_deviation(b, e)) {
            if d > lim.max(b.vertex(v).tol) && d * MARGIN <= cap {
                b.vertices[v.idx()].tol = d * MARGIN;
                n += 1;
                out.push(Fixed { code: "VERTEX_TOLERANCE", entity: format!("vertex#{}", v.idx()), detail: format!("vertex tolerance set to {:.3e}", d * MARGIN) });
            } else if d > lim.max(b.vertex(v).tol) {
                over += 1;
            }
        }
    }
    (n, over)
}
