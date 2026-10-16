//! Independent solid validator. Check logic adapted from brepkit v2.129.15
//! (MIT OR Apache-2.0), crates/check/src/validate/{shell,wire,edge,solid}.rs,
//! re-expressed over ck-topo bodies and ck-geom evaluation.

mod edge;
pub use edge::{face_deviation, vertex_deviation};
mod shell;
mod solid;
mod wire;

use ck_topo::body::{Body, EdgeId, FaceId};

/// One validation finding. `code` is a stable machine id, `entity` a face
/// id (persistent name) or `edge#N[faceA|faceB]`, `detail` human text.
#[derive(Debug, Clone, PartialEq)]
pub struct Issue {
    pub code: &'static str,
    pub entity: String,
    pub detail: String,
}

pub(crate) fn face_ent(b: &Body, f: FaceId) -> String {
    b.face_id_str(f)
}
pub(crate) fn edge_ent(b: &Body, e: EdgeId) -> String {
    let fs: Vec<String> = b.edge_faces(e).into_iter().map(|f| b.face_id_str(f)).collect();
    format!("edge#{}[{}]", e.idx(), fs.join("|"))
}

/// Default cap (mm) on an edge / vertex tolerance: a larger one is an error.
pub const DEFAULT_TOL_CAP: f64 = 0.05;

/// Largest edge or vertex tolerance in the body (0 when every entity is exact).
pub fn tolerance_used(b: &Body) -> f64 {
    let e = b.edge_ids().map(|e| b.edge(e).tol);
    let v = b.vertex_ids().map(|v| b.vertex(v).tol);
    e.chain(v).fold(0.0, f64::max)
}

/// Run every check with model tolerance `tol` (mm). Empty result = valid.
pub fn validate(b: &Body, tol: f64) -> Vec<Issue> {
    validate_cap(b, tol, DEFAULT_TOL_CAP)
}

/// `validate` with an explicit cap on edge / vertex tolerances.
pub fn validate_cap(b: &Body, tol: f64, cap: f64) -> Vec<Issue> {
    let mut out = vec![];
    shell::check(b, &mut out);
    wire::check(b, tol, &mut out);
    edge::check(b, tol, cap, &mut out);
    solid::check(b, &mut out);
    out
}

#[cfg(test)]
mod tests;
