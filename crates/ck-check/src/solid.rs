// Adapted from brepkit v2.129.15 (MIT OR Apache-2.0), crates/check/src/validate/solid.rs
//! Per-solid checks. A body may hold several solids (one lump each: a
//! disjoint or point-contact union, a cut that splits a part, a multi-solid
//! STEP file). Every lump is judged on its own:
//! - its signed volume must be positive (outward normals);
//! - it holds exactly one outer shell: closed components with positive
//!   volume beyond the first are separate solids that were left in the same
//!   lump (they would be mis-counted, exported as one solid and shadow each
//!   other's voids). Negative closed components are its internal voids.

use crate::Issue;
use ck_topo::body::{Body, FaceId};
use ck_topo::mass::{lump_faces, signed_volume};

pub fn check(b: &Body, out: &mut Vec<Issue>) {
    let comps = ck_topo::region::components(b);
    for l in b.lump_ids() {
        let fs = lump_faces(b, l);
        if fs.is_empty() {
            continue;
        }
        let ent = format!("lump#{}", b.lump_index(l));
        let v = signed_volume(b, &fs);
        if !(v > 0.0) {
            out.push(Issue { code: "VOLUME_NONPOSITIVE", entity: ent.clone(), detail: format!("signed volume {v:.6e} (inside-out or degenerate)") });
            continue;
        }
        // closed components of this lump with a positive volume = outer shells
        let mut outers = 0;
        for c in comps.iter().filter(|c| b.face_lump(c[0]) == l) {
            if c.len() < 2 || !closed(b, c) {
                continue;
            }
            if signed_volume(b, c) > 0.0 {
                outers += 1;
            }
        }
        if outers > 1 {
            out.push(Issue { code: "SOLID_MULTIPLE_OUTERS", entity: ent, detail: format!("{outers} outer shells in one solid") });
        }
    }
}

/// Every edge of the faces is used by exactly two coedges within them.
fn closed(b: &Body, faces: &[FaceId]) -> bool {
    faces.iter().all(|&f| b.face_edges(f).into_iter().all(|e| b.edge_coedges(e).len() == 2))
}
