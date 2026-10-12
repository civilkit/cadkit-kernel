// Adapted from brepkit v2.129.15 (MIT OR Apache-2.0), crates/check/src/validate/solid.rs
//! Orientation: each lump's signed volume must be positive (outward normals).

use crate::Issue;
use ck_topo::body::Body;
use ck_topo::mass::{lump_faces, signed_volume};

pub fn check(b: &Body, out: &mut Vec<Issue>) {
    for l in b.lump_ids() {
        let fs = lump_faces(b, l);
        if fs.is_empty() {
            continue;
        }
        let v = signed_volume(b, &fs);
        if !(v > 0.0) {
            out.push(Issue {
                code: "VOLUME_NONPOSITIVE",
                entity: format!("lump#{}", b.lump_index(l)),
                detail: format!("signed volume {v:.6e} (inside-out or degenerate)"),
            });
        }
    }
}
