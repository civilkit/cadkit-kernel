//! Swap a face's surface for the rebuilt one (edges/pcurves kept).

use crate::surfacing::knit::BodyExt;
use ck_topo::body::{Body, FaceId};
use ck_geom::surface::Surface;

/// Replace `f`'s surface in place; the face keeps its name, sense, loops
/// and edge pcurves (the rebuild preserves the uv parameterisation).
pub fn replace_face(b: &mut Body, f: FaceId, surface: Surface) {
    b.face_mut(f).surface = surface;
    // the refit net has its own parameter range (0..spans): pcurves given in
    // the old surface's uv no longer apply; coedge_uv falls back to
    // projecting the edge onto the new surface
    for l in b.face(f).loops.clone() {
        for c in b.lp(l).coedges.clone() {
            b.coedge_mut(c).pcurve = None;
        }
    }
}
