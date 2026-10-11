//! Swap a face's surface for the rebuilt one (edges/pcurves kept).

use crate::surfacing::knit::BodyExt;
use ck_topo::body::{Body, FaceId};
use ck_geom::surface::Surface;

/// Replace `f`'s surface in place; the face keeps its name, sense, loops
/// and edge pcurves (the rebuild preserves the uv parameterisation).
pub fn replace_face(b: &mut Body, f: FaceId, surface: Surface) {
    b.face_mut(f).surface = surface;
}
