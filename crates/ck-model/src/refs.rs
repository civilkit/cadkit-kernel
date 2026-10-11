//! Reference resolution against the current body: face ids -> faces, planar
//! faces -> sketch/plane frames.
//!
//! Face frames (`{"kind":"face","id":"F2.C1"}` sketch planes and plane-feature
//! bases) are pinned by oracle probes on extruded boxes / prisms:
//! * `z` = the face's OUTWARD normal;
//! * `o` = the world origin projected onto the face plane (`z·(z·p)`);
//! * `x` = `normalize(Y × z)`, except when `|z.y| > 0.99` (bisected between
//!   0.98997 and 0.99002), where `x = X` exactly — NOT projected into the
//!   plane, so near-horizontal faces get a slightly skew frame (oracle:
//!   `z = (0.0994, 0.995, 0)` -> `x = (1,0,0)`);
//! * `y = z × x`, unnormalised (oracle: `y = (0,0,-0.995)` in that case).
//!
//! This reproduces the std frames for axis-aligned faces (front/top/right,
//! and for the opposite normals x is mirrored except on ±Y).
//! Non-planar or unknown faces -> `None` (`missingReference`).

use ck_geom::surface::Surface;
use ck_geom::{Frame, Vec3};
use ck_topo::body::{Body, FaceId};

/// The live face named `id`.
pub fn find_face(body: &Body, id: &str) -> Option<FaceId> {
    body.find_face(id)
}

/// Frame of the planar face `id` (see module docs).
pub fn face_frame(body: &Body, id: &str) -> Option<Frame> {
    let f = find_face(body, id)?;
    let Surface::Plane { frame } = &body.face(f).surface else {
        // GUESS: only analytic planes count as planar for sketching.
        return None;
    };
    let z = body.planar_normal(f);
    let o = z * z.dot(frame.o);
    let x = if z.y.abs() > 0.99 { Vec3::X } else { Vec3::Y.cross(z).normalize() };
    let y = z.cross(x);
    Some(Frame { o, x, y, z })
}
