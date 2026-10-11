//! `draft` (ck-model, original f1204): tilt faces about a neutral plane.
//!
//! Pinned by oracle probes (tools/brep_cases.mjs `draft*` parts on the
//! 40x20x10 top-plane box):
//! * `neutral` is a plane reference (std / plane feature / planar face); the
//!   pull direction is the plane's `z` (times -1 for `flip`), the angle is in
//!   degrees; a body is required (`nothingToCut`), the neutral must resolve
//!   (`missingReference`), `faces` must be non-empty (`noFaces`) and at
//!   least one of them must resolve (`missingReference`);
//! * faces tangent to a drafted face (normals closer than 0.999999 at a
//!   shared edge) are drafted with it (the list grows while iterated);
//! * ck-ops errors carry `params.detail` (none for `invalidValue`).

use super::pattern::plane_frame;
use super::{Brep, FeatureKind};
use crate::regen::{FeatureError, RegenState};
use crate::Feature;
use ck_json::Json;
use ck_topo::body::{Body, FaceId};

/// Tangent-propagation threshold (cos of the angle between outward normals).
const TANGENT_DOT: f64 = 0.999999;

/// Faces smoothly continuing a drafted face join the draft.
pub fn grow_tangent(b: &Body, ids: &mut Vec<FaceId>) {
    let mut i = 0;
    while i < ids.len() {
        let f = ids[i];
        for e in b.face_edges(f) {
            let m = b.edge_mid(e);
            for g in b.edge_faces(e) {
                if g == f || ids.contains(&g) {
                    continue;
                }
                let n = |h: FaceId| {
                    let (u, v) = b.face(h).surface.closest_param(m);
                    b.outward_normal(h, u, v)
                };
                let (nf, ng) = (n(f), n(g));
                if nf.x * ng.x + nf.y * ng.y + nf.z * ng.z > TANGENT_DOT {
                    ids.push(g);
                }
            }
        }
        i += 1;
    }
}

pub fn exec_draft(br: &mut Brep, state: &mut RegenState, f: &Feature) -> Result<(), FeatureError> {
    let FeatureKind::Draft { neutral, faces, flip, .. } = &f.kind else { unreachable!() };
    let angle = state.num(f, "angle", 0.0)?;
    let missing = || FeatureError::new("missingReference");
    let Some(body) = br.body.as_ref() else { return Err(FeatureError::new("nothingToCut")) };
    let fr = neutral.as_ref().and_then(|r| plane_frame(br, state, r)).ok_or_else(missing)?;
    if faces.is_empty() {
        return Err(FeatureError::new("noFaces"));
    }
    let mut ids: Vec<FaceId> = vec![];
    for r in faces {
        if let Some(g) = body.find_face(r.as_str()) {
            if !ids.contains(&g) {
                ids.push(g);
            }
        }
    }
    if ids.is_empty() {
        return Err(missing());
    }
    grow_tangent(body, &mut ids);
    // a face neutral pulls INTO its material (against the outward normal)
    let face_sign = if matches!(neutral, Some(crate::GeomRef::Face(_))) { -1.0 } else { 1.0 };
    let sg = (if *flip { -1.0 } else { 1.0 }) * face_sign;
    let dir = ck_geom::Vec3::new(sg * fr.z.x, sg * fr.z.y, sg * fr.z.z);
    let out = ck_ops::draft::draft(body, fr.o, dir, &ids, angle * 0.017453292519943295, super::TOL).map_err(|e| {
        let fe = FeatureError::new(e.code);
        match e.detail {
            Some(d) => fe.with("detail", Json::Str(d)),
            None => fe,
        }
    })?;
    br.body = Some(out);
    Ok(())
}
