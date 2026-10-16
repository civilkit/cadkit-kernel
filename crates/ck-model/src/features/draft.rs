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
        } else {
            // a face split into pieces `<id>~k` (six pilot holes) is named by
            // its base id (oracle: large sample F49 on `F32.S.h1`)
            let base = super::lip::base_id(r.as_str());
            for g in body.face_ids().filter(|&g| super::lip::base_id(&body.face_id_str(g)) == base) {
                if !ids.contains(&g) {
                    ids.push(g);
                }
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
    let out = ck_ops::draft::draft(body, fr.o, dir, &ids, angle * 0.017453292519943295, super::TOL)
        .map_err(|e| draft_error(e, 0.0))?;
    br.body = Some(out);
    Ok(())
}

/// The feature error of a failed draft. `offset` is the distance from the neutral plane to where the
/// material starts; a `draftTooLarge` from a closing cone carries `maxHeight` (measured from there).
fn draft_error(e: ck_ops::draft::DraftError, offset: f64) -> FeatureError {
    let fe = FeatureError::new(e.code);
    if let Some(tip) = e.tip {
        let h = tip - offset;
        return fe
            .with("detail", Json::Str(format!("the taper closes to a point at height {h}")))
            .with("maxHeight", Json::Num(h));
    }
    match e.detail {
        Some(d) => fe.with("detail", Json::Str(d)),
        None => fe,
    }
}

/// Distance from the sketch plane to where a drafted extrude starts (0 for a blind extrude).
fn start_offset(o: &ck_topo::make::ExtrudeOpts) -> f64 {
    let (s0, s1) = (o.s0.min(o.s1), o.s0.max(o.s1));
    if s1 <= 1e-12 {
        -s1
    } else {
        s0.max(0.0)
    }
}

/// Largest extrude depth at which a drafted circular profile still has a rim: the taper closes to a point
/// there. `None` unless the profile is one full circle (no holes), the taper goes inward and the extrude
/// runs to one side of the sketch plane.
pub fn tip_depth(profile: &ck_topo::make::Profile, o: &ck_topo::make::ExtrudeOpts) -> Option<f64> {
    use ck_geom::curve2::Curve2;
    if !(o.draft > 0.0) || (o.s0.min(o.s1) < -1e-12 && o.s0.max(o.s1) > 1e-12) {
        return None;
    }
    let [reg] = profile.regions.as_slice() else { return None };
    let [seg] = reg.outer.segs.as_slice() else { return None };
    if !reg.holes.is_empty() || (seg.t1 - seg.t0 - std::f64::consts::TAU).abs() > 1e-9 {
        return None;
    }
    let Curve2::Circle { r, .. } = &seg.curve else { return None };
    let h = r / o.draft.tan() - start_offset(o);
    (h > 0.0).then_some(h)
}

/// A drafted extrude as the oracle builds it: the straight extrude, then the
/// draft operation on every side face about the sketch plane (the pull
/// direction runs from the sketch plane into the material). `o.draft` is the
/// signed angle in radians (negative = `draftOut`).
///
/// A circular profile whose taper closes exactly at the far end gives a cone with one apex. Past that
/// (or when a polygon's edge collapses) the error is `draftTooLarge` with `maxHeight`, the largest depth
/// that still builds.
pub fn drafted_extrude(
    feature: &str,
    profile: &ck_topo::make::Profile,
    o: ck_topo::make::ExtrudeOpts,
) -> Result<Body, FeatureError> {
    let err = match drafted_extrude_once(feature, profile, o) {
        Ok(b) => return Ok(b),
        Err(e) => e,
    };
    if err.code != "draftTooLarge" || err.params.iter().any(|(k, _)| k == "maxHeight") {
        return Err(err);
    }
    // not a closing cone: find the deepest extrude that still builds
    let offset = start_offset(&o);
    let at = |x: f64| {
        let mut q = o;
        if o.s0.max(o.s1) > 1e-12 {
            q.s1 = x;
        } else {
            q.s0 = -x;
        }
        // a body with a sliver edge is not a usable answer: the height must leave every edge visible
        drafted_extrude_once(feature, profile, q).is_ok_and(|b| b.edge_ids().all(|e| b.edge_length(e) > 1e-5))
    };
    let (mut lo, mut hi) = (offset, o.s0.abs().max(o.s1.abs()));
    for _ in 0..48 {
        let mid = 0.5 * (lo + hi);
        if at(mid) {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    let h = lo - offset;
    if !(h > 0.0) {
        return Err(err);
    }
    let orig = err.params.iter().find(|(k, _)| k == "detail").and_then(|(_, v)| v.as_str().map(str::to_string));
    Err(FeatureError::new("draftTooLarge")
        .with(
            "detail",
            Json::Str(format!("the taper collapses the profile at height {h}{}", orig.map(|d| format!(" ({d})")).unwrap_or_default())),
        )
        .with("maxHeight", Json::Num(h)))
}

fn drafted_extrude_once(
    feature: &str,
    profile: &ck_topo::make::Profile,
    o: ck_topo::make::ExtrudeOpts,
) -> Result<Body, FeatureError> {
    use ck_topo::body::Role;
    let fr = profile.frame;
    let mut d = o.dir.unwrap_or(fr.z).normalize();
    if d.dot(fr.z) < 0.0 {
        d = -d;
    }
    let (s0, s1) = (o.s0.min(o.s1), o.s0.max(o.s1));
    let skew = (d - fr.z).len() > 1e-12;
    if skew || (s0 < -1e-12 && s1 > 1e-12) {
        return Err(FeatureError::new("draftUnsupported"));
    }
    let straight = ck_topo::make::extrude(feature, profile, ck_topo::make::ExtrudeOpts { draft: 0.0, ..o })
        .map_err(|e| FeatureError::new(&e))?;
    let sides: Vec<FaceId> =
        straight.face_ids().filter(|&f| matches!(straight.face(f).name.role, Role::Side(_))).collect();
    let pull = if s1 <= 1e-12 { -d } else { d };
    ck_ops::draft::draft(&straight, fr.o, pull, &sides, o.draft, super::TOL).map_err(|e| draft_error(e, start_offset(&o)))
}
