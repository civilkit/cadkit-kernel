//! `surfaceOffset` (Offset Surface): offset faces of the solid, or of a surface body, by a distance into a new surface body.
//!
//! ```json
//! {"type":"surfaceOffset","id":"F5","faces":["F2.C1","F2.S.e1"],"distance":2}
//! {"type":"surfaceOffset","id":"F6","surface":"F4","distance":-1.5,"keep":false}
//! ```
//!
//! * `faces` (names) or `surface` (a surface feature id or any face of a surface body, all its faces): what to offset.
//!   Faces of the solid and faces of one surface body cannot be mixed (`invalidValue` `faces`).
//! * `distance` (mm, default 1): along the face normal, outward for a face of a solid, the surface normal for a sheet.
//!   Positive grows the solid, negative moves into it. `flip` reverses the sign. Zero makes a plain copy.
//! * `keep` (default true): the source stays. `keep: false` takes a surface body away once it is offset; a solid cannot be taken
//!   away (`invalidValue` `keep`).
//! * The result is a surface body registered under the feature id, built by the face offset of the shell machinery
//!   (`ck-ops` offset): planes slide, cylinders, cones, spheres and tori change radius, and neighbouring faces of the pick meet in
//!   re-solved edges. Its faces are named `<id>.O<k>` in pick order (`F5.O1`, `F5.O2`).
//! * Only analytic faces (plane, cylinder, cone, sphere, torus) are offset. Any other surface is `offsetUnsupported {face,
//!   detail}`, not an approximation. A distance that reaches a centre of curvature is `offsetDegenerate` with the
//!   `limit`, `minRadius` and `maxThickness` params of the shell errors.
//! * Errors: `missingReference` (no `faces` / `surface`, unknown name), `invalidValue {field, detail}` (`distance` not finite,
//!   mixed sources, `keep`), `offsetUnsupported`, `offsetDegenerate`, `offsetFailed {detail}`.

use super::surfaces::Slot;
use super::Brep;
use crate::regen::{codes, FeatureError, RegenState};
use crate::Feature;
use ck_geom::surface::Surface;
use ck_json::Json;
use ck_topo::body::{FaceId, FaceName, Role};

fn invalid(field: &str, detail: &str) -> FeatureError {
    FeatureError::new(codes::INVALID_VALUE).with("field", Json::Str(field.into())).with("detail", Json::Str(detail.into()))
}

fn missing() -> FeatureError {
    FeatureError::new(codes::MISSING_REFERENCE)
}

pub fn exec_surface_offset(br: &mut Brep, state: &mut RegenState, f: &Feature) -> Result<(), FeatureError> {
    let mut d = state.num(f, "distance", 1.0)?;
    if !d.is_finite() {
        return Err(invalid("distance", "distance is a finite number"));
    }
    if crate::sketch::jtruthy(f.raw.get("flip")) {
        d = -d;
    }
    let keep = !matches!(f.raw.get("keep"), Some(k) if !crate::sketch::jtruthy(Some(k)));

    // what is picked: (slot, face ids in pick order)
    let names: Vec<&str> = f.raw.get("faces").and_then(|a| a.as_arr()).map(|a| a.iter().filter_map(|x| x.as_str()).collect()).unwrap_or_default();
    let (slot, ids) = if !names.is_empty() {
        let slot = br.slot_of_face(names[0]).ok_or_else(|| br.missing_face(names[0]))?;
        let body = br.slot_body(slot).ok_or_else(missing)?;
        let mut ids: Vec<FaceId> = vec![];
        for n in &names {
            if br.slot_of_face(n).ok_or_else(|| br.missing_face(n))? != slot {
                return Err(invalid("faces", "the faces belong to different bodies: offset one body at a time"));
            }
            let id = body.find_face(n).ok_or_else(|| br.missing_face(n))?;
            if !ids.contains(&id) {
                ids.push(id);
            }
        }
        (slot, ids)
    } else if let Some(r) = f.raw.get("surface").and_then(|s| s.as_str()) {
        let i = br.surface_ref(r).ok_or_else(missing)?;
        (Slot::Surf(i), br.surfs[i].1.face_ids().collect())
    } else {
        return Err(missing());
    };
    if matches!(slot, Slot::Solid) && !keep {
        return Err(invalid("keep", "a solid stays: only a surface body can be replaced by its offset"));
    }
    let body = br.slot_body(slot).ok_or_else(missing)?.clone();
    if ids.is_empty() {
        return Err(missing());
    }
    for &id in &ids {
        if !matches!(body.face(id).surface, Surface::Plane { .. } | Surface::Cylinder { .. } | Surface::Cone { .. } | Surface::Sphere { .. } | Surface::Torus { .. }) {
            return Err(FeatureError::new("offsetUnsupported")
                .with("face", Json::Str(body.face_id_str(id)))
                .with("detail", Json::Str("surface type".into()))
                .with("hint", Json::Str("only plane, cylinder, cone, sphere and torus faces can be offset".into())));
        }
    }

    // the picked faces as a sheet, offset face by face (the pick order of the sheet is the body order of the faces)
    let sheet = ck_ops::surfacing::sheet::sheet(&body, &ids);
    let mut out = ck_ops::offset::move_faces_body(&sheet, &|_| ck_ops::offset::FaceMove::Offset(d)).map_err(|e| {
        let detail = e.detail.clone().unwrap_or_default();
        let code = if e.code == "offsetDegenerate" || e.code == "offsetTooLarge" { "offsetDegenerate" } else if e.code == "offsetUnsupported" { "offsetUnsupported" } else { "offsetFailed" };
        let mut fe = FeatureError::new(code).with("detail", Json::Str(detail));
        if let Some(l) = &e.limit {
            fe = super::offset_limit_params(fe, l);
        }
        fe
    })?;
    for e in out.edge_ids() {
        let ed = out.edge(e);
        let p = ed.curve.eval(0.5 * (ed.t0 + ed.t1));
        if !(p.x.is_finite() && p.y.is_finite() && p.z.is_finite()) || !(ed.t1 - ed.t0 > 0.0) {
            return Err(FeatureError::new("offsetFailed").with("detail", Json::Str("an edge degenerated".into())));
        }
    }
    let faces: Vec<FaceId> = out.face_ids().collect();
    // sheet() keeps body order: name by the position the source face has among the picks, in pick order
    let mut order: Vec<FaceId> = ids.clone();
    order.sort_by_key(|x| x.idx());
    for (k, nf) in faces.iter().enumerate() {
        let pick = ids.iter().position(|x| *x == order[k]).unwrap_or(k);
        out.faces[nf.idx()].name = FaceName { feature: f.id.clone(), role: Role::Other(format!("O{}", pick + 1)), region: None, suffix: String::new() };
        out.faces[nf.idx()].absorbed.clear();
    }
    if let Slot::Surf(i) = slot {
        if !keep {
            br.drop_surfaces(vec![i]);
        }
    }
    br.set_surface(&f.id, out);
    Ok(())
}
