//! `moveBody`: rigid move (and optional copy) of one or more bodies.
//!
//! Wire shape (GUI handles, docs/spec-agent-interface.md "Direct manipulation"):
//!
//! ```json
//! {"type":"moveBody","id":"F9","bodies":[{"face":"F2.C1"}],
//!  "translate":[10,0,0],
//!  "rotate":{"axis":[0,0,1],"origin":[0,0,0],"angle":30},
//!  "copy":false}
//! ```
//!
//! * `bodies` (also `target`, and `body` when it is an object or an array and `copy` is not true; an object, a
//!   string or an array of either):
//!   `{body: "<name>"}` selects every lump with that body name (none: `missingReference {name, nearest}`),
//!   `{face: "<face id>"}` (or the bare string) selects the lump holding that face,
//!   `{feature: "<feature id>"}` (or a bare id that is not a face name) selects every
//!   lump holding a face made by that feature (`F17.*`, instance and patterned copies
//!   `F17.*@...` too; several entries add up, none found: `missingReference`),
//!   `{surface: "<feature id>"}` a surface body. Missing or empty: every lump of the
//!   solid body.
//! * Motion: rotation first (about `rotate.axis` through `rotate.origin`, `angle` in
//!   degrees, `axis` a vector, `"x"|"y"|"z"` or `{kind:"axis",axis:"x"}`), then the
//!   translation. `translate:[x,y,z]` (numbers or `"=expr"`); the legacy `dx/dy/dz`
//!   and `axis` + `angle` (world axis through the origin) are read too. No motion is
//!   a no-op.
//! * `copy: true` leaves the original and appends a moved copy whose face names carry
//!   the instance suffix `@<id>.1`; otherwise face names (hence references) are
//!   unchanged: the entities are moved in place.
//! * `body: "<name>"` (with `copy: true`) names the copies; the original keeps its name. A name a lump already has is
//!   `bodyNameConflict {name, features}`. Without `copy` a string `body` keeps its older meaning, a selection
//!   (`bodies` alias), so existing documents read as before.
//! * Errors: `nothingToCut` (no body), `missingReference`, `invalidValue`
//!   (non-finite numbers / zero rotation axis, with `detail`).

use super::surfaces::Slot;
use super::Brep;
use crate::regen::{codes, FeatureError, RegenState};
use crate::Feature;
use ck_core::xform::Xform;
use ck_geom::Vec3;
use ck_json::Json;
use ck_topo::body::{transform_surface, Body, FaceId};

fn missing() -> FeatureError {
    FeatureError::new(codes::MISSING_REFERENCE)
}
fn invalid(detail: &str) -> FeatureError {
    FeatureError::new(codes::INVALID_VALUE).with("detail", Json::Str(detail.into()))
}

fn num(state: &RegenState, j: Option<&Json>) -> Result<f64, FeatureError> {
    crate::expr::eval_field(j, &state.vars, 0.0).map_err(FeatureError::new)
}

pub(super) fn vec3(state: &RegenState, j: Option<&Json>) -> Result<Option<Vec3>, FeatureError> {
    let Some(a) = j.and_then(|a| a.as_arr()) else { return Ok(None) };
    if a.len() != 3 {
        return Err(invalid("a vector needs three numbers"));
    }
    let v = Vec3::new(num(state, a.first())?, num(state, a.get(1))?, num(state, a.get(2))?);
    if !(v.x.is_finite() && v.y.is_finite() && v.z.is_finite()) {
        return Err(invalid("non-finite vector"));
    }
    Ok(Some(v))
}

fn axis_vec(state: &RegenState, j: &Json) -> Result<Option<Vec3>, FeatureError> {
    if let Some(v) = vec3(state, Some(j))? {
        return Ok(Some(v));
    }
    let name = j.as_str().or_else(|| j.get_str("axis"));
    Ok(match name {
        Some("x") => Some(Vec3::X),
        Some("y") => Some(Vec3::Y),
        Some("z") => Some(Vec3::Z),
        _ => None,
    })
}

/// The rigid motion the feature asks for (`None` = identity).
pub fn motion(state: &RegenState, f: &Feature) -> Result<Option<Xform>, FeatureError> {
    let raw = &f.raw;
    let mut rot: Option<Xform> = None;
    let mut spec: Option<(Option<&Json>, Option<Vec3>, Option<&Json>)> = None;
    if let Some(r) = raw.get("rotate").filter(|r| r.as_obj().is_some()) {
        spec = Some((r.get("axis"), vec3(state, r.get("origin"))?, r.get("angle")));
    } else if let Some(ax) = raw.get("axis").filter(|a| !matches!(a, Json::Null)) {
        spec = Some((Some(ax), None, raw.get("angle")));
    }
    if let Some((ax, origin, ang)) = spec {
        let angle = num(state, ang)?;
        if !angle.is_finite() {
            return Err(invalid("non-finite angle"));
        }
        if angle != 0.0 {
            let axis = match ax {
                Some(a) => axis_vec(state, a)?.ok_or_else(|| invalid("unknown rotation axis"))?,
                None => return Err(invalid("rotation needs an axis")),
            };
            if !(axis.len() > 1e-12) {
                return Err(invalid("zero rotation axis"));
            }
            rot = Some(Xform::rotation(origin.unwrap_or(Vec3::ZERO), axis.normalize(), angle.to_radians()));
        }
    }
    let t = match vec3(state, raw.get("translate"))? {
        Some(t) => t,
        None => Vec3::new(num(state, raw.get("dx"))?, num(state, raw.get("dy"))?, num(state, raw.get("dz"))?),
    };
    if !(t.x.is_finite() && t.y.is_finite() && t.z.is_finite()) {
        return Err(invalid("non-finite translation"));
    }
    let tr = (t.len() > 0.0).then(|| Xform::translation(t));
    Ok(match (rot, tr) {
        (None, None) => None,
        (Some(r), None) => Some(r),
        (None, Some(t)) => Some(t),
        (Some(r), Some(t)) => Some(t * r),
    })
}

/// Move the listed faces' entities (vertices, edges, surfaces) in place. The
/// faces must form whole components (closed lumps) so no entity is shared with
/// a face that stays.
fn move_faces(b: &mut Body, faces: &[FaceId], x: &Xform) {
    let mut vs = vec![false; b.vertices.len()];
    let mut es = vec![false; b.edges.len()];
    for &f in faces {
        for &l in &b.face(f).loops.clone() {
            for &c in &b.lp(l).coedges.clone() {
                let e = b.coedge(c).edge;
                if !es[e.idx()] {
                    es[e.idx()] = true;
                    let ed = b.edge(e);
                    vs[ed.v0.idx()] = true;
                    vs[ed.v1.idx()] = true;
                }
            }
        }
    }
    for (i, v) in b.vertices.iter_mut().enumerate() {
        if vs[i] {
            v.p = x.apply_point(v.p);
        }
    }
    for (i, e) in b.edges.iter_mut().enumerate() {
        if es[i] {
            e.curve = e.curve.transform(|p| x.apply_point(p), |d| x.apply_vec(d));
        }
    }
    for &f in faces {
        let s = transform_surface(&b.face(f).surface, x);
        b.faces[f.idx()].surface = s;
    }
}

fn entries(raw: &Json) -> Vec<&Json> {
    let copy = crate::sketch::jtruthy(raw.get("copy"));
    for k in ["bodies", "body", "target"] {
        // with `copy: true` a string `body` is the name of the copies, not a selection
        if k == "body" && copy && matches!(raw.get(k), Some(Json::Str(_))) {
            continue;
        }
        match raw.get(k) {
            Some(Json::Arr(a)) => return a.iter().collect(),
            Some(j @ (Json::Str(_) | Json::Obj(_))) => return vec![j],
            _ => {}
        }
    }
    vec![]
}

/// The feature that made a face name (`"F17.S.e1~1@F4.2"` -> `F17`): the id up to
/// the first `.` (face ids always carry one; a bare feature id does not).
fn owner_of(name: &str) -> &str {
    name.split('.').next().unwrap_or(name)
}

/// Does this face belong to feature `fid`? Its own name's feature, any absorbed
/// alias's, or an instance level (`@F17.1` = a pattern / insert copy of `F17`).
fn face_has_feature(b: &Body, f: FaceId, fid: &str) -> bool {
    let face = b.face(f);
    let own = |n: &str| owner_of(n) == fid || n.split(|c| c == '.' || c == '@').any(|seg| seg == fid);
    own(&face.name.id()) || face.absorbed.iter().any(|a| own(a))
}

/// Faces of the solid body made by `fid` (direct names and instance copies).
fn faces_of_feature(b: &Body, fid: &str) -> Option<Vec<FaceId>> {
    let fs: Vec<FaceId> = b.face_ids().filter(|&f| face_has_feature(b, f, fid)).collect();
    (!fs.is_empty()).then_some(fs)
}

pub fn exec_move_body(br: &mut Brep, state: &mut RegenState, f: &Feature) -> Result<(), FeatureError> {
    if br.body.is_none() && br.surfs.is_empty() {
        return Err(FeatureError::new(codes::NOTHING_TO_CUT));
    }
    let copy = crate::sketch::jtruthy(f.raw.get("copy"));
    let copy_name = if copy { super::bodies::body_name(&f.raw)? } else { None };
    if let Some(n) = &copy_name {
        super::bodies::check_free(br.body.as_ref(), n, &f.id, &[])?;
    }
    let x = motion(state, f)?;
    // ---- resolve the selection
    let mut solid_faces: Vec<FaceId> = vec![];
    let mut surf_hit: Vec<usize> = vec![];
    let sel = entries(&f.raw);
    for r in &sel {
        // `{face}` / a bare string that names a face; a bare id without `.`
        // falls through to the feature selection below.
        let face = r.get_str("face").or_else(|| r.as_str().filter(|s| s.contains('.')));
        if let Some(name) = r.get_str("body") {
            // `{body: "<name>"}`: every lump with that body name
            let b = br.body.as_ref();
            let lumps = super::bodies::lumps_named(b, name)?;
            let b = b.ok_or_else(missing)?;
            for l in lumps {
                solid_faces.extend(ck_topo::mass::lump_faces(b, l));
            }
        } else if let Some(face) = face {
            match br.slot_of_face(face).ok_or_else(missing)? {
                Slot::Surf(i) => surf_hit.push(i),
                Slot::Solid => {
                    let b = br.body.as_ref().ok_or_else(missing)?;
                    let fid = b.find_face(face).ok_or_else(missing)?;
                    let comp = ck_topo::region::components(b).into_iter().find(|c| c.contains(&fid)).ok_or_else(missing)?;
                    solid_faces.extend(comp);
                }
            }
        } else if let Some(s) = r.get_str("surface") {
            surf_hit.push(br.surface_ref(s).ok_or_else(missing)?);
        } else if let Some(fid) = r.get_str("feature").or_else(|| r.as_str()) {
            // `{feature: "F17"}` / a bare feature id: every lump holding a face
            // made by that feature (`F17.*`, instance / patterned copies too)
            let b = br.body.as_ref().ok_or_else(missing)?;
            let hit = faces_of_feature(b, fid).ok_or_else(missing)?;
            let mut lumps: Vec<ck_topo::body::LumpId> = hit.iter().map(|&g| b.face_lump(g)).collect();
            lumps.sort_unstable();
            lumps.dedup();
            for l in lumps {
                solid_faces.extend(ck_topo::mass::lump_faces(b, l));
            }
        } else {
            return Err(missing());
        }
    }
    if sel.is_empty() {
        if let Some(b) = &br.body {
            solid_faces = b.face_ids().collect();
        }
    }
    solid_faces.sort();
    solid_faces.dedup();
    surf_hit.sort_unstable();
    surf_hit.dedup();
    // a copy with no motion (a copy in place, to be named and worked on separately) still makes the copy
    let x = match x { Some(x) => x, None if copy => Xform::IDENTITY, None => return Ok(()) };
    // ---- apply
    if !solid_faces.is_empty() {
        if copy {
            let b = br.body.as_ref().unwrap();
            let mut c = ck_ops::boolean::assemble::rebuild_in_order(b, &solid_faces);
            ck_topo::region::rebuild_lumps(&mut c);
            c.push_instance(&format!("{}.1", f.id));
            c.transform(&x);
            br.append_body(&c);
            if let (Some(n), Some(b)) = (&copy_name, br.body.as_mut()) {
                super::bodies::stamp_origin(b, &f.id, n);
            }
        } else if let Some(b) = br.body.as_mut() {
            move_faces(b, &solid_faces, &x);
        }
    }
    for i in surf_hit {
        if copy {
            let mut c = br.surfs[i].1.clone();
            c.push_instance(&format!("{}.1", f.id));
            c.transform(&x);
            let id = format!("{}.{}", f.id, br.surfs[i].0);
            br.surfs.push((id, c));
        } else {
            br.surfs[i].1.transform(&x);
        }
    }
    Ok(())
}
