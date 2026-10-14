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
//! * `bodies` (also `body` / `target`; an object, a string or an array of either):
//!   `{face: "<face id>"}` (or the bare string) selects the lump holding that face,
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
    for k in ["bodies", "body", "target"] {
        match raw.get(k) {
            Some(Json::Arr(a)) => return a.iter().collect(),
            Some(j @ (Json::Str(_) | Json::Obj(_))) => return vec![j],
            _ => {}
        }
    }
    vec![]
}

pub fn exec_move_body(br: &mut Brep, state: &mut RegenState, f: &Feature) -> Result<(), FeatureError> {
    if br.body.is_none() && br.surfs.is_empty() {
        return Err(FeatureError::new(codes::NOTHING_TO_CUT));
    }
    let copy = crate::sketch::jtruthy(f.raw.get("copy"));
    let x = motion(state, f)?;
    // ---- resolve the selection
    let mut solid_faces: Vec<FaceId> = vec![];
    let mut surf_hit: Vec<usize> = vec![];
    let sel = entries(&f.raw);
    for r in &sel {
        let face = r.as_str().or_else(|| r.get_str("face"));
        if let Some(face) = face {
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
    let Some(x) = x else { return Ok(()) };
    // ---- apply
    if !solid_faces.is_empty() {
        if copy {
            let b = br.body.as_ref().unwrap();
            let mut c = ck_ops::boolean::assemble::rebuild_in_order(b, &solid_faces);
            ck_topo::region::rebuild_lumps(&mut c);
            c.push_instance(&format!("{}.1", f.id));
            c.transform(&x);
            br.append_body(&c);
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
