//! `pipeCut {radius, edges, faces}` (ck-model/src/features/pipecut.rs):
//! roll a ball along the picked edges (and the rims of the picked faces) and
//! cut away everything it touches; the body becomes a SURFACE body (ck-ops
//! `pipecut`). Oracle probes (tools/stub_cases.mjs `pipeCut*`):
//!
//! * check order: `radius <= 0` (an empty payload too) -> `invalidValue`;
//!   no edge and no face -> `missingReference`; an unresolved edge / face ->
//!   `missingReference`; refs in more than one body -> `alignPipeCutOneBody`;
//! * the picks may live in the solid or in a surface body; the result
//!   replaces that body and is registered under the feature id;
//! * `pipeCutRadius {detail, max}` (`max` a string, 2 decimals),
//!   `pipeCutNothingLeft {detail}`, `pipeCutFailed {detail}`; on any error the
//!   model is unchanged.

use super::surfaces::{pick_edge, Slot};
use super::Brep;
use crate::regen::{codes, FeatureError, RegenState};
use crate::Feature;
use ck_geom::Vec3;
use ck_json::Json;
use ck_topo::body::EdgeId;

fn missing() -> FeatureError {
    FeatureError::new(codes::MISSING_REFERENCE)
}

pub fn exec_pipecut(br: &mut Brep, state: &mut RegenState, f: &Feature) -> Result<(), FeatureError> {
    let r = state.num(f, "radius", 0.0)?;
    if !(r > 0.0) || !r.is_finite() {
        return Err(FeatureError::new(codes::INVALID_VALUE));
    }
    let picks: Vec<crate::EdgeRef> =
        f.raw.get("edges").and_then(|a| a.as_arr()).map(|a| a.iter().filter_map(crate::EdgeRef::from_json).collect()).unwrap_or_default();
    let faces: Vec<String> = f
        .raw
        .get("faces")
        .and_then(|a| a.as_arr())
        .map(|a| a.iter().filter_map(|x| x.as_str().map(str::to_string)).collect())
        .unwrap_or_default();
    if picks.is_empty() && faces.is_empty() {
        return Err(missing());
    }
    // ---- resolve to (slot, edges); every pick must live in one body
    let mut slot: Option<Slot> = None;
    let mut edges: Vec<EdgeId> = vec![];
    let mut put = |s: Slot| -> Result<(), FeatureError> {
        match slot {
            None => slot = Some(s),
            Some(t) if t != s => return Err(FeatureError::new("alignPipeCutOneBody")),
            _ => {}
        }
        Ok(())
    };
    for p in &picks {
        let at = p.at.map(|a| Vec3::new(a[0], a[1], a[2]));
        let (s, e) = pick_edge(br, &p.faces, at)?;
        put(s)?;
        if !edges.contains(&e) {
            edges.push(e);
        }
    }
    for name in &faces {
        let s = br.slot_of_face(name).ok_or_else(|| br.missing_face(name))?;
        put(s)?;
        let b = br.slot_body(s).ok_or_else(missing)?;
        let fid = b.find_face(name).ok_or_else(|| crate::refs::missing_face([b], name))?;
        for e in b.face_edges(fid) {
            // a seam (both sides the same face) is no edge of the model
            let fs = b.edge_faces(e);
            if fs.len() == 2 && fs[0] == fs[1] {
                continue;
            }
            if !edges.contains(&e) {
                edges.push(e);
            }
        }
    }
    let slot = slot.ok_or_else(missing)?;
    let body = br.slot_body(slot).ok_or_else(missing)?.clone();
    // ---- cut
    let out = ck_ops::pipecut::pipe_cut(&body, &edges, r, super::TOL).map_err(|e| {
        let mut fe = FeatureError::new(e.code);
        if let Some(d) = e.detail {
            fe = fe.with("detail", Json::Str(d));
        }
        if let Some(m) = e.max {
            fe = fe.with("max", Json::Str(format!("{m:.2}")));
        }
        fe
    })?;
    match slot {
        Slot::Solid => br.body = None,
        Slot::Surf(i) => {
            br.surfs.remove(i);
        }
    }
    br.set_surface(&f.id, out);
    Ok(())
}
