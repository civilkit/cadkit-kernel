//! `meshToSolid`: a triangle mesh held by the part, converted to a solid (`ck-mesh`, the `meshToSolid` request) on every
//! regeneration. The mesh is the feature's input and lives in the part's top-level `blobs` table, as the STEP of
//! `importStep` and the part file of `importBody` do; the converted body is cached (`built_cache`) so that reopening a
//! document does not convert again.
//!
//! ```text
//! {"type":"meshToSolid","id":"F1","name":"Bracket scan","blob":"<sha-256 hex>",
//!  "opts":{"close":true,"weldTol":0.001},"replace":false,"source":{"name":"bracket.stl","format":"stl"}}
//! ```
//!
//! The feature is not an importer: it is format-agnostic. The app reads STL / OBJ / GLB / 3MF / SketchUp and hands the
//! kernel triangles, packed into the blob.
//!
//! * `source`: `{name, format}` (`stl`, `obj`, `glb`, `3mf`, `skp`): where the mesh came from. Informational only (the
//!   panel shows it); the regeneration never reads it and it is not part of the cache key. Default none.
//! * `blob`: a key into the part's `blobs` table. The entry is the mesh as base64 text (the layouts are in
//!   `ck_mesh::blob`: the app's packed `CKM2` payload, or `u32 nVerts, u32 nTris, f32 xyz.., u32 indices..`, each
//!   optionally deflated) and the key is the SHA-256 of that text, checked on every run.
//! * `opts`: the options of the `meshToSolid` request (`close`, `weldTol`, `planeTol`, `angleTolDeg`, `maxTriangles`,
//!   `maxFaces`, `analytic`, `deviation`, `tol`, `fitTol`, `maxSeconds`, `decimate`), read the same way. Part of the cache key.
//! * `replace: true`: the bodies so far (solid lumps and surface bodies) are removed first, then the mesh's solid
//!   joins. Without it the solid joins beside the bodies so far, as `importStep` does. Default false.
//! * `body`: the name of the solid, as `importStep`.
//! * `preview: true`: a dry run (`opts.dryRun` of the request): nothing is built or added. The feature carries the
//!   warning `meshToSolidPreview` with the report and, per hole `opts.close` would fill, the triangles it would invent.
//!   Default false.
//! * Faces are named `<id>.I1F<n>` (n = the face's place in the body, from 1).
//! * Result: the warning `meshToSolidReport {detail, report}`, informational, on every successful run (also when the
//!   result was read from the saved cache): `report` is the `import` object of the `meshToSolid` reply (`faceTypes`,
//!   `analytic.fitted` / `faceted`, `deviation`, `volume`, `closing` with `holes[].at`, ...). The panel reads it after a
//!   reload from here.
//! * Errors: the refusal codes of `meshToSolid` (`meshNotWatertight`, `meshNonManifold`, `meshHoleTooLarge`,
//!   `meshHoleSelfIntersects`, `meshGapAmbiguous`, `meshTooLarge`, `meshDegenerate`, `meshBadInput`), each with the
//!   params `detail` and `message` (the same text), `hint` and the refusal's data (`positions`, `holes`, counts);
//!   `meshToSolidInvalid {detail, hint}` for a feature that is not well formed or a blob that is missing, does not match
//!   its key or is not base64. A time budget that runs out in the conversion is `featureBudgetExceeded` /
//!   `regenBudgetExceeded` as for any feature.

use super::{extrude::Combine, import_body::Sorted, import_step, Brep};
use crate::regen::{FeatureError, RegenState};
use crate::Feature;
use ck_json::Json;
use ck_topo::body::FaceName;

pub const MESH_SOLID_INVALID: &str = "meshToSolidInvalid";
/// Informational warning of every successful run: the conversion report.
pub const MESH_SOLID_REPORT: &str = "meshToSolidReport";
/// Warning of a `preview` run: the report and the triangles a closing would add.
pub const MESH_SOLID_PREVIEW: &str = "meshToSolidPreview";
/// Bump when the conversion (`ck-mesh`) gives another body for the same mesh and options at the same crate version.
pub const MESH_REV: u32 = 1;

fn invalid(detail: impl Into<String>) -> FeatureError {
    FeatureError::new(MESH_SOLID_INVALID).with("detail", Json::Str(detail.into())).with(
        "hint",
        Json::Str("give the feature a `blob` key that names the mesh, as base64 text, in the part's `blobs` table (the key is the SHA-256 of the text)".into()),
    )
}

/// A refusal of the conversion as the feature's error: its code, `detail` / `message`, `hint` and data.
fn refusal(r: &ck_mesh::Refusal) -> FeatureError {
    let mut e = FeatureError::new(r.code).with("detail", Json::Str(r.message.clone())).with("message", Json::Str(r.message.clone())).with("hint", Json::Str(r.hint.clone()));
    for (k, v) in &r.data {
        e = e.with(k, v.clone());
    }
    e
}

/// The base64 text of the mesh the feature names, checked against its key; the key in lower case.
fn data<'a>(state: &'a RegenState, f: &'a Feature) -> Result<(&'a str, String), FeatureError> {
    let key = match f.raw.get("blob") {
        Some(Json::Str(k)) => k,
        Some(_) => return Err(invalid("`blob` must be a string (the key of an entry of the part's `blobs`)")),
        None => return Err(invalid("no data: the feature has no `blob`")),
    };
    let text = state
        .part
        .raw
        .get("blobs")
        .and_then(|b| b.get(key))
        .ok_or_else(|| invalid(format!("the part has no blob `{key}` in its `blobs` table")))?
        .as_str()
        .ok_or_else(|| invalid(format!("blob `{key}` is not text")))?;
    if import_step::blob_key(text) != key.to_ascii_lowercase() {
        return Err(invalid(format!("blob `{key}` does not match its key: the key must be the SHA-256 of the text")));
    }
    Ok((text, key.to_ascii_lowercase()))
}

fn flag(f: &Feature, k: &str) -> Result<bool, FeatureError> {
    match f.raw.get(k) {
        None => Ok(false),
        Some(Json::Bool(b)) => Ok(*b),
        Some(_) => Err(invalid(format!("`{k}` must be true or false"))),
    }
}

/// The one-line summary of a conversion report.
fn summary(report: &Json) -> String {
    let n = |k: &str| report.get_f64(k).unwrap_or(0.0) as usize;
    let mut s = format!("{} triangles became {} faces", n("triangles"), n("faces"));
    let (fitted, faceted) = (
        report.path(&["analytic", "fitted"]).and_then(|v| v.as_arr()).map_or(0, |a| a.len()),
        report.path(&["analytic", "faceted"]).and_then(|v| v.as_f64()).unwrap_or(0.0) as usize,
    );
    if fitted > 0 || faceted > 0 {
        s += &format!(" ({fitted} analytic surface(s) fitted, {faceted} facet(s) left faceted)");
    }
    if let Some(c) = report.get("closing") {
        let k = c.get_f64("filledHoles").unwrap_or(0.0) as usize;
        let m = c.get_f64("mergedVertices").unwrap_or(0.0) as usize;
        let st = c.get_f64("stitched").unwrap_or(0.0) as usize;
        if k + m + st > 0 {
            s += &format!("; closing filled {k} hole(s), merged {m} vertex(es), stitched {st} edge(s)");
        }
    }
    if let Some(d) = report.get("decimation") {
        let t = |k: &str| d.get_f64(k).unwrap_or(0.0) as usize;
        s += &format!("; decimated from {} to {} triangles (deviation at most {:.3} mm of {:.3} mm allowed)", t("inputTriangles"), t("triangles"), d.path(&["deviation", "max"]).and_then(|v| v.as_f64()).unwrap_or(0.0), d.get_f64("tol").unwrap_or(0.0));
    }
    if report.path(&["analytic", "budgetExhausted"]).and_then(|v| v.as_bool()) == Some(true) {
        let pct = report.path(&["analytic", "fittedFraction"]).and_then(|v| v.as_f64()).unwrap_or(0.0) * 100.0;
        s += &format!("; the fit stopped at its work limit (opts.maxSeconds): {pct:.0}% of the surface is fitted, the rest stays faceted");
    }
    s
}

pub fn exec_mesh_solid(br: &mut Brep, state: &mut RegenState, f: &Feature) -> Result<(), FeatureError> {
    match f.raw.get("source") {
        None | Some(Json::Null) | Some(Json::Obj(_)) => {}
        Some(_) => return Err(invalid("`source` must be an object ({name, format}), shown in the panel and never read")),
    }
    let replace = flag(f, "replace")?;
    let preview = flag(f, "preview")?;
    match f.raw.get("opts") {
        None | Some(Json::Null) | Some(Json::Obj(_)) => {}
        Some(_) => return Err(invalid("`opts` must be an object (the options of `meshToSolid`)")),
    }
    let name = super::bodies::body_name(&f.raw)?;
    let (text, blob_key) = data(state, f)?;
    let label = if f.name.is_empty() { f.id.clone() } else { f.name.clone() };
    let mut o = ck_mesh::request::opts_of(&f.raw);
    let decode = || ck_mesh::blob::decode_base64(text).map_err(|r| refusal(&r));

    if preview {
        let m = decode()?;
        let rep = ck_mesh::preview(&m.positions, Some(&m.indices), &o).map_err(|r| refusal(&r))?;
        let report = ck_mesh::request::preview_json(&label, &rep);
        let w = FeatureError::new(MESH_SOLID_PREVIEW)
            .with("detail", Json::Str("dry run: nothing was built".into()))
            .with("report", report);
        state.result.warnings.push(crate::regen::RegenError { feature: f.id.clone(), error: w });
        return Ok(());
    }

    o.dry_run = false;
    let opts_key = f.raw.get("opts").map(ck_json::to_string).unwrap_or_default();
    let kind = format!("meshToSolid{MESH_REV}");
    let sorted = super::built_cache::through(&state.part.raw, &kind, &format!("{blob_key}|{opts_key}"), &f.id, || {
        let m = decode()?;
        let c = ck_mesh::mesh_to_solid(&m.positions, Some(&m.indices), &o).map_err(|r| refusal(&r))?;
        let mut body = c.body;
        ck_topo::region::rebuild_lumps(&mut body);
        let ids: Vec<_> = body.face_ids().collect();
        for (n, fid) in ids.into_iter().enumerate() {
            body.faces[fid.idx()].name = FaceName::other(&f.id, &format!("I1F{}", n + 1));
        }
        let report = ck_mesh::request::report_json(&label, &body, &c.report);
        Ok(Sorted { solid: body, surfaces: vec![], parts: vec![], note: Some(ck_json::to_string(&report)) })
    })?;
    if sorted.solid.num_faces() == 0 {
        return Err(invalid("the mesh gave no faces"));
    }
    if replace {
        br.body = None;
        br.surfs.clear();
    }
    if let Some(n) = &name {
        super::bodies::check_free(br.body.as_ref(), n, &f.id, &[])?;
    }
    br.combine(&sorted.solid, Combine::Separate, false)?;
    if let (Some(n), Some(b)) = (name, br.body.as_mut()) {
        let mut k = 0;
        let ls: Vec<_> = b.lump_ids().collect();
        for l in ls {
            if b.lump(l).name.is_none() && super::bodies::origin(b, l) == Some(f.id.as_str()) {
                k += 1;
                b.lumps[l.idx()].name = Some(if k == 1 { n.clone() } else { format!("{n} {k}") });
            }
        }
    }
    let report = sorted.note.as_deref().and_then(|t| ck_json::parse(t).ok()).unwrap_or(Json::Null);
    let w = FeatureError::new(MESH_SOLID_REPORT).with("detail", Json::Str(summary(&report))).with("report", report);
    state.result.warnings.push(crate::regen::RegenError { feature: f.id.clone(), error: w });
    Ok(())
}
