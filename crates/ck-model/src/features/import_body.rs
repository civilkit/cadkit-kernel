//! `importBody`: the solid of a native part file held by the part, read again on every regeneration. It is what an
//! editable import of a SolidWorks part ends with when only the first features of the file's history can be rebuilt:
//! the rebuilt features come first, then this feature puts the file's own body in place of theirs (`replace`), so the
//! final body is exactly the plain import and the features before it stay editable.
//!
//! ```text
//! {"type":"importBody","id":"F9","name":"Imported remainder","format":"sldprt","blob":"<sha-256 hex>","replace":true,
//!  "from":{"id":7,"name":"Loft1","kind":"Loft"},"reason":"feature type Loft","steps":[{"id":7,"name":"Loft1","kind":"Loft"}]}
//! ```
//!
//! * `format`: `sldprt` (a SolidWorks part file, read with `ck-sldprt` as the `importSldprt` request reads it). The only
//!   format; a missing or other value is `importBodyInvalid`.
//! * `blob`: a key into the part's top-level `blobs` table, as for `importStep`. The entry is the file as base64 text and
//!   the key the SHA-256 of that text (`import_step::blob_key`); the key is checked against the text on every run, so a
//!   cached state can never belong to other data than its key says. The regen cache keys the feature by its JSON, so the
//!   file adds only its key.
//! * `replace: true`: the bodies so far (solid lumps and surface bodies) are removed first, then the file's body joins.
//!   Without it the file's solids join beside the bodies so far, as `importStep` does.
//! * `body`: names of the solids (`<body>`, `<body> 2`, ...), as `importStep`.
//! * `from`, `reason`, `steps`: what the remainder stands for (the first step of the file's history that was not rebuilt,
//!   why, and every step from there on). Not read by the regeneration; for the feature tree.
//! * Faces are named `<id>.I<solid>F<n>` (solid = the region's place in the file, n = the face's place in it, both from
//!   1), the same on every run, so features added after the remainder can reference them.
//! * The body is the one the `importSldprt` request gives: edges get the file's tolerance, nothing else is changed; a
//!   region whose faces do not make a valid solid becomes the surface body `<id>.P<k>` and the feature carries the
//!   warning `importBodyParts` (same params as `importStepParts`).
//! * Error: `importBodyInvalid {detail, reason?}` (no or bad data, the key does not match, the file cannot be read).

use super::{extrude::Combine, import_step, Brep};
use crate::regen::{FeatureError, RegenState};
use crate::Feature;
use ck_json::Json;
use ck_topo::body::{Body, FaceName};

pub const IMPORT_BODY_INVALID: &str = "importBodyInvalid";
/// The warning of an import with regions that are not valid solids.
pub const IMPORT_BODY_PARTS: &str = "importBodyParts";

fn invalid(detail: impl Into<String>) -> FeatureError {
    FeatureError::new(IMPORT_BODY_INVALID).with("detail", Json::Str(detail.into())).with(
        "hint",
        Json::Str("give the feature `format: \"sldprt\"` and a `blob` key that names the part file, as base64 text, in the part's `blobs` table (the key is the SHA-256 of the text)".into()),
    )
}

/// The base64 text of the file the feature names, checked against its key.
fn data<'a>(state: &'a RegenState, f: &'a Feature) -> Result<&'a str, FeatureError> {
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
    Ok(text)
}

/// What the reader gives for a part file, sorted as the `importSldprt` request sorts it: the solid body, the surface
/// bodies (`(<prefix>.P<k>, body)`) and what became of each region.
pub struct Sorted {
    pub solid: Body,
    pub surfaces: Vec<(String, Body)>,
    pub parts: Vec<ck_heal::honest::Part>,
}

/// Sort the body of a part file (`ck_sldprt::import_sldprt`): edges that follow their faces only to the file's accuracy
/// get a tolerance (as a STEP import does), nothing else changes; a region whose faces make a valid solid joins the
/// solid body, any other is a surface body `<prefix>.P<k>`. With `rename`, faces are first named `<rename>.I<k>F<n>`.
pub fn sort_sldprt(mut body: Body, parts: &[ck_sldprt::build::PartRead], prefix: &str, rename: Option<&str>) -> Sorted {
    use ck_heal::honest::{self, PartIn};
    if let Some(id) = rename {
        for (k, p) in parts.iter().enumerate() {
            for (n, &fid) in p.faces.iter().enumerate() {
                if fid.idx() < body.faces.len() {
                    body.faces[fid.idx()].name = FaceName::other(id, &format!("I{}F{}", k + 1, n + 1));
                }
            }
        }
    }
    ck_topo::region::rebuild_lumps(&mut body);
    ck_heal::heal_with(&mut body, &ck_heal::tolerant_only());
    let ins: Vec<PartIn> = parts
        .iter()
        .enumerate()
        .map(|(k, p)| PartIn {
            label: format!("body {} (#{})", k + 1, p.body),
            faces: p.faces.clone(),
            faces_total: p.faces_total,
            reasons: {
                let mut r: Vec<honest::Reason> = p.refused.iter().map(|(f, e)| honest::refused_face(e.code, format!("F{f}"), &e.message)).collect();
                if p.sheet {
                    // a sheet body is a surface whatever it encloses: it has an area and no volume
                    r.push(honest::Reason { code: "sheetBody".into(), faces: vec![], message: "the file holds this as a sheet body (a surface), not a solid".into() });
                }
                r
            },
        })
        .collect();
    let s = honest::sort(&body, &ins, ck_check::DEFAULT_TOL_CAP);
    let surfaces = s.surfaces.into_iter().map(|(k, b)| (import_step::surface_id(prefix, k), b)).collect();
    Sorted { solid: s.solid, surfaces, parts: s.parts }
}

pub fn exec_import_body(br: &mut Brep, state: &mut RegenState, f: &Feature) -> Result<(), FeatureError> {
    match f.raw.get("format") {
        Some(Json::Str(s)) if s == "sldprt" => {}
        Some(Json::Str(s)) => return Err(invalid(format!("format `{s}` is not supported (only `sldprt`)"))),
        Some(_) => return Err(invalid("`format` must be a string")),
        None => return Err(invalid("no `format` (`sldprt`)")),
    }
    let replace = match f.raw.get("replace") {
        None => false,
        Some(Json::Bool(b)) => *b,
        Some(_) => return Err(invalid("`replace` must be true or false")),
    };
    let name = super::bodies::body_name(&f.raw)?;
    let text = data(state, f)?;
    // `data` checked that the feature's `blob` is the SHA-256 of `text`
    let blob_key = f.raw.get("blob").and_then(|k| k.as_str()).unwrap_or_default().to_ascii_lowercase();
    let sorted = super::built_cache::through(&state.part.raw, "importBody", &blob_key, &f.id, || {
        let bytes = ck_text::base64_decode(text).map_err(|e| invalid(format!("the blob is not base64 text: {e}")))?;
        let imp = ck_sldprt::import_sldprt(&bytes).map_err(|e| FeatureError::new(IMPORT_BODY_INVALID).with("detail", Json::Str(e.message.clone())).with("reason", Json::Str(e.code.to_string())))?;
        Ok(sort_sldprt(imp.body, &imp.parts, &f.id, Some(&f.id)))
    })?;
    if sorted.solid.num_faces() == 0 && sorted.surfaces.is_empty() {
        return Err(invalid("the file holds no solid"));
    }
    if replace {
        br.body = None;
        br.surfs.clear();
    }
    if let Some(n) = &name {
        super::bodies::check_free(br.body.as_ref(), n, &f.id, &[])?;
    }
    if sorted.solid.num_faces() > 0 {
        br.combine(&sorted.solid, Combine::Separate, false)?;
    }
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
    for (id, sb) in sorted.surfaces {
        br.set_surface(&id, sb);
    }
    if sorted.parts.iter().any(|p| p.outcome != ck_heal::honest::Outcome::Solid) {
        let n = |o| sorted.parts.iter().filter(|p| p.outcome == o).count() as f64;
        let (ns, total) = (sorted.parts.iter().filter(|p| p.outcome == ck_heal::honest::Outcome::Surface).count(), sorted.parts.len());
        let w = FeatureError::new(IMPORT_BODY_PARTS)
            .with("detail", Json::Str(format!("{ns} of {total} region(s) of the file are not valid solids and came in as surface bodies of the faces that could be read")))
            .with("solids", Json::Num(n(ck_heal::honest::Outcome::Solid)))
            .with("surfaces", Json::Num(n(ck_heal::honest::Outcome::Surface)))
            .with("refused", Json::Num(n(ck_heal::honest::Outcome::Refused)))
            .with("parts", import_step::parts_json(&f.id, &sorted.parts));
        state.result.warnings.push(crate::regen::RegenError { feature: f.id.clone(), error: w });
    }
    Ok(())
}
