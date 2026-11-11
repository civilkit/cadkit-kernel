//! Geometry queries over a regenerated state: `measure`, `massProps`,
//! `interference`, `step`.
//!
//! Frontend messages (app.pretty.js):
//! - `{type:"measure", tag, refs:[{kind:"face",id}|{kind:"edge",faces,at}]}`
//!   -> `{items:[{area|wedge|...}]}`
//! - `{type:"massProps", tag:"main", bodies:true, densities:[...]}` — with
//!   `densities` every `bodies[]` entry also carries `mass`; `densities`
//!   alone implies `bodies:true` and must have one entry per lump.
//! - `{type:"interference", tag:"main"}` -> `{bodies:n, pairs:[...]}`
//! - `{type:"step", tag:"main", name}` -> STEP text (ck-io)
//!
//! Pinned with an empty state (body null): all four answer `null`.
//! `massProps` / `interference` run inside the spans "mass properties" /
//! "interference between the bodies", opened BEFORE the state lookup.

use crate::session::Cx;
use ck_json::Json;

pub const MASS_LABEL: &str = "mass properties";
pub const INTERFERENCE_LABEL: &str = "interference between the bodies";

fn null_or_query(cx: &mut Cx, ty: &str) -> Result<Json, String> {
    if !cx.state()?.has_body {
        return Ok(Json::Null);
    }
    cx.query(ty)
}

pub fn measure(cx: &mut Cx) -> Result<Json, String> {
    null_or_query(cx, "measure")
}

pub fn mass_props(cx: &mut Cx) -> Result<Json, String> {
    cx.span(MASS_LABEL, |cx| null_or_query(cx, "massProps"))
}

pub fn interference(cx: &mut Cx) -> Result<Json, String> {
    cx.span(INTERFERENCE_LABEL, |cx| null_or_query(cx, "interference"))
}

pub fn validate(cx: &mut Cx) -> Result<Json, String> {
    null_or_query(cx, "validate")
}

/// `{type:"bodyBoolean", tag, op, target, tools, keepTools?, tolerance?, autoFuzzy?}`: the `combine` feature's
/// operation on a copy of the current body; the reply says what the result would be, nothing is committed.
pub fn body_boolean(cx: &mut Cx) -> Result<Json, String> {
    null_or_query(cx, "bodyBoolean")
}

/// `{type:"exportMesh", tag, quality?, chord_mm?}`: the triangle mesh of the current body at the tolerance an export asks for
/// (docs/spec-agent-interface.md "Export mesh"). Null when the tag holds no body.
pub fn export_mesh(cx: &mut Cx) -> Result<Json, String> {
    null_or_query(cx, "exportMesh")
}

pub fn heal(cx: &mut Cx) -> Result<Json, String> {
    null_or_query(cx, "heal")
}

/// `{"type":"importSat","tag":t,"name":n?,"data":"<SAT text>"}` -> regen-shaped
/// result (`errors`, `body`, ...). A failed import answers `body:null` with a
/// structured error and leaves the tag's state alone.
pub fn import_sat(cx: &mut Cx) -> Result<Json, String> {
    if cx.msg.get_str("data").is_none() {
        return Err("importSat: data (the SAT file text) is required".to_string());
    }
    let tag = cx.tag();
    let Some(h) = cx.hooks.import_sat else { return Err(crate::session::todo("importSat")) };
    let (reply, state) = h(cx)?;
    if let Some(s) = state {
        cx.session.put(&tag, s);
    }
    Ok(reply)
}

/// `{"type":"importSldprt","tag":t,"name":n?,"data":"<base64 of a .SLDPRT file>"}` -> regen-shaped result like
/// `importStep` (`body`, `errors`, plus `import`). A failed import answers `body:null` with a structured error and
/// leaves the tag's state alone.
pub fn import_sldprt(cx: &mut Cx) -> Result<Json, String> {
    if cx.msg.get_str("data").is_none() {
        return Err("importSldprt: data (the .SLDPRT file as base64) is required".to_string());
    }
    let tag = cx.tag();
    let Some(h) = cx.hooks.import_sldprt else { return Err(crate::session::todo("importSldprt")) };
    let (reply, state) = h(cx)?;
    if let Some(s) = state {
        cx.session.put(&tag, s);
    }
    Ok(reply)
}

/// `{"type":"sldprtHistory","name":n?,"data":"<base64 of a .SLDPRT file>"}` -> `{ok, part?, complete?, reason?, outcome?}`: the
/// part file's feature history (sketches, extrudes, fillets, ...) as a part document that regenerates to the body
/// `importSldprt` reads, or `ok:false` with the reason (a file that cannot be read back is not an error of the request:
/// the caller keeps the plain import). Read only: no tag, no state.
pub fn sldprt_history(cx: &mut Cx) -> Result<Json, String> {
    if cx.msg.get_str("data").is_none() {
        return Err("sldprtHistory: data (the .SLDPRT file as base64) is required".to_string());
    }
    let Some(h) = cx.hooks.sldprt_history else { return Err(crate::session::todo("sldprtHistory")) };
    h(cx)
}

/// `{"type":"builtExport","feature":id?}` -> `{ok, entries:[{feature, key, source, rejected?, data}]}`: the saved results of the
/// import features (`importBody`) that the last regens built from their file or read from the document's `built` table,
/// oldest first (at most the last few). `data` is the base64 text to store as `built[key]` of the part document (or beside
/// it), compressed by the app if it likes before keeping and expanded again before it goes back in; `source` is `built` or
/// `saved`; `rejected` says why an entry under the key was present but not used. With `feature`, only that feature's.
/// Read only: no tag, no state. The entry is a cache: a document without it, or with a damaged or foreign one, rebuilds.
pub fn built_export(cx: &mut Cx) -> Result<Json, String> {
    let Some(h) = cx.hooks.built_export else { return Err(crate::session::todo("builtExport")) };
    h(cx)
}

/// `{"type":"importSkp","tag":t,"name":n?,"data":"<base64 of a .skp file>","opts":{"hidden":bool?,"maxFaces":n?}?}` ->
/// regen-shaped result like `importStep` (`body`, `errors`, plus `import`). The body is a surface model: one planar face per
/// SketchUp face (holes kept), vertices welded, edges shared; `import.faceMaterials` lists the colour index of each
/// `skp.F<n>` face. A failed import answers `body:null` with a structured error and leaves the tag's state alone.
pub fn import_skp(cx: &mut Cx) -> Result<Json, String> {
    if cx.msg.get_str("data").is_none() {
        return Err("importSkp: data (the .skp file as base64) is required".to_string());
    }
    let tag = cx.tag();
    let Some(h) = cx.hooks.import_skp else { return Err(crate::session::todo("importSkp")) };
    let (reply, state) = h(cx)?;
    if let Some(s) = state {
        cx.session.put(&tag, s);
    }
    Ok(reply)
}

pub fn step(cx: &mut Cx) -> Result<Json, String> {
    null_or_query(cx, "step")
}

/// Extension `{type:"importStep", tag?, name?, data:"<STEP text>"}`: parse the
/// B-rep solids of a STEP file into the tag's state (same body tagging as
/// regen, so massProps / inspect / validate / mesh / step work on it).
pub fn import_step(cx: &mut Cx) -> Result<Json, String> {
    let data = cx.msg.get_str("data").ok_or_else(|| "importStep: data must be a STEP file as a string".to_string())?;
    let name = cx.msg.get_str("name").unwrap_or("part");
    let tag = cx.tag();
    let Some(h) = cx.hooks.import_step else { return Err(crate::session::todo("importStep")) };
    let (result, state) = h(cx, name, data)?;
    if let Some(st) = state {
        cx.session.put(&tag, st);
    }
    Ok(result)
}
