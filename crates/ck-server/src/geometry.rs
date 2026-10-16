//! Geometry queries over a regenerated state: `measure`, `massProps`,
//! `interference`, `step`.
//!
//! Frontend messages (app.pretty.js):
//! - `{type:"measure", tag, refs:[{kind:"face",id}|{kind:"edge",faces,at}]}`
//!   -> `{items:[{area|wedge|...}]}`
//! - `{type:"massProps", tag:"main", bodies:true, densities:[...]}`
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
