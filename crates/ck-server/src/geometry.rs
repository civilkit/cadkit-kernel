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

pub fn step(cx: &mut Cx) -> Result<Json, String> {
    null_or_query(cx, "step")
}
