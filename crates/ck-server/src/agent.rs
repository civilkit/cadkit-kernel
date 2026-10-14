//! Agent-interface extensions (not in the original kernel): `check`,
//! `resolve`, `describe` (body queries) and `refDiff` (stateless, two parts).
//! Wire shapes: docs/agent-queries.md.

use crate::session::{check_part, Cx};
use ck_json::Json;

fn null_or_query(cx: &mut Cx, ty: &str) -> Result<Json, String> {
    if !cx.state()?.has_body {
        return Ok(Json::Null);
    }
    cx.query(ty)
}

/// `{type:"check", tag, expect:[{kind, equals|min|max, tol?}]}`
pub fn check(cx: &mut Cx) -> Result<Json, String> {
    null_or_query(cx, "check")
}

/// `{type:"resolve", tag, descriptions:[{kind:"face", ...}]}`
pub fn resolve(cx: &mut Cx) -> Result<Json, String> {
    null_or_query(cx, "resolve")
}

/// `{type:"describe", tag, names:["F2.C1", "F2.C1|F2.S.e1#3", ...]}`
pub fn describe(cx: &mut Cx) -> Result<Json, String> {
    null_or_query(cx, "describe")
}

/// `{type:"refDiff", base:<part>, part:<part>}`: regenerates both privately
/// and reports references that now point at a different entity.
pub fn ref_diff(cx: &mut Cx) -> Result<Json, String> {
    check_part(cx.msg.get("base"))?;
    check_part(cx.msg.get("part"))?;
    cx.query("refDiff")
}

/// `{type:"threadData", standard?, size?}`: the thread tables (stateless).
/// No `standard`: the list of standards with their size designations; with
/// it: that standard's sizes with all numbers (mm); with `size` too: the one
/// entry. Unknown standard / size are wire errors that list the choices.
pub fn thread_data(cx: &mut Cx) -> Result<Json, String> {
    cx.query("threadData")
}
