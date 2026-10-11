//! Selection / diagnostics queries: `inspect`, `continuity`.
//!
//! - `{type:"inspect", tag:"main", ref:{kind,id,entity,at}}` (app.pretty.js)
//! - `{type:"continuity", tag, refs, whole, required, tol:{gap,angle,curvature,rate}}`

use crate::session::Cx;
use ck_json::{Json, Obj};

pub const INSPECT_LABEL: &str = "inspect (geometry diagnostics)";
pub const CONTINUITY_LABEL: &str = "continuity check (G0 to G3 along edges)";

/// Continuity tolerance defaults (oracle, no `tol` given).
pub const TOL_DEFAULTS: [(&str, f64); 4] = [("gap", 0.01), ("angle", 0.1), ("curvature", 0.05), ("rate", 0.1)];

/// Inspect. Pinned on an empty state (all inside the span):
/// - `ref` missing / not an object / `kind` not a string -> `cannot inspect nothing`
/// - `face` -> `face not found`, `edge` -> `edge not found`,
///   `sketch` -> `sketch not found`, `curve` -> `curve not found`,
///   `plane` -> `plane not found`
/// - `vertex` without `at` -> `vertex without a point`, with `at` -> `no vertex`
/// - `feature` -> `{"type":"feature"}` (ok, even for an unknown id)
/// - any other kind (incl. `body`, `FACE`, `""`) -> `cannot inspect <kind>`
pub fn inspect(cx: &mut Cx) -> Result<Json, String> {
    cx.span(INSPECT_LABEL, |cx| {
        let has_body = cx.state()?.has_body;
        let r = cx.msg.get("ref");
        let kind = r.and_then(|r| r.get_str("kind"));
        let Some(kind) = kind else { return Err("cannot inspect nothing".into()) };
        match kind {
            "face" | "edge" | "sketch" | "curve" | "plane" | "vertex" | "feature" if has_body => cx.query("inspect"),
            "face" | "edge" | "sketch" | "curve" | "plane" => Err(format!("{kind} not found")),
            "vertex" => match r.and_then(|r| r.get("at")) {
                // GUESS: any `at` counts; only `[0,0,0]` was probed.
                Some(_) => Err("no vertex".into()),
                None => Err("vertex without a point".into()),
            },
            // GUESS: the original fills this from the part's feature list;
            // with no features only the bare type is reported.
            "feature" => Ok(Obj::new().put("type", "feature").build()),
            k => Err(format!("cannot inspect {k}")),
        }
    })
}

/// `required`: case-insensitive `G0..G3` normalised to upper case; anything
/// else (`"G7"`, `"G1 "`, `"C2"`, `5`) falls back to `"G2"`.
pub fn parse_required(v: Option<&Json>) -> &'static str {
    match v.and_then(Json::as_str).map(|s| s.to_ascii_uppercase()) {
        Some(s) if s == "G0" => "G0",
        Some(s) if s == "G1" => "G1",
        Some(s) if s == "G3" => "G3",
        _ => "G2",
    }
}

/// `tol`: each key overrides its default only when it is a number;
/// unknown keys are dropped; key order is fixed.
pub fn parse_tol(v: Option<&Json>) -> Json {
    let mut o = Obj::new();
    for (k, d) in TOL_DEFAULTS {
        o = o.put(k, v.and_then(|t| t.get_f64(k)).unwrap_or(d));
    }
    o.build()
}

pub fn continuity(cx: &mut Cx) -> Result<Json, String> {
    cx.span(CONTINUITY_LABEL, |cx| {
        if cx.state()?.has_body {
            return cx.query("continuity");
        }
        Ok(Obj::new()
            .put("required", parse_required(cx.msg.get("required")))
            .put("tol", parse_tol(cx.msg.get("tol")))
            .put("edges", Json::Arr(vec![]))
            .build())
    })
}
