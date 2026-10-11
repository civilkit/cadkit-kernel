//! Curvature combs: `{type:"comb", tag, refs:[...], density, lines, dir}`
//! -> one entry per ref.

use crate::session::Cx;
use ck_json::{Json, Obj};

pub const COMB_LABEL: &str = "curvature combs";

/// Span "curvature combs". Empty state, per ref: an object whose `kind` is
/// `face`/`edge`/`curve` -> `{"error":"notFound"}`; anything else (other
/// kinds, `{}`, strings, numbers, null) -> `{"error":"notCurve"}`.
/// `refs` not an array -> `[]`.
pub fn comb(cx: &mut Cx) -> Result<Json, String> {
    cx.span(COMB_LABEL, |cx| {
        if cx.state()?.has_body {
            return cx.query("comb");
        }
        let refs = cx.msg.get_arr("refs").unwrap_or(&[]);
        Ok(Json::Arr(
            refs.iter()
                .map(|r| {
                    let e = match r.get_str("kind") {
                        Some("face" | "edge" | "curve") => "notFound",
                        _ => "notCurve",
                    };
                    Obj::new().put("error", e).build()
                })
                .collect(),
        ))
    })
}
