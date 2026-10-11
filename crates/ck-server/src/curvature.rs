//! Curvature queries: `curvatureAt` (probe points) and `curvatureMap`
//! (per-face false-colour mesh, buffer-backed).
//!
//! - `{type:"curvatureAt", tag, points:[...]}` -> one entry per point
//! - `{type:"curvatureMap", tag}` -> `{bodies:[{id,faces,...$buf}], failed:[]}`
//!   (bracket: 4 buffers f32,f32,f32,i32)

use crate::session::Cx;
use ck_json::{Json, Obj};

pub const MAP_LABEL: &str = "curvature map";

pub fn not_found() -> Json {
    Obj::new().put("error", "notFound").build()
}

/// No span. Empty state: `{"error":"notFound"}` per element of `points`
/// (any element type); `points` not an array -> `[]`.
pub fn curvature_at(cx: &mut Cx) -> Result<Json, String> {
    if !cx.state()?.has_body {
        let n = cx.msg.get_arr("points").map_or(0, |p| p.len());
        return Ok(Json::Arr((0..n).map(|_| not_found()).collect()));
    }
    cx.query("curvatureAt")
}

/// Span "curvature map" (opened before the state lookup).
/// Empty state: `{"bodies":[],"failed":[]}`.
pub fn curvature_map(cx: &mut Cx) -> Result<Json, String> {
    cx.span(MAP_LABEL, |cx| {
        if !cx.state()?.has_body {
            return Ok(Obj::new().put("bodies", Json::Arr(vec![])).put("failed", Json::Arr(vec![])).build());
        }
        cx.query("curvatureMap")
    })
}
