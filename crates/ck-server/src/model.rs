//! Model edits & sketch helpers: `drag`, `convert`, `sketchOffset`,
//! `refCurve`, `matchEdges`.
//!
//! Frontend messages (app.pretty.js):
//! - `{type:"drag", tag:"main", face, point, target}` -> `{delta, place, moves}`
//! - `{type:"convert", tag:"asm"|"sketch", sketch, frame, ref}` -> `{sketch, ids}`
//! - `{type:"sketchOffset", sketch, ids, distance}` -> `{chains}` (no tag/state)
//! - `{type:"refCurve", tag:"sketch"|"bridge", ref}` -> `{curve, closed, start, end}`
//! - `{type:"matchEdges", part, upTo, mode, seeds, exclude}` -> `[...]` (no state)

use crate::session::{check_part, regen_label, todo, Cx};
use ck_json::{Json, Obj};

pub const DRAG_REQUIRED: &str = "drag: face, point and target are required";
pub const BAD_FRAME: &str = "bad sketch frame";
pub const NO_DISTANCE: &str = "sketchOffset without a distance";

fn missing_reference() -> Json {
    Obj::new().put("error", "missingReference").build()
}

/// State first (`noState`), then `face` string + `point`/`target` numeric
/// triples (`point:[0,0]` or `[0,0,0,1]` both rejected), then on an empty
/// state `{"error":"missingReference"}`.
pub fn drag(cx: &mut Cx) -> Result<Json, String> {
    let has_body = cx.state()?.has_body;
    let m = cx.msg;
    let ok = m.get_str("face").is_some()
        && m.get("point").and_then(Json::as_vec3).is_some()
        && m.get("target").and_then(Json::as_vec3).is_some();
    if !ok {
        return Err(DRAG_REQUIRED.into());
    }
    if !has_body {
        return Ok(missing_reference());
    }
    cx.query("drag")
}

/// A sketch frame is an object with `o`,`x`,`y`,`z` numeric triples.
/// Degenerate axes (all zero, non-unit z) are accepted (oracle).
pub fn valid_frame(f: Option<&Json>) -> bool {
    match f {
        Some(f @ Json::Obj(_)) => ["o", "x", "y", "z"].iter().all(|k| f.get(k).and_then(Json::as_vec3).is_some()),
        _ => false,
    }
}

/// State, then frame (`bad sketch frame`), then `null` on an empty state.
pub fn convert(cx: &mut Cx) -> Result<Json, String> {
    let has_body = cx.state()?.has_body;
    if !valid_frame(cx.msg.get("frame")) {
        return Err(BAD_FRAME.into());
    }
    if !has_body {
        return Ok(Json::Null);
    }
    cx.query("convert")
}

/// No state needed. `distance` must be a number (a string `"1"` is rejected);
/// a sketch with no entities offsets to `{"chains":[]}`.
pub fn sketch_offset(cx: &mut Cx) -> Result<Json, String> {
    let d = cx.msg.get_f64("distance").ok_or_else(|| NO_DISTANCE.to_string())?;
    let has_entities = cx.msg.get("sketch").and_then(|s| s.get_arr("entities")).is_some_and(|e| !e.is_empty());
    if !has_entities {
        return Ok(Obj::new().put("chains", Json::Arr(vec![])).build());
    }
    match cx.hooks.sketch_offset {
        Some(h) => h(cx, d),
        None => Err(todo("sketchOffset of sketch entities")),
    }
}

/// State, then `{"error":"missingReference"}` on an empty state (any `ref`).
pub fn ref_curve(cx: &mut Cx) -> Result<Json, String> {
    if !cx.state()?.has_body {
        return Ok(missing_reference());
    }
    cx.query("refCurve")
}

/// Regenerates `msg.part` privately (span "regenerate N features", no
/// snapshot span, no state stored); `invalid part: Format` on a bad part;
/// an empty part matches no edges (`[]`).
pub fn match_edges(cx: &mut Cx) -> Result<Json, String> {
    let part = check_part(cx.msg.get("part"))?;
    if let Some(h) = cx.hooks.match_edges {
        return h(part, cx);
    }
    let n = part.get_arr("features").map_or(0, |f| f.len());
    if n > 0 {
        return Err(todo("matchEdges on a part with features"));
    }
    cx.span(regen_label(0), |_| ());
    Ok(Json::Arr(vec![]))
}
