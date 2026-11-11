//! 2D drawing extraction: `{type:"drawing", tag:"main", views:[{name}|{x,y}],
//! hidden:true}` -> `{views:[...]}`.
//!
//! Pinned by oracle probes:
//! - the tag's state is checked first (`noState` even for malformed views);
//! - a state without a body answers `{}` (the empty object, NOT null);
//! - every view must carry a known `name` or both `x` and `y`, else the whole
//!   request fails with "drawing: a view needs a name (front, back, left,
//!   right, top, bottom, iso) or x and y" (a `views` array of non-objects);
//! - missing/non-array `views` -> `{"views":[]}`;
//! - the span label is "drawing views".
//!
//! Geometry (projection, hidden lines, sections) lives in ck-ops::drawing,
//! reached through the query hook like every body-bearing request.

use crate::session::Cx;
use ck_json::Json;

pub const VIEW_NEEDS_NAME: &str = "drawing: a view needs a name (front, back, left, right, top, bottom, iso) or x and y";
pub const VIEW_NAMES: [&str; 7] = ["front", "back", "left", "right", "top", "bottom", "iso"];

pub const SPAN_LABEL: &str = "drawing views";

pub fn drawing(cx: &mut Cx) -> Result<Json, String> {
    if !cx.state()?.has_body {
        // pinned by tests-golden/wire.json: the empty-state answer is `{}` when
        // hidden is truthy and `null` otherwise
        let hidden = cx.msg.get("hidden").map_or(false, |h| match h {
            Json::Bool(b) => *b,
            Json::Num(n) => *n != 0.0,
            _ => false,
        });
        return Ok(if hidden { Json::Obj(vec![]) } else { Json::Null });
    }
    // every view needs a name (front, back, left, right, top, bottom, iso)
    // or x and y (the whole request fails on the first bad one)
    if let Some(Json::Arr(views)) = cx.msg.get("views") {
        for v in views {
            let ok = v.get_str("name").map_or(false, |n| VIEW_NAMES.contains(&n))
                || (v.get_arr("x").is_some() && v.get_arr("y").is_some());
            if !ok {
                return Err(VIEW_NEEDS_NAME.to_string());
            }
        }
    }
    cx.span(SPAN_LABEL, |cx| cx.query("drawing"))
}
