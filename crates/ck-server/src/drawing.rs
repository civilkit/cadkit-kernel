//! 2D drawing extraction: `{type:"drawing", tag:"main", views:[{name}|{x,y}], hidden:true}`
//! -> `{views:[...]}`.
//!
//! Strings pinned in kernel.strings: "drawing views", "drawing: a view needs a
//! name (front, back, left, right, top, bottom, iso) or x and y".

use crate::session::Cx;
use ck_json::Json;

pub const VIEW_NEEDS_NAME: &str = "drawing: a view needs a name (front, back, left, right, top, bottom, iso) or x and y";
pub const VIEW_NAMES: [&str; 7] = ["front", "back", "left", "right", "top", "bottom", "iso"];

/// No span. Empty state -> `null` (whatever `views` holds).
pub fn drawing(cx: &mut Cx) -> Result<Json, String> {
    if !cx.state()?.has_body {
        return Ok(Json::Null);
    }
    cx.query("drawing")
}
