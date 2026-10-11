//! Shell (hollow) a solid: offset every face inward by the wall thickness
//! and cut the offset cavity out of the body.
//!
//! Semantics pinned by oracle probes (box, opened cap / side, thickness
//! too large, outward):
//! - result = body - cavity; the cavity is the body offset by -t on every
//!   kept face. An opened (removed) face is pushed OUTWARD instead, so the
//!   cavity runs out through it and the face survives as a rim ring
//!   (unsuffixed name; every other face is split `~1` outer / `~2` inner by
//!   the boolean).
//! - `outward`: the wall grows outside - base = body offset by +t (opened
//!   faces stay put), cavity = the original body.
//! - errors: t <= 0 -> invalidValue; an offset edge shrinking / reversing ->
//!   shellTooThick{detail}; opened face not planar -> shellUnsupported;
//!   offset errors (offsetUnsupported / offsetTooLarge) pass through; a
//!   boolean failure -> shellFailed.
//!
//! The original (shell.rs:156, 18 KB) builds an "opening prism" per opened
//! face and verifies flush openings by volume; this version gets the same
//! topology by overshooting the opened face instead.

use crate::boolean::{boolean, Op};
use crate::offset::{offset_body, OffsetError};
use ck_geom::surface::Surface;
use ck_topo::body::{Body, FaceId};

#[derive(Debug, Clone, PartialEq)]
pub struct ShellError {
    pub code: String,
    pub detail: Option<String>,
}

impl ShellError {
    fn code(c: &str) -> ShellError {
        ShellError { code: c.into(), detail: None }
    }
    fn detail(c: &str, d: impl Into<String>) -> ShellError {
        ShellError { code: c.into(), detail: Some(d.into()) }
    }
}

fn from_offset(e: OffsetError) -> ShellError {
    if e.is_degenerate() {
        ShellError { code: "shellTooThick".into(), detail: e.detail }
    } else {
        ShellError { code: e.code.into(), detail: e.detail }
    }
}

/// Hollow `body` with wall `thickness`, removing the `open` faces.
pub fn shell(body: &Body, open: &[FaceId], thickness: f64, outward: bool, tol: f64) -> Result<Body, ShellError> {
    if !(thickness > 0.0) || !thickness.is_finite() {
        return Err(ShellError::code("invalidValue"));
    }
    for &f in open {
        if !matches!(body.face(f).surface, Surface::Plane { .. }) {
            return Err(ShellError::detail("shellUnsupported", format!("opened face {} is not planar", body.face_id_str(f))));
        }
    }
    let is_open = |f: FaceId| open.contains(&f);
    let t = thickness;
    let (base, cavity) = if outward {
        let base = offset_body(body, &|f| if is_open(f) { 0.0 } else { t }).map_err(from_offset)?;
        let cav = offset_body(body, &|f| if is_open(f) { t } else { 0.0 }).map_err(from_offset)?;
        (base, cav)
    } else {
        let cav = offset_body(body, &|f| if is_open(f) { t } else { -t }).map_err(from_offset)?;
        (body.clone(), cav)
    };
    let out = boolean(&base, &cavity, Op::Cut, tol).map_err(|_| ShellError::code("shellFailed"))?;
    if out.no_effect || out.body.num_faces() == 0 {
        return Err(ShellError::code("shellFailed"));
    }
    let mut b = out.body;
    b.number_split_faces();
    Ok(b)
}
