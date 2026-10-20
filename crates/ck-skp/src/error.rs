//! Structured errors and safety limits: the reader never panics on malformed input, every failure is a value with a
//! stable code, a message and (where it makes sense) the byte offset in the input or in the inflated stream.
//!
//! Part of ck-skp, a Rust reimplementation of the SketchUp reader of OpenSKP (MIT, (c) Ahsan Mehmood), see NOTICE.

use std::fmt;

#[derive(Debug, Clone, PartialEq)]
pub struct SkpError {
    /// Stable machine code: `container.*`, `zip.*`, `inflate.*`, `limit.*`, `tlv.*`, `legacy.*`, `scene.*`, `body.*`.
    pub code: &'static str,
    pub message: String,
    /// Byte offset the problem was found at (file offset for container errors, offset inside `model.dat` for VFF
    /// structure errors, offset inside the file for the legacy archive walk).
    pub offset: Option<usize>,
}

impl SkpError {
    pub fn new(code: &'static str, message: impl Into<String>) -> SkpError {
        SkpError { code, message: message.into(), offset: None }
    }
    pub fn at(code: &'static str, offset: usize, message: impl Into<String>) -> SkpError {
        SkpError { code, message: message.into(), offset: Some(offset) }
    }
}

impl fmt::Display for SkpError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self.offset {
            Some(o) => write!(f, "{} @{:#x}: {}", self.code, o, self.message),
            None => write!(f, "{}: {}", self.code, self.message),
        }
    }
}

impl std::error::Error for SkpError {}

pub type R<T> = Result<T, SkpError>;

/// Safety limits for untrusted input. Every limit is checked before the allocation it guards.
#[derive(Debug, Clone)]
pub struct Limits {
    /// Largest accepted file.
    pub max_file: usize,
    /// Largest single inflated ZIP entry (`model.dat`, material images).
    pub max_entry: usize,
    /// Largest total of all inflated ZIP entries.
    pub max_total: usize,
    /// Most ZIP entries read.
    pub max_entries: usize,
    /// Most TLV nodes in `model.dat` (VFF).
    pub max_nodes: usize,
    /// Deepest TLV container nesting (VFF).
    pub max_tlv_depth: usize,
    /// Most objects in the legacy archive store map.
    pub max_objects: usize,
    /// Deepest object nesting in the legacy archive walk.
    pub max_obj_depth: usize,
    /// Most vertices of one face (all loops together) that are triangulated.
    pub max_face_verts: usize,
    /// Most bytes of texture images kept.
    pub max_texture_bytes: usize,
}

impl Default for Limits {
    fn default() -> Limits {
        Limits {
            max_file: 512 << 20,
            max_entry: 512 << 20,
            max_total: 1024 << 20,
            max_entries: 20_000,
            max_nodes: 40_000_000,
            max_tlv_depth: 64,
            max_objects: 40_000_000,
            max_obj_depth: 200,
            max_face_verts: 200_000,
            max_texture_bytes: 256 << 20,
        }
    }
}
