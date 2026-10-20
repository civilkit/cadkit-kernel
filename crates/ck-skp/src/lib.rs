//! ck-skp: a SketchUp (`.skp`) reader. Parses both containers (the modern 2021+ ZIP/TLV one and the classic 3 to 2020 MFC
//! `CArchive` stream), keeps the model (vertices, edges, faces with holes and UVs, component definitions and instances
//! with transforms, groups, tags, materials with colour, opacity and texture bytes), bakes it into world-space
//! triangulated meshes per material in millimetres, and builds a kernel body of planar faces.
//!
//! Untrusted input: the reader never panics. Every read is bounds checked, sizes and depths are limited (see
//! [`Limits`], [`BakeOptions`]) and failures are [`SkpError`] values with a stable code and, where possible, a byte
//! offset.
//!
//! This crate is a Rust reimplementation of the SketchUp reader of OpenSKP (MIT, (c) Ahsan Mehmood,
//! github.com/iamahsanmehmood/openskp), which reverse engineered the formats. See NOTICE. It uses no SketchUp SDK and
//! no proprietary material. `SketchUp` is a trademark of its owner; it is used here only to name the file format read.

#![forbid(unsafe_code)]

pub mod body;
pub mod error;
pub mod export;
pub mod inflate;
pub mod legacy;
pub mod legacy_ar;
pub mod model;
pub mod scene;
pub mod tlv;
pub mod tri;
pub mod vff;
pub mod xml;
pub mod zip;

pub use error::{Limits, SkpError};
pub use model::{Container, Model};
pub use scene::{bake, BakeOptions, Scene};

/// Parse a `.skp` file (either container) with the default limits.
pub fn parse(data: &[u8]) -> Result<Model, SkpError> {
    parse_with(data, &Limits::default())
}

pub fn parse_with(data: &[u8], lim: &Limits) -> Result<Model, SkpError> {
    if data.len() > lim.max_file {
        return Err(SkpError::new("limit.file", format!("file of {} bytes exceeds the limit of {}", data.len(), lim.max_file)));
    }
    if !data.starts_with(&vff::MAGIC) {
        return Err(SkpError::at("container.magic", 0, "not a SketchUp file (bad header magic)"));
    }
    if legacy::is_legacy(data) {
        return legacy::parse(data, lim);
    }
    if zip::find_local_header(&data[..data.len().min(4096)], 0).is_some() {
        return vff::parse(data, lim);
    }
    Err(SkpError::new("container.unknown", "neither a ZIP container nor a classic CVersionMap archive was found"))
}
