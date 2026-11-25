//! Fusion 360 (`.f3d` / `.f3z`) reader.
//!
//! The ASM part (modules [`asm`], [`brep`], [`convert`], [`history_states`], [`ids`], [`vec3`],
//! `testdata`) is MitCad's ASM reader (github.com/mitcad, commit 806a421, MIT), adapted only
//! where it must be: the `mitcad-vec3` crate is the [`vec3`] module, the C++ builder's flat
//! encoding (`flat.rs`) is gone, and everything else is as upstream. The format knowledge comes
//! from inspecting files only (black-box reverse engineering); see `ASM_FORMAT.md`.
//!
//! Ours:
//! - [`zip`]: a small zip reader (stored, deflate and Zstandard entries) with its own decoders,
//!   so the crate has no dependency outside the workspace.
//! - [`container`]: the zip of an `.f3d` / `.f3z` and its body blobs.
//! - [`build`]: the neutral B-rep ([`brep::Body`], millimetres) built into a `ck-topo` body.
//! - [`import`]: the whole path from the bytes of a file or of one `.smb` blob to bodies and a
//!   report in plain English.
//!
//! Layers, bottom up: bytes, [`zip`], [`container`], [`asm`] (tokens and records), [`convert`]
//! (ASM records to [`brep::Body`]), [`build`] (to `ck-topo`), [`import`].

pub mod asm;
pub mod brep;
pub mod build;
pub mod container;
pub mod convert;
pub mod history_states;
pub mod ids;
pub mod import;
#[doc(hidden)]
pub mod testdata;
pub mod vec3;
pub mod zip;
mod inflate;
mod zstd;

/// All bodies of one body blob.
pub struct BlobBodies {
    pub entry: String,
    pub header: asm::Header,
    pub coverage: asm::Coverage,
    /// Tokenising stopped early (unknown tag); the message.
    pub truncated: Option<String>,
    pub bodies: Vec<convert::ConvertedBody>,
}

/// Parses one body blob and converts its top-level bodies.
pub fn read_blob(
    entry: &str,
    data: &[u8],
    options: &convert::Options,
) -> Result<BlobBodies, asm::AsmError> {
    let file = asm::AsmFile::parse(data)?;
    let coverage = asm::Coverage::of(&file);
    let bodies = convert::convert_file(&file, options);
    Ok(BlobBodies {
        entry: entry.to_string(),
        header: file.header.clone(),
        coverage,
        truncated: file.truncated.as_ref().map(|e| e.to_string()),
        bodies,
    })
}
