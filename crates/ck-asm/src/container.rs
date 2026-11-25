//! The zip container of an `.f3d` document and of an `.f3z` package, down to the ASM body blobs.
//!
//! An `.f3d` keeps its B-rep bodies as `Breps.BlobParts/BREP.<guid>.smb` entries (and `.smbh`
//! with the ASM history) under an asset folder (`FusionAssetName[Active]`). An `.f3z` package is
//! a zip of such documents (`<guid>.f3d` entries) with a `Manifest.json`. Only the body blobs
//! are read here; the design segment (timeline, parameters, sketches) is not.
//!
//! The entry classification follows MitCad's `core/f3d-file` container reader.

use crate::zip::{Archive, R, ZipError};

/// The deepest nesting of documents read (an `.f3z` holds `.f3d` documents; they hold none).
const MAX_DEPTH: usize = 3;

/// One body blob of a file.
#[derive(Clone, Debug)]
pub struct Blob {
    /// The entry's path; inside an `.f3z` package it is `<document>/<entry>`.
    pub entry: String,
    /// The blob's GUID (`BREP.<guid>.smb`).
    pub guid: String,
    /// `.smbh`: the blob carries ASM history (its bodies are the stored state).
    pub history: bool,
    pub data: Vec<u8>,
}

/// Whether an entry path is a body blob; the GUID and whether it has history.
pub fn classify_blob(name: &str) -> Option<(String, bool)> {
    let parts: Vec<&str> = name.split('/').collect();
    let file = *parts.last()?;
    if !parts.contains(&"Breps.BlobParts") {
        return None;
    }
    let history = file.ends_with(".smbh");
    if !history && !file.ends_with(".smb") {
        return None;
    }
    let stem = file.trim_end_matches(".smbh").trim_end_matches(".smb");
    Some((stem.strip_prefix("BREP.").unwrap_or(stem).to_string(), history))
}

/// Every body blob of an `.f3d` or `.f3z` file, in archive order (documents of a package in
/// theirs). A blob that does not read (a bad CRC, an unsupported compression method) is an
/// error naming the entry.
pub fn read_blobs(bytes: &[u8]) -> R<Vec<Blob>> {
    let mut out = Vec::new();
    collect(bytes, "", 0, &mut out)?;
    Ok(out)
}

fn collect(bytes: &[u8], prefix: &str, depth: usize, out: &mut Vec<Blob>) -> R<()> {
    let archive = Archive::new(bytes)?;
    for e in archive.entries() {
        if e.name.ends_with('/') {
            continue;
        }
        if let Some((guid, history)) = classify_blob(&e.name) {
            let data = archive.read(e).map_err(|err| {
                ZipError::new(err.code, format!("{prefix}{}: {}", e.name, err.message))
            })?;
            out.push(Blob {
                entry: format!("{prefix}{}", e.name),
                guid,
                history,
                data,
            });
        } else if depth + 1 < MAX_DEPTH && !e.name.contains('/') && e.name.ends_with(".f3d") {
            let doc = archive.read(e).map_err(|err| {
                ZipError::new(err.code, format!("{prefix}{}: {}", e.name, err.message))
            })?;
            collect(&doc, &format!("{prefix}{}/", e.name), depth + 1, out)?;
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::zip::stored_zip;

    #[test]
    fn classifies_blob_entries() {
        let (g, h) = classify_blob("FusionAssetName[Active]/Breps.BlobParts/BREP.1dc4-x.smbh").unwrap();
        assert_eq!((g.as_str(), h), ("1dc4-x", true));
        let (g, h) = classify_blob("A/Breps.BlobParts/BREP.ab.smb").unwrap();
        assert_eq!((g.as_str(), h), ("ab", false));
        assert!(classify_blob("A/Manifest.dat").is_none());
        assert!(classify_blob("A/Other/BREP.ab.smb").is_none());
    }

    #[test]
    fn reads_blobs_of_a_document_and_of_a_package() {
        let doc = stored_zip(&[
            ("Properties.dat", b"x"),
            ("A/Breps.BlobParts/BREP.g1.smb", b"one"),
            ("A/Breps.BlobParts/BREP.g1.smbh", b"two"),
        ]);
        let blobs = read_blobs(&doc).unwrap();
        assert_eq!(blobs.len(), 2);
        assert_eq!(blobs[0].data, b"one");
        assert!(blobs[1].history);
        let pkg = stored_zip(&[("Manifest.json", b"{}"), ("d1.f3d", &doc)]);
        let blobs = read_blobs(&pkg).unwrap();
        assert_eq!(blobs.len(), 2);
        assert_eq!(blobs[0].entry, "d1.f3d/A/Breps.BlobParts/BREP.g1.smb");
        assert!(read_blobs(b"nope").is_err());
    }
}
