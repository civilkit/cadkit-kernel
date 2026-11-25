//! The zip layer of this crate: the API of MitCad's `mitcad-zip` reader (an owned [`Archive`], [`Entry`] with 64-bit
//! sizes, the [`Method`] and [`ZipError`] enums, per-read size limits) on top of `ck_asm::zip`, whose stored, deflate
//! and Zstandard decoders this crate shares with ck-asm (no dependency outside the workspace).
//!
//! Ours (this repository's own code); the interface follows MitCad's `core/zip` so that the f3d-file sources above it
//! are unchanged. Differences from `mitcad-zip`: no ZIP64 (a 4 GB archive is refused as not supported), an entry reads
//! up to ck_asm's 512 MiB whatever limit is asked, no zip writer (nothing here writes), and
//! [`Archive::open`] exists only off wasm32 (nothing touches the file system on wasm). The ratio check of mitcad#923
//! (a large entry declaring far more than 1032 times its compressed size is refused before it is decoded) is kept.

use std::fmt;

use ck_asm::zip as az;

/// Compression method of a zip entry.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Method {
    Stored,
    Deflate,
    Zstd,
    Other(u16),
}

impl Method {
    fn from_code(code: u16) -> Self {
        match code {
            0 => Method::Stored,
            8 => Method::Deflate,
            93 => Method::Zstd,
            other => Method::Other(other),
        }
    }

    pub fn code(self) -> u16 {
        match self {
            Method::Stored => 0,
            Method::Deflate => 8,
            Method::Zstd => 93,
            Method::Other(code) => code,
        }
    }
}

impl fmt::Display for Method {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Method::Stored => f.write_str("stored"),
            Method::Deflate => f.write_str("deflate"),
            Method::Zstd => f.write_str("zstd"),
            Method::Other(code) => write!(f, "method {code}"),
        }
    }
}

/// Error while reading the container.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ZipError {
    /// No end-of-central-directory record: not a zip file.
    NotZip,
    /// A record lies outside the file or has a wrong signature.
    Corrupt(String),
    /// The entry uses a feature this reader does not implement.
    Unsupported(String),
    /// Decompression failed.
    Decompress(String),
    /// The entry's CRC-32 does not match its data.
    Crc { name: String },
    NotFound(String),
}

impl fmt::Display for ZipError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            ZipError::NotZip => f.write_str("not a zip file (no end of central directory)"),
            ZipError::Corrupt(what) => write!(f, "corrupt zip: {what}"),
            ZipError::Unsupported(what) => write!(f, "unsupported zip feature: {what}"),
            ZipError::Decompress(what) => write!(f, "decompression failed: {what}"),
            ZipError::Crc { name } => write!(f, "CRC-32 mismatch in {name}"),
            ZipError::NotFound(name) => write!(f, "no entry named {name}"),
        }
    }
}

impl std::error::Error for ZipError {}

/// The ck-asm error as this layer's. `name` is the entry that was being read, for the CRC case.
fn convert(e: az::ZipError, name: &str) -> ZipError {
    match e.code {
        "zip.no_directory" => ZipError::NotZip,
        "zip.crc" => ZipError::Crc { name: name.to_string() },
        "zip.missing" => ZipError::NotFound(name.to_string()),
        "zip.unsupported" | "zip.limit" | "limit.inflate" | "zstd.limit" => ZipError::Unsupported(e.message),
        c if c.starts_with("inflate.") || c.starts_with("zstd.") || c == "zip.size_mismatch" => {
            ZipError::Decompress(e.message)
        }
        _ => ZipError::Corrupt(e.message),
    }
}

/// One entry of the central directory.
#[derive(Clone, Debug)]
pub struct Entry {
    pub name: String,
    pub method: Method,
    pub compressed_size: u64,
    pub size: u64,
    pub crc32: u32,
    inner: az::Entry,
}

impl Entry {
    pub fn is_dir(&self) -> bool {
        self.name.ends_with('/')
    }
}

/// A zip archive held in memory.
pub struct Archive {
    data: Vec<u8>,
    entries: Vec<Entry>,
}

/// Upper bound for one decompressed entry read with [`Archive::read`] (mitcad#923).
pub const DEFAULT_ENTRY_LIMIT: u64 = 256 << 20;

/// The largest limit a caller may opt in to with [`Archive::read_with_limit`]. ck_asm's own reader stops at 512 MiB.
pub const LARGE_ENTRY_LIMIT: u64 = 4 << 30;

/// An entry that declares more than this many times its compressed size is refused before it is decompressed.
pub const MAX_EXPANSION_RATIO: u64 = 1032;

/// Entries up to this size skip the ratio check (a run of equal bytes legitimately packs a few KB into a dozen).
const RATIO_CHECK_FLOOR: u64 = 1 << 20;

impl Archive {
    /// Reads the central directory of a zip file held in memory.
    pub fn new(data: Vec<u8>) -> Result<Self, ZipError> {
        let entries = {
            let a = az::Archive::new(&data).map_err(|e| convert(e, ""))?;
            a.entries()
                .iter()
                .map(|e| Entry {
                    name: e.name.clone(),
                    method: Method::from_code(e.method),
                    compressed_size: e.compressed_size as u64,
                    size: e.size as u64,
                    crc32: e.crc,
                    inner: e.clone(),
                })
                .collect()
        };
        Ok(Archive { data, entries })
    }

    /// Reads a file (not on wasm32, which has no file system).
    #[cfg(not(target_arch = "wasm32"))]
    pub fn open(path: &std::path::Path) -> Result<Self, ZipError> {
        let data = std::fs::read(path).map_err(|e| ZipError::Corrupt(format!("cannot read file: {e}")))?;
        Archive::new(data)
    }

    pub fn entries(&self) -> &[Entry] {
        &self.entries
    }

    pub fn find(&self, name: &str) -> Option<&Entry> {
        self.entries.iter().find(|e| e.name == name)
    }

    pub fn read_by_name(&self, name: &str) -> Result<Vec<u8>, ZipError> {
        let entry = self.find(name).ok_or_else(|| ZipError::NotFound(name.to_string()))?;
        self.read(entry)
    }

    /// [`Archive::read_by_name`] with the limit of [`Archive::read_with_limit`].
    pub fn read_by_name_with_limit(&self, name: &str, limit: u64) -> Result<Vec<u8>, ZipError> {
        let entry = self.find(name).ok_or_else(|| ZipError::NotFound(name.to_string()))?;
        self.read_with_limit(entry, limit)
    }

    /// Decompresses an entry and checks its CRC-32. The entry may declare at most [`DEFAULT_ENTRY_LIMIT`] bytes.
    pub fn read(&self, entry: &Entry) -> Result<Vec<u8>, ZipError> {
        self.read_with_limit(entry, DEFAULT_ENTRY_LIMIT)
    }

    /// [`Archive::read`] of an entry that may declare up to `limit` bytes (at most [`LARGE_ENTRY_LIMIT`]): the explicit
    /// opt-in for the body blobs of a design. The ratio check applies whatever the limit.
    pub fn read_with_limit(&self, entry: &Entry, limit: u64) -> Result<Vec<u8>, ZipError> {
        let limit = limit.min(LARGE_ENTRY_LIMIT);
        if entry.size > limit {
            return Err(ZipError::Unsupported(format!(
                "entry {} declares {} bytes, over the limit of {limit}",
                entry.name, entry.size
            )));
        }
        if entry.size > RATIO_CHECK_FLOOR && entry.size / MAX_EXPANSION_RATIO > entry.compressed_size {
            return Err(ZipError::Unsupported(format!(
                "entry {} declares {} bytes from {} compressed: an expansion beyond {MAX_EXPANSION_RATIO} times",
                entry.name, entry.size, entry.compressed_size
            )));
        }
        // ck_asm's reader holds the bytes by reference and keeps no state, so it is set up per read (its directory
        // parse is a few microseconds for the few hundred entries of a document).
        let a = az::Archive::new(&self.data).map_err(|e| convert(e, &entry.name))?;
        a.read(&entry.inner).map_err(|e| convert(e, &entry.name))
    }
}

/// A zip of stored entries (tests).
#[cfg(test)]
pub use ck_asm::zip::stored_zip;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn reads_stored_entries_and_names_the_errors() {
        let z = stored_zip(&[("a.txt", b"hello"), ("d/b.bin", &[1, 2, 3]), ("d/", b"")]);
        let a = Archive::new(z.clone()).unwrap();
        assert_eq!(a.entries().len(), 3);
        assert_eq!(a.find("a.txt").unwrap().method, Method::Stored);
        assert!(a.find("d/").unwrap().is_dir());
        assert_eq!(a.read_by_name("a.txt").unwrap(), b"hello");
        assert_eq!(a.read_by_name("d/b.bin").unwrap(), [1, 2, 3]);
        assert_eq!(a.read_by_name("c"), Err(ZipError::NotFound("c".into())));
        assert_eq!(Archive::new(b"not a zip".to_vec()).err(), Some(ZipError::NotZip));

        let mut bad = z;
        let at = bad.iter().position(|&b| b == b'h').unwrap();
        bad[at] = b'j';
        let a = Archive::new(bad).unwrap();
        assert_eq!(a.read_by_name("a.txt"), Err(ZipError::Crc { name: "a.txt".into() }));
    }

    #[test]
    fn a_limit_refuses_an_entry_before_it_is_read() {
        let a = Archive::new(stored_zip(&[("big", &[7u8; 100])])).unwrap();
        assert!(matches!(a.read_by_name_with_limit("big", 50), Err(ZipError::Unsupported(_))));
        assert_eq!(a.read_by_name_with_limit("big", 100).unwrap().len(), 100);
    }
}
