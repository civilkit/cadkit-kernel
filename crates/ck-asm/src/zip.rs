//! A small zip reader for the container of `.f3d` and `.f3z` files: the central directory, stored
//! (0), deflate (8) and Zstandard (93) entries. Sizes are checked before anything is allocated
//! and every entry's CRC-32 is verified. No ZIP64, no encryption, no multi-disk archives. The
//! decoders are this crate's own (no dependencies); nothing here touches the file system.
//!
//! Fusion writes Zstandard entries (method 93) on Windows and deflate ones elsewhere.

use crate::inflate::{crc32, inflate_raw};

/// A zip read failure: a stable code and what went wrong.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ZipError {
    pub code: &'static str,
    pub message: String,
}

impl ZipError {
    pub fn new(code: &'static str, message: impl Into<String>) -> ZipError {
        ZipError {
            code,
            message: message.into(),
        }
    }
}

impl std::fmt::Display for ZipError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}: {}", self.code, self.message)
    }
}

impl std::error::Error for ZipError {}

pub type R<T> = Result<T, ZipError>;

/// The largest entry (declared size) that is read: body blobs and the documents of `.f3z`
/// packages are the large ones.
pub const MAX_ENTRY: usize = 512 << 20;

/// The most entries of an archive.
pub const MAX_ENTRIES: usize = 100_000;

#[derive(Debug, Clone)]
pub struct Entry {
    pub name: String,
    pub method: u16,
    pub compressed_size: usize,
    pub size: usize,
    pub crc: u32,
    local_off: usize,
}

/// An opened archive (the bytes stay borrowed).
pub struct Archive<'a> {
    data: &'a [u8],
    entries: Vec<Entry>,
}

fn u16_at(d: &[u8], o: usize) -> Option<u16> {
    Some(u16::from_le_bytes(d.get(o..o.checked_add(2)?)?.try_into().ok()?))
}

fn u32_at(d: &[u8], o: usize) -> Option<u32> {
    Some(u32::from_le_bytes(d.get(o..o.checked_add(4)?)?.try_into().ok()?))
}

impl<'a> Archive<'a> {
    pub fn new(data: &'a [u8]) -> R<Archive<'a>> {
        // The end-of-central-directory record: in the last 22 + 65535 bytes.
        let lo = data.len().saturating_sub(22 + 65535);
        let mut i = data.len().saturating_sub(22);
        let eocd = loop {
            if u32_at(data, i) == Some(0x0605_4B50) {
                break i;
            }
            if i <= lo {
                return Err(ZipError::new(
                    "zip.no_directory",
                    "not a zip file (no end-of-central-directory record)",
                ));
            }
            i -= 1;
        };
        let n = u16_at(data, eocd + 10).unwrap_or(0) as usize;
        let cd_size = u32_at(data, eocd + 12).unwrap_or(0) as usize;
        let cd_off = u32_at(data, eocd + 16).unwrap_or(0) as usize;
        if n == 0xFFFF || cd_off == 0xFFFF_FFFF {
            return Err(ZipError::new(
                "zip.unsupported",
                "ZIP64 archives are not supported",
            ));
        }
        if n > MAX_ENTRIES {
            return Err(ZipError::new("zip.limit", format!("{n} entries")));
        }
        if cd_off.checked_add(cd_size).is_none_or(|end| end > data.len()) {
            return Err(ZipError::new(
                "zip.bad_directory",
                "the central directory lies outside the file",
            ));
        }
        let mut entries = Vec::with_capacity(n.min(4096));
        let mut p = cd_off;
        for _ in 0..n {
            if u32_at(data, p) != Some(0x0201_4B50) {
                return Err(ZipError::new(
                    "zip.bad_directory",
                    "bad central directory entry",
                ));
            }
            let trunc = || ZipError::new("zip.bad_directory", "truncated central directory entry");
            let method = u16_at(data, p + 10).ok_or_else(trunc)?;
            let crc = u32_at(data, p + 16).ok_or_else(trunc)?;
            let csz = u32_at(data, p + 20).ok_or_else(trunc)? as usize;
            let usz = u32_at(data, p + 24).ok_or_else(trunc)? as usize;
            let nl = u16_at(data, p + 28).ok_or_else(trunc)? as usize;
            let el = u16_at(data, p + 30).ok_or_else(trunc)? as usize;
            let cl = u16_at(data, p + 32).ok_or_else(trunc)? as usize;
            let local_off = u32_at(data, p + 42).ok_or_else(trunc)? as usize;
            let name_end = p + 46 + nl;
            let name = data.get(p + 46..name_end).ok_or_else(trunc)?;
            entries.push(Entry {
                name: String::from_utf8_lossy(name).into_owned(),
                method,
                compressed_size: csz,
                size: usz,
                crc,
                local_off,
            });
            p = name_end + el + cl;
        }
        Ok(Archive { data, entries })
    }

    pub fn entries(&self) -> &[Entry] {
        &self.entries
    }

    pub fn find(&self, name: &str) -> Option<&Entry> {
        self.entries.iter().find(|e| e.name == name)
    }

    /// One entry decompressed and checked (size and CRC-32).
    pub fn read(&self, e: &Entry) -> R<Vec<u8>> {
        if e.size > MAX_ENTRY {
            return Err(ZipError::new(
                "zip.limit",
                format!("'{}' declares {} bytes, over the limit of {MAX_ENTRY}", e.name, e.size),
            ));
        }
        let lh = e.local_off;
        if u32_at(self.data, lh) != Some(0x0403_4B50) {
            return Err(ZipError::new(
                "zip.bad_local",
                format!("bad local header for '{}'", e.name),
            ));
        }
        let (Some(nl), Some(el)) = (u16_at(self.data, lh + 26), u16_at(self.data, lh + 28)) else {
            return Err(ZipError::new("zip.bad_local", "truncated local header"));
        };
        let start = lh + 30 + nl as usize + el as usize;
        let end = start
            .checked_add(e.compressed_size)
            .filter(|&x| x <= self.data.len())
            .ok_or_else(|| {
                ZipError::new(
                    "zip.truncated",
                    format!("the data of '{}' runs past the end of the file", e.name),
                )
            })?;
        let raw = &self.data[start..end];
        let out = match e.method {
            0 => {
                if e.compressed_size != e.size {
                    return Err(ZipError::new(
                        "zip.bad_local",
                        "stored entry with different sizes",
                    ));
                }
                raw.to_vec()
            }
            8 => inflate_raw(raw, e.size)?.0,
            93 => crate::zstd::decompress(raw, e.size)?,
            m => {
                return Err(ZipError::new(
                    "zip.unsupported",
                    format!("compression method {m} of '{}' is not supported", e.name),
                ));
            }
        };
        if out.len() != e.size {
            return Err(ZipError::new(
                "zip.size_mismatch",
                format!("'{}' is {} bytes, declared {}", e.name, out.len(), e.size),
            ));
        }
        if crc32(&out) != e.crc {
            return Err(ZipError::new(
                "zip.crc",
                format!("CRC mismatch in '{}'", e.name),
            ));
        }
        Ok(out)
    }

    pub fn read_by_name(&self, name: &str) -> R<Vec<u8>> {
        let e = self
            .find(name)
            .ok_or_else(|| ZipError::new("zip.missing", format!("no entry '{name}'")))?;
        self.read(e)
    }
}

/// A zip of stored entries (tests). Entries are `(name, data)`.
#[doc(hidden)]
pub fn stored_zip(entries: &[(&str, &[u8])]) -> Vec<u8> {
    let mut out = Vec::new();
    let mut cd = Vec::new();
    for (name, data) in entries {
        let off = out.len() as u32;
        let crc = crc32(data);
        out.extend_from_slice(&0x0403_4B50u32.to_le_bytes());
        out.extend_from_slice(&[20, 0, 0, 0, 0, 0, 0, 0, 0, 0]);
        out.extend_from_slice(&crc.to_le_bytes());
        out.extend_from_slice(&(data.len() as u32).to_le_bytes());
        out.extend_from_slice(&(data.len() as u32).to_le_bytes());
        out.extend_from_slice(&(name.len() as u16).to_le_bytes());
        out.extend_from_slice(&0u16.to_le_bytes());
        out.extend_from_slice(name.as_bytes());
        out.extend_from_slice(data);
        cd.extend_from_slice(&0x0201_4B50u32.to_le_bytes());
        cd.extend_from_slice(&[20, 0, 20, 0, 0, 0, 0, 0, 0, 0, 0, 0]);
        cd.extend_from_slice(&crc.to_le_bytes());
        cd.extend_from_slice(&(data.len() as u32).to_le_bytes());
        cd.extend_from_slice(&(data.len() as u32).to_le_bytes());
        cd.extend_from_slice(&(name.len() as u16).to_le_bytes());
        cd.extend_from_slice(&[0; 12]);
        cd.extend_from_slice(&off.to_le_bytes());
        cd.extend_from_slice(name.as_bytes());
    }
    let cd_off = out.len() as u32;
    out.extend_from_slice(&cd);
    out.extend_from_slice(&0x0605_4B50u32.to_le_bytes());
    out.extend_from_slice(&[0, 0, 0, 0]);
    out.extend_from_slice(&(entries.len() as u16).to_le_bytes());
    out.extend_from_slice(&(entries.len() as u16).to_le_bytes());
    out.extend_from_slice(&(cd.len() as u32).to_le_bytes());
    out.extend_from_slice(&cd_off.to_le_bytes());
    out.extend_from_slice(&[0, 0]);
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn reads_stored_entries() {
        let z = stored_zip(&[("a.txt", b"hello"), ("d/b.bin", &[1, 2, 3])]);
        let a = Archive::new(&z).unwrap();
        assert_eq!(a.entries().len(), 2);
        assert_eq!(a.read_by_name("a.txt").unwrap(), b"hello");
        assert_eq!(a.read_by_name("d/b.bin").unwrap(), [1, 2, 3]);
        assert_eq!(a.read_by_name("c").unwrap_err().code, "zip.missing");
    }

    #[test]
    fn refuses_garbage_and_bad_crc() {
        assert!(Archive::new(b"not a zip").is_err());
        assert!(Archive::new(&[]).is_err());
        let mut z = stored_zip(&[("a", b"hello")]);
        let at = z.iter().position(|&b| b == b'h').unwrap();
        z[at] = b'j';
        let a = Archive::new(&z).unwrap();
        assert_eq!(a.read_by_name("a").unwrap_err().code, "zip.crc");
    }

    #[test]
    fn refuses_a_truncated_directory() {
        let z = stored_zip(&[("a", b"hello")]);
        for cut in 0..z.len() {
            // never panics, whatever is cut off
            let _ = Archive::new(&z[..cut]).and_then(|a| a.read_by_name("a"));
        }
    }
}
