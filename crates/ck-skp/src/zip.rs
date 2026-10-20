//! Minimal ZIP reader for the container embedded in VFF (2021+) files: central directory, stored and deflated entries,
//! sizes checked before any allocation, CRC-32 verified.
//!
//! Part of ck-skp, a Rust reimplementation of the SketchUp reader of OpenSKP (MIT, (c) Ahsan Mehmood), see NOTICE.
//! OpenSKP's ceiling and ratio checks on entry sizes (`_validate_zip_entry_size`) are mirrored in `Zip::read`.

use crate::error::{Limits, SkpError, R};
use crate::inflate::{crc32, inflate_raw};

#[derive(Debug, Clone)]
pub struct ZipEntry {
    pub name: String,
    pub method: u16,
    pub comp_size: usize,
    pub size: usize,
    pub crc: u32,
    pub local_off: usize,
}

pub struct Zip<'a> {
    data: &'a [u8],
    pub entries: Vec<ZipEntry>,
}

fn u16_at(d: &[u8], o: usize) -> Option<u16> {
    Some(u16::from_le_bytes(d.get(o..o.checked_add(2)?)?.try_into().ok()?))
}
fn u32_at(d: &[u8], o: usize) -> Option<u32> {
    Some(u32::from_le_bytes(d.get(o..o.checked_add(4)?)?.try_into().ok()?))
}

/// Offset of the first local-file-header signature `PK\x03\x04` at or after `from`.
pub fn find_local_header(data: &[u8], from: usize) -> Option<usize> {
    let sig = [0x50u8, 0x4B, 0x03, 0x04];
    let mut i = from;
    while i + 4 <= data.len() {
        // memchr-like scan for 'P'
        match data[i..].iter().position(|&b| b == 0x50) {
            None => return None,
            Some(p) => i += p,
        }
        if data.get(i..i + 4) == Some(&sig[..]) {
            return Some(i);
        }
        i += 1;
    }
    None
}

impl<'a> Zip<'a> {
    /// `data` starts at the first local file header; offsets in the central directory are relative to it.
    pub fn open(data: &'a [u8], lim: &Limits) -> R<Zip<'a>> {
        // end of central directory record: last 22 + 65535 bytes
        let lo = data.len().saturating_sub(22 + 65535);
        let mut eocd = None;
        let mut i = data.len().saturating_sub(22);
        loop {
            if i < lo {
                break;
            }
            if u32_at(data, i) == Some(0x0605_4B50) {
                eocd = Some(i);
                break;
            }
            if i == 0 {
                break;
            }
            i -= 1;
        }
        let e = eocd.ok_or_else(|| SkpError::new("zip.no_directory", "no ZIP end-of-central-directory record"))?;
        let n = u16_at(data, e + 10).unwrap_or(0) as usize;
        let cd_size = u32_at(data, e + 12).unwrap_or(0) as usize;
        let cd_off = u32_at(data, e + 16).unwrap_or(0) as usize;
        if n == 0xFFFF || cd_off == 0xFFFF_FFFF {
            return Err(SkpError::at("zip.unsupported", e, "ZIP64 archives are not supported"));
        }
        if n > lim.max_entries {
            return Err(SkpError::at("limit.entries", e, format!("{n} ZIP entries exceed the limit of {}", lim.max_entries)));
        }
        if cd_off.checked_add(cd_size).map_or(true, |end| end > data.len()) {
            return Err(SkpError::at("zip.bad_directory", e, "central directory lies outside the file"));
        }
        let mut entries = Vec::with_capacity(n.min(4096));
        let mut p = cd_off;
        for _ in 0..n {
            if u32_at(data, p) != Some(0x0201_4B50) {
                return Err(SkpError::at("zip.bad_directory", p, "bad central directory entry signature"));
            }
            let (method, crc, csz, usz, nl, el, cl, lo) = (
                u16_at(data, p + 10), u32_at(data, p + 16), u32_at(data, p + 20), u32_at(data, p + 24), u16_at(data, p + 28), u16_at(data, p + 30), u16_at(data, p + 32), u32_at(data, p + 42),
            );
            let (Some(method), Some(crc), Some(csz), Some(usz), Some(nl), Some(el), Some(cl), Some(lo)) = (method, crc, csz, usz, nl, el, cl, lo) else {
                return Err(SkpError::at("zip.bad_directory", p, "truncated central directory entry"));
            };
            let name_end = p + 46 + nl as usize;
            let name = data.get(p + 46..name_end).ok_or_else(|| SkpError::at("zip.bad_directory", p, "entry name past the end"))?;
            entries.push(ZipEntry { name: String::from_utf8_lossy(name).into_owned(), method, comp_size: csz as usize, size: usz as usize, crc, local_off: lo as usize });
            p = name_end + el as usize + cl as usize;
        }
        Ok(Zip { data, entries })
    }

    pub fn find(&self, name: &str) -> Option<&ZipEntry> {
        self.entries.iter().find(|e| e.name == name)
    }

    /// Inflate one entry. Declared size above `lim.max_entry`, or an implausible ratio (>= 1 MiB declared from more
    /// than 1000 times fewer bytes), is refused before anything is allocated; the real output is capped at the
    /// declared size and its CRC is checked.
    pub fn read(&self, e: &ZipEntry, lim: &Limits) -> R<Vec<u8>> {
        if e.size > lim.max_entry {
            return Err(SkpError::at("limit.entry", e.local_off, format!("ZIP entry '{}' declares {} bytes, over the limit of {}", e.name, e.size, lim.max_entry)));
        }
        if e.size >= 1 << 20 && (e.comp_size == 0 || e.size / e.comp_size > 1000) {
            return Err(SkpError::at("limit.ratio", e.local_off, format!("ZIP entry '{}': {} bytes from {} compressed is an implausible ratio", e.name, e.size, e.comp_size)));
        }
        let lh = e.local_off;
        if u32_at(self.data, lh) != Some(0x0403_4B50) {
            return Err(SkpError::at("zip.bad_local", lh, format!("bad local header for '{}'", e.name)));
        }
        let (Some(nl), Some(el)) = (u16_at(self.data, lh + 26), u16_at(self.data, lh + 28)) else {
            return Err(SkpError::at("zip.bad_local", lh, "truncated local header"));
        };
        let start = lh + 30 + nl as usize + el as usize;
        let end = start.checked_add(e.comp_size).filter(|&x| x <= self.data.len()).ok_or_else(|| SkpError::at("zip.truncated", start, format!("data of '{}' runs past the end of the file", e.name)))?;
        let raw = &self.data[start..end];
        let out = match e.method {
            0 => {
                if e.comp_size != e.size {
                    return Err(SkpError::at("zip.bad_local", lh, "stored entry with different sizes"));
                }
                raw.to_vec()
            }
            8 => {
                let (o, _) = inflate_raw(raw, e.size)?;
                o
            }
            m => return Err(SkpError::at("zip.unsupported", lh, format!("compression method {m} of '{}' is not supported", e.name))),
        };
        if out.len() != e.size {
            return Err(SkpError::at("zip.size_mismatch", start, format!("'{}' inflated to {} bytes, declared {}", e.name, out.len(), e.size)));
        }
        if crc32(&out) != e.crc {
            return Err(SkpError::at("zip.crc", start, format!("CRC mismatch in '{}'", e.name)));
        }
        Ok(out)
    }
}
