//! Phase A: the two SolidWorks file envelopes, as a list of named streams.
//!
//! Version detection (first match wins):
//!
//! 1. bytes 0..8 = `D0 CF 11 E0 A1 B1 1A E1`: OLE2 / Compound File Binary (SolidWorks 2014 and older; also
//!    tested with 2011 files). Streams are CFB streams (path-joined names such as `Contents/Config-0-Partition`);
//!    the Parasolid transmit inside is a GUID-wrapped zlib frame (see [`Container::transmits`]).
//! 2. otherwise, if the block marker `14 00 06 00 08 00` occurs: the modern block container (SolidWorks 2015+;
//!    bytes 4..8 are the big-endian version word `00 00 00 04`). Each block is
//!    `marker | type u32 | crc32 u32 | comp u32 | uncomp u32 | name_len u32 | name | raw DEFLATE`, the CRC-32 is of
//!    the inflated payload, the name is stored with its nibbles swapped. Fixed-size "cache cells" between blocks reuse the
//!    marker but are not payloads; a hit is accepted only if the sizes are plausible, the DEFLATE data inflates
//!    to exactly `uncomp` bytes and the CRC matches, so false marker hits are dropped (counted in `skipped_hits`).
//! 3. anything else: `container.format` error.
//!
//! Facts about the layouts are from the open sldprt-format-research (MIT), cadmpeg (Apache-2.0) and sldprt2step
//! (Apache-2.0) descriptions; see THIRD_PARTY.md. The code is our own.

use crate::error::{Limits, SldprtError, R};
use crate::inflate::{crc32, inflate_raw, inflate_zlib};

pub const MARKER: [u8; 6] = [0x14, 0x00, 0x06, 0x00, 0x08, 0x00];
pub const OLE_MAGIC: [u8; 8] = [0xD0, 0xCF, 0x11, 0xE0, 0xA1, 0xB1, 0x1A, 0xE1];
/// GUID (231dd571-da81-48a2-a858-98b21b89ef99, byte order as stored) that introduces a wrapped Parasolid transmit.
pub const BLOB_GUID: [u8; 16] = [0x23, 0x1d, 0xd5, 0x71, 0xda, 0x81, 0x48, 0xa2, 0xa8, 0x58, 0x98, 0xb2, 0x1b, 0x89, 0xef, 0x99];

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Kind {
    /// SolidWorks 2015 and later.
    Modern,
    /// OLE2 / CFB, SolidWorks 2014 and earlier.
    Ole2,
}

#[derive(Debug, Clone)]
pub struct Stream {
    pub name: String,
    pub data: Vec<u8>,
    /// Modern container: the CRC-32 stored in the block header matched the inflated bytes.
    pub crc_ok: bool,
}

#[derive(Debug, Clone)]
pub struct Container {
    pub kind: Kind,
    pub streams: Vec<Stream>,
    /// Marker hits that were not blocks (cache cells, chance matches).
    pub skipped_hits: usize,
    pub warnings: Vec<String>,
    /// Modern container: blocks that are there but cannot be trusted (see [`Damage`]). Empty for a sound file.
    pub damaged: Vec<Damage>,
}

/// A block of the modern container that is damaged or lost: the file was cut short, or bytes inside it changed.
#[derive(Debug, Clone, PartialEq)]
pub struct Damage {
    /// Stream name of the block.
    pub name: String,
    /// What is wrong with it.
    pub why: String,
}

impl Damage {
    /// A block the geometry comes from (the live or ghost partition, the stock bodies, the local bodies of a feature, the
    /// inserted-part transmits): without it the file cannot be read as the model it was.
    pub fn is_geometry(&self) -> bool {
        let n = self.name.as_str();
        n.contains("Partition") || n.contains("StockBodies") || n.contains("LocalBodies") || n.contains("Deltas")
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TransmitKind {
    Partition,
    Deltas,
    Part,
    Other,
}

#[derive(Debug, Clone)]
pub struct Transmit {
    pub stream: String,
    pub kind: TransmitKind,
    /// UTF-16 body name stored just before the blob (imported bodies), if any.
    pub name: String,
    pub data: Vec<u8>,
}

fn u32le(d: &[u8], p: usize) -> Option<u32> {
    d.get(p..p.checked_add(4)?).map(|b| u32::from_le_bytes([b[0], b[1], b[2], b[3]]))
}

fn find(hay: &[u8], needle: &[u8], from: usize) -> Option<usize> {
    if hay.len() < needle.len() || from > hay.len() - needle.len() {
        return None;
    }
    let first = needle[0];
    let mut i = from;
    let last = hay.len() - needle.len();
    while i <= last {
        match hay[i..=last].iter().position(|&b| b == first) {
            None => return None,
            Some(k) => {
                i += k;
                if &hay[i..i + needle.len()] == needle {
                    return Some(i);
                }
                i += 1;
            }
        }
    }
    None
}

/// Detect the envelope without reading it.
pub fn detect(bytes: &[u8]) -> Option<Kind> {
    if bytes.len() >= 8 && bytes[..8] == OLE_MAGIC {
        Some(Kind::Ole2)
    } else if find(bytes, &MARKER, 0).is_some() {
        Some(Kind::Modern)
    } else {
        None
    }
}

/// Read every stream of a part file.
pub fn open(bytes: &[u8], limits: &Limits) -> R<Container> {
    if bytes.len() > limits.max_file {
        return Err(SldprtError::new("limit.file", format!("file is {} bytes, limit {}", bytes.len(), limits.max_file)));
    }
    match detect(bytes) {
        Some(Kind::Ole2) => open_ole2(bytes, limits),
        Some(Kind::Modern) => open_modern(bytes, limits),
        None => Err(SldprtError::new("container.format", "neither an OLE2 compound file nor a SolidWorks block container")),
    }
}

fn unswap_name(raw: &[u8]) -> String {
    raw.iter().map(|&b| (b << 4) | (b >> 4)).map(|b| if (0x20..0x7f).contains(&b) { b as char } else { '?' }).collect()
}

/// A block name as the container writes it: printable ASCII only (anything else means the header is not one).
fn plausible_name(raw: &[u8]) -> bool {
    !raw.is_empty() && raw.iter().map(|&b| (b << 4) | (b >> 4)).all(|b| (0x20..0x7f).contains(&b))
}

fn open_modern(buf: &[u8], limits: &Limits) -> R<Container> {
    let mut streams: Vec<Stream> = vec![];
    let mut skipped = 0usize;
    let mut total = 0usize;
    let mut warnings = vec![];
    let mut idx = 0usize;
    // blocks whose header is well formed but whose data does not check out (the cache cells that sit between the blocks
    // reuse the marker, never inflate and have `uncomp` twice `comp` or one more: they are not blocks)
    let mut bad: Vec<Damage> = vec![];
    while let Some(at) = find(buf, &MARKER, idx) {
        idx = at + 1;
        let (Some(crc), Some(csz), Some(dsz), Some(nl)) = (u32le(buf, at + 10), u32le(buf, at + 14), u32le(buf, at + 18), u32le(buf, at + 22)) else {
            skipped += 1;
            continue;
        };
        let (csz, dsz, nl) = (csz as usize, dsz as usize, nl as usize);
        let start = at + 26 + nl;
        if nl == 0 || nl >= 1024 || csz == 0 || dsz > limits.max_stream || at + 26 + nl > buf.len() {
            skipped += 1;
            continue;
        }
        let name_raw = &buf[at + 26..at + 26 + nl];
        let clean = plausible_name(name_raw);
        let cell = dsz == csz.saturating_mul(2) || dsz == csz.saturating_mul(2).saturating_add(1);
        if start.checked_add(csz).map_or(true, |e| e > buf.len()) {
            if clean && !cell {
                bad.push(Damage { name: unswap_name(name_raw), why: format!("the block is cut off: {csz} bytes are announced, {} are left in the file", buf.len() - start) });
            }
            skipped += 1;
            continue;
        }
        let inflated = inflate_raw(&buf[start..start + csz], dsz.max(1));
        let why = match &inflated {
            Err(_) => Some("the compressed data is damaged".to_string()),
            Ok((data, _)) if data.len() != dsz => Some(format!("it inflates to {} bytes, {dsz} are recorded", data.len())),
            Ok((data, _)) if crc32(data) != crc => Some("the CRC-32 of its data does not match the recorded one".to_string()),
            Ok(_) => None,
        };
        if let Some(why) = why {
            if clean {
                if !cell {
                    bad.push(Damage { name: unswap_name(name_raw), why });
                }
            }
            skipped += 1;
            continue;
        }
        let Ok((data, _)) = inflated else { continue };
        total += data.len();
        if total > limits.max_total {
            return Err(SldprtError::new("limit.total", "total inflated size exceeds the limit"));
        }
        if streams.len() >= limits.max_streams {
            return Err(SldprtError::new("limit.streams", "too many streams"));
        }
        streams.push(Stream { name: unswap_name(name_raw), data, crc_ok: true });
        idx = start + csz;
    }
    // a damaged block counts only when no sound block of the same name exists (stale copies of a stream are left behind)
    let mut damaged: Vec<Damage> = vec![];
    for d in bad {
        if !streams.iter().any(|s| s.name == d.name) && !damaged.iter().any(|x| x.name == d.name) {
            damaged.push(d);
        }
    }
    if streams.is_empty() {
        return Err(SldprtError::new("container.empty", "block marker found but no block passed the size and CRC checks"));
    }
    if buf.len() >= 8 && buf[4..8] != [0, 0, 0, 4] {
        warnings.push("unexpected container version word (not 00 00 00 04)".into());
    }
    Ok(Container { kind: Kind::Modern, streams, skipped_hits: skipped, warnings, damaged })
}

// ------------------------------------------------------------------ OLE2 / CFB

struct Cfb<'a> {
    buf: &'a [u8],
    ss: usize,
    fat: Vec<u32>,
    mini_fat: Vec<u32>,
    mini_cutoff: usize,
    mini_ss: usize,
    mini_stream: Vec<u8>,
}

const FREE: u32 = 0xFFFF_FFFF;
const ENDCHAIN: u32 = 0xFFFF_FFFE;

impl<'a> Cfb<'a> {
    fn sector(&self, s: u32) -> Option<&'a [u8]> {
        let off = (s as usize).checked_add(1)?.checked_mul(self.ss)?;
        self.buf.get(off..off.checked_add(self.ss)?)
    }

    fn chain(&self, start: u32, max_bytes: usize) -> R<Vec<u8>> {
        let mut out = vec![];
        let mut s = start;
        let mut hops = 0usize;
        while s < 0xFFFF_FFFA {
            hops += 1;
            if hops > self.fat.len() + 1 {
                return Err(SldprtError::new("container.cfb", "cyclic FAT chain"));
            }
            let sec = self.sector(s).ok_or_else(|| SldprtError::new("container.cfb", "sector beyond end of file"))?;
            if out.len() + sec.len() > max_bytes.saturating_add(self.ss) {
                return Err(SldprtError::new("limit.stream", "CFB stream exceeds the limit"));
            }
            out.extend_from_slice(sec);
            s = *self.fat.get(s as usize).unwrap_or(&ENDCHAIN);
        }
        Ok(out)
    }

    fn mini_chain(&self, start: u32, size: usize) -> R<Vec<u8>> {
        let mut out = vec![];
        let mut s = start;
        let mut hops = 0usize;
        while s < 0xFFFF_FFFA {
            hops += 1;
            if hops > self.mini_fat.len() + 1 {
                return Err(SldprtError::new("container.cfb", "cyclic miniFAT chain"));
            }
            let off = (s as usize) * self.mini_ss;
            let sec = self.mini_stream.get(off..off + self.mini_ss).ok_or_else(|| SldprtError::new("container.cfb", "mini sector beyond mini stream"))?;
            out.extend_from_slice(sec);
            s = *self.mini_fat.get(s as usize).unwrap_or(&ENDCHAIN);
            if out.len() > size + self.mini_ss {
                break;
            }
        }
        Ok(out)
    }
}

struct DirEnt {
    name: String,
    ty: u8,
    left: u32,
    right: u32,
    child: u32,
    start: u32,
    size: u64,
}

fn open_ole2(buf: &[u8], limits: &Limits) -> R<Container> {
    let bad = |m: &str| SldprtError::new("container.cfb", m.to_string());
    if buf.len() < 512 {
        return Err(bad("file shorter than a CFB header"));
    }
    let u16h = |p: usize| u16::from_le_bytes([buf[p], buf[p + 1]]);
    let u32h = |p: usize| u32::from_le_bytes([buf[p], buf[p + 1], buf[p + 2], buf[p + 3]]);
    let ss_shift = u16h(30) as u32;
    let mini_shift = u16h(32) as u32;
    if !(7..=16).contains(&ss_shift) || mini_shift > 12 || mini_shift == 0 {
        return Err(bad("unsupported sector shift"));
    }
    let ss = 1usize << ss_shift;
    let mut cfb = Cfb { buf, ss, fat: vec![], mini_fat: vec![], mini_cutoff: u32h(56) as usize, mini_ss: 1usize << mini_shift, mini_stream: vec![] };
    // DIFAT -> FAT sectors
    let mut fat_secs: Vec<u32> = (0..109).map(|i| u32h(76 + 4 * i)).filter(|&s| s < 0xFFFF_FFFA).collect();
    let mut dsec = u32h(68);
    let mut hops = 0;
    while dsec < 0xFFFF_FFFA {
        hops += 1;
        if hops > 100_000 {
            return Err(bad("DIFAT chain too long"));
        }
        let sec = cfb.sector(dsec).ok_or_else(|| bad("DIFAT sector beyond end of file"))?;
        let n = ss / 4 - 1;
        for i in 0..n {
            let s = u32::from_le_bytes([sec[4 * i], sec[4 * i + 1], sec[4 * i + 2], sec[4 * i + 3]]);
            if s < 0xFFFF_FFFA {
                fat_secs.push(s);
            }
        }
        dsec = u32::from_le_bytes([sec[ss - 4], sec[ss - 3], sec[ss - 2], sec[ss - 1]]);
    }
    if fat_secs.len() > buf.len() / ss + 1 {
        return Err(bad("more FAT sectors than the file can hold"));
    }
    for s in fat_secs {
        let sec = cfb.sector(s).ok_or_else(|| bad("FAT sector beyond end of file"))?;
        for i in 0..ss / 4 {
            cfb.fat.push(u32::from_le_bytes([sec[4 * i], sec[4 * i + 1], sec[4 * i + 2], sec[4 * i + 3]]));
        }
    }
    // directory
    let dir_bytes = cfb.chain(u32h(48), limits.max_stream)?;
    let mut ents: Vec<DirEnt> = vec![];
    for e in dir_bytes.chunks_exact(128) {
        let nl = u16::from_le_bytes([e[64], e[65]]) as usize;
        let nb = nl.saturating_sub(2).min(62);
        let units: Vec<u16> = e[..nb].chunks_exact(2).map(|c| u16::from_le_bytes([c[0], c[1]])).collect();
        let name = String::from_utf16_lossy(&units);
        let r32 = |p: usize| u32::from_le_bytes([e[p], e[p + 1], e[p + 2], e[p + 3]]);
        ents.push(DirEnt { name, ty: e[66], left: r32(68), right: r32(72), child: r32(76), start: r32(116), size: r32(120) as u64 });
        if ents.len() > 1_000_000 {
            return Err(bad("too many directory entries"));
        }
    }
    let root = ents.first().filter(|r| r.ty == 5).ok_or_else(|| bad("no root directory entry"))?;
    // mini FAT + mini stream
    let mf_start = u32h(60);
    if mf_start < 0xFFFF_FFFA {
        let mf = cfb.chain(mf_start, limits.max_stream)?;
        cfb.mini_fat = mf.chunks_exact(4).map(|c| u32::from_le_bytes([c[0], c[1], c[2], c[3]])).collect();
        let mut ms = cfb.chain(root.start, limits.max_stream)?;
        ms.truncate(root.size as usize);
        cfb.mini_stream = ms;
    }
    // tree walk (iterative), building path names
    let mut streams = vec![];
    let mut total = 0usize;
    let mut warnings = vec![];
    let mut stack: Vec<(u32, String)> = vec![(root.child, String::new())];
    let mut visited = vec![false; ents.len()];
    while let Some((id, prefix)) = stack.pop() {
        if id == FREE || id as usize >= ents.len() {
            continue;
        }
        if visited[id as usize] {
            return Err(bad("cycle in the directory tree"));
        }
        visited[id as usize] = true;
        let e = &ents[id as usize];
        stack.push((e.left, prefix.clone()));
        stack.push((e.right, prefix.clone()));
        let path = if prefix.is_empty() { e.name.clone() } else { format!("{prefix}/{}", e.name) };
        match e.ty {
            1 => stack.push((e.child, path)),
            2 => {
                let size = e.size as usize;
                if size > limits.max_stream {
                    warnings.push(format!("stream {path} skipped: {size} bytes exceeds the limit"));
                    continue;
                }
                if streams.len() >= limits.max_streams {
                    return Err(SldprtError::new("limit.streams", "too many streams"));
                }
                let data = if size < cfb.mini_cutoff { cfb.mini_chain(e.start, size) } else { cfb.chain(e.start, size) };
                match data {
                    Ok(mut d) => {
                        d.truncate(size);
                        total += d.len();
                        if total > limits.max_total {
                            return Err(SldprtError::new("limit.total", "total stream size exceeds the limit"));
                        }
                        streams.push(Stream { name: path, data: d, crc_ok: true });
                    }
                    Err(er) => warnings.push(format!("stream {path} unreadable: {er}")),
                }
            }
            _ => {}
        }
    }
    streams.sort_by(|a, b| a.name.cmp(&b.name));
    Ok(Container { kind: Kind::Ole2, streams, skipped_hits: 0, warnings, damaged: vec![] })
}

// ------------------------------------------------------------------ Parasolid blobs

/// The GUID-wrapped zlib frame whose GUID starts at `h`: `[u32 size] | GUID | u32 inflated | u32 deflated | zlib`, then
/// continuation chunks `u32 inflated | u32 deflated | zlib`. The size word before the GUID is read when `h >= 4`.
/// `Ok(None)` when nothing inflates; only a limit is an error.
fn decode_blob(d: &[u8], h: usize, limits: &Limits) -> R<Option<Vec<u8>>> {
    if h + 24 > d.len() {
        return Ok(None);
    }
    let blob_size = if h >= 4 { u32le(d, h - 4).unwrap_or(0) as usize } else { 0 };
    let (Some(_dsz), Some(csz)) = (u32le(d, h + 16), u32le(d, h + 20)) else { return Ok(None) };
    let limit = if blob_size != 0 { d.len().min(h.saturating_add(blob_size).saturating_add(8)) } else { d.len() };
    let mut pos = h + 24;
    let mut csz = csz as usize;
    let mut payload: Vec<u8> = vec![];
    loop {
        if csz == 0 || pos.checked_add(csz).map_or(true, |e| e > d.len()) {
            break;
        }
        match inflate_zlib(&d[pos..pos + csz], limits.max_stream.saturating_sub(payload.len())) {
            Ok((chunk, _)) => payload.extend_from_slice(&chunk),
            Err(e) if e.code.starts_with("limit.") => return Err(e),
            Err(_) => break,
        }
        pos += csz;
        if pos + 9 > limit || pos + 9 > d.len() {
            break;
        }
        let next = u32le(d, pos + 4).unwrap_or(0) as usize;
        if next == 0 || pos + 8 + next > d.len() || d[pos + 8] != 0x78 {
            break;
        }
        csz = next;
        pos += 8;
    }
    Ok(if payload.is_empty() { None } else { Some(payload) })
}

impl Container {
    pub fn stream(&self, name: &str) -> Option<&Stream> {
        self.streams.iter().find(|s| s.name == name || s.name.ends_with(&format!("/{name}")))
    }

    /// The bytes of stream `name`, unwrapped: OLE2 files store some streams as `<name>__ZLB`, a GUID-wrapped zlib frame
    /// (the same wrapping as the Parasolid transmits), which is inflated here. `None` when the stream is missing or its
    /// frame does not inflate.
    pub fn data(&self, name: &str, limits: &Limits) -> Option<Vec<u8>> {
        if let Some(s) = self.stream(name) {
            return Some(s.data.clone());
        }
        let d = &self.stream(&format!("{name}__ZLB"))?.data;
        let h = if d.get(4..20) == Some(&BLOB_GUID[..]) { 4 } else if d.get(..16) == Some(&BLOB_GUID[..]) { 0 } else { return None };
        decode_blob(d, h, limits).ok().flatten()
    }

    /// Every GUID-wrapped Parasolid transmit in any stream, inflated. A blob is
    /// `u32 size | GUID | u32 inflated | u32 deflated | zlib` followed by continuation chunks `u32 inflated | u32 deflated | zlib`.
    pub fn transmits(&self, limits: &Limits) -> R<Vec<Transmit>> {
        let mut out = vec![];
        let mut total = 0usize;
        for s in &self.streams {
            let d = &s.data;
            let mut from = 0usize;
            while let Some(h) = find(d, &BLOB_GUID, from) {
                from = h + 1;
                if h < 4 {
                    continue;
                }
                let Some(payload) = decode_blob(d, h, limits)? else { continue };
                total += payload.len();
                if total > limits.max_total {
                    return Err(SldprtError::new("limit.total", "total transmit size exceeds the limit"));
                }
                let head = &payload[..payload.len().min(80)];
                let has = |n: &[u8]| find(head, n, 0).is_some();
                let kind = if !payload.starts_with(b"PS") {
                    TransmitKind::Other
                } else if has(b"(partition)") {
                    TransmitKind::Partition
                } else if has(b"(deltas)") {
                    TransmitKind::Deltas
                } else if has(b"TRANSMIT FILE created") {
                    TransmitKind::Part
                } else {
                    TransmitKind::Other
                };
                // optional name record `ff fe ff <len> <utf16>` within 200 bytes before the blob
                let win = &d[h.saturating_sub(200)..h];
                let mut name = String::new();
                let mut k = 0;
                while k + 4 <= win.len() {
                    if win[k] == 0xff && win[k + 1] == 0xfe && win[k + 2] == 0xff {
                        let n = win[k + 3] as usize;
                        if let Some(raw) = win.get(k + 4..k + 4 + 2 * n) {
                            let units: Vec<u16> = raw.chunks_exact(2).map(|c| u16::from_le_bytes([c[0], c[1]])).collect();
                            name = String::from_utf16_lossy(&units);
                        }
                    }
                    k += 1;
                }
                out.push(Transmit { stream: s.name.clone(), kind, name, data: payload });
            }
        }
        Ok(out)
    }
}
