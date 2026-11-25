//! Zstandard decompressor (RFC 8878), written from the format specification for this crate: no
//! dependencies, no unsafe, no dictionaries, no streaming. Fusion writes zip entries with
//! compression method 93 (Zstandard) on Windows; this is what reads them.
//!
//! Everything is bounds-checked and the output is capped: a corrupt or hostile stream gives an
//! error, never a panic or an allocation beyond `max_out`.
//!
//! The decoders follow the specification's own names: literals sections (raw, RLE, Huffman with
//! one or four streams, treeless), sequences sections (predefined, RLE, FSE-compressed and
//! repeated tables), repeated offsets and frame checksums (skipped: the zip entry's CRC-32 covers
//! the data).

use crate::zip::{R, ZipError};

fn bad(msg: &str) -> ZipError {
    ZipError::new("zstd.corrupt", msg)
}

const MAGIC: u32 = 0xFD2F_B528;
const SKIPPABLE: u32 = 0x184D_2A50;
const MAX_BLOCK: usize = 128 * 1024;

/// Decompresses every frame of `src` (skippable frames are skipped); the output is at most
/// `max_out` bytes.
pub fn decompress(src: &[u8], max_out: usize) -> R<Vec<u8>> {
    let mut out: Vec<u8> = Vec::new();
    let mut p = 0usize;
    while p < src.len() {
        let magic = u32_at(src, p)?;
        if magic & 0xFFFF_FFF0 == SKIPPABLE {
            let n = u32_at(src, p + 4)? as usize;
            p = p
                .checked_add(8)
                .and_then(|x| x.checked_add(n))
                .filter(|&x| x <= src.len())
                .ok_or_else(|| bad("skippable frame runs past the end"))?;
            continue;
        }
        if magic != MAGIC {
            return Err(ZipError::new("zstd.magic", "not a Zstandard frame"));
        }
        p = frame(src, p + 4, &mut out, max_out)?;
    }
    Ok(out)
}

fn u32_at(d: &[u8], o: usize) -> R<u32> {
    d.get(o..o + 4)
        .map(|b| u32::from_le_bytes([b[0], b[1], b[2], b[3]]))
        .ok_or_else(|| bad("truncated"))
}

fn slice(d: &[u8], a: usize, n: usize) -> R<&[u8]> {
    a.checked_add(n)
        .and_then(|e| d.get(a..e))
        .ok_or_else(|| bad("truncated"))
}

/// What a frame's blocks share: the Huffman table, the three FSE tables and the repeated offsets.
struct State {
    huf: Option<Huf>,
    ll: Option<Fse>,
    of: Option<Fse>,
    ml: Option<Fse>,
    rep: [usize; 3],
}

fn frame(src: &[u8], mut p: usize, out: &mut Vec<u8>, max_out: usize) -> R<usize> {
    let fd = *src.get(p).ok_or_else(|| bad("truncated frame header"))?;
    p += 1;
    if fd & 0x08 != 0 {
        return Err(bad("reserved frame header bit set"));
    }
    let fcs_flag = fd >> 6;
    let single = fd & 0x20 != 0;
    let checksum = fd & 0x04 != 0;
    if !single {
        p += 1; // window descriptor: the whole output is the window
    }
    let dict_bytes = [0usize, 1, 2, 4][(fd & 3) as usize];
    let dict_id = slice(src, p, dict_bytes)?;
    if dict_id.iter().any(|&b| b != 0) {
        return Err(ZipError::new(
            "zstd.dictionary",
            "frames that use a dictionary are not supported",
        ));
    }
    p += dict_bytes;
    let fcs_bytes = match fcs_flag {
        0 => usize::from(single),
        1 => 2,
        2 => 4,
        _ => 8,
    };
    let fcs = slice(src, p, fcs_bytes)?;
    p += fcs_bytes;
    if fcs_bytes > 0 {
        let mut v = 0u64;
        for (i, b) in fcs.iter().enumerate() {
            v |= u64::from(*b) << (8 * i);
        }
        if fcs_bytes == 2 {
            v += 256;
        }
        if v > max_out as u64 {
            return Err(ZipError::new(
                "zstd.limit",
                "the frame declares more output than the limit",
            ));
        }
        out.reserve((v as usize).min(1 << 28));
    }
    let start = out.len();
    let mut st = State {
        huf: None,
        ll: None,
        of: None,
        ml: None,
        rep: [1, 4, 8],
    };
    let mut lits: Vec<u8> = Vec::new();
    loop {
        let h = slice(src, p, 3)?;
        let h = u32::from(h[0]) | u32::from(h[1]) << 8 | u32::from(h[2]) << 16;
        p += 3;
        let last = h & 1 != 0;
        let size = (h >> 3) as usize;
        match (h >> 1) & 3 {
            0 => {
                let data = slice(src, p, size)?;
                if out.len() + size > max_out {
                    return Err(ZipError::new("zstd.limit", "output over the limit"));
                }
                out.extend_from_slice(data);
                p += size;
            }
            1 => {
                let b = *src.get(p).ok_or_else(|| bad("truncated"))?;
                if out.len() + size > max_out {
                    return Err(ZipError::new("zstd.limit", "output over the limit"));
                }
                out.resize(out.len() + size, b);
                p += 1;
            }
            2 => {
                if size > MAX_BLOCK {
                    return Err(bad("block over 128 KiB"));
                }
                let data = slice(src, p, size)?;
                block(data, &mut st, out, start, &mut lits, max_out)?;
                p += size;
            }
            _ => return Err(bad("reserved block type")),
        }
        if last {
            break;
        }
    }
    if checksum {
        slice(src, p, 4)?;
        p += 4;
    }
    Ok(p)
}

// ---- bit readers ----

/// A stream read backwards: the last byte holds a 1 bit above the data (the end mark); bits are
/// taken from the top down.
struct Rev<'a> {
    d: &'a [u8],
    /// Bits left (negative once more were taken than there were).
    pos: i64,
}

impl<'a> Rev<'a> {
    fn new(d: &'a [u8]) -> R<Rev<'a>> {
        let last = *d.last().ok_or_else(|| bad("empty bit stream"))?;
        if last == 0 {
            return Err(bad("bit stream without an end mark"));
        }
        let top = 7 - last.leading_zeros() as i64;
        Ok(Rev {
            d,
            pos: (d.len() as i64 - 1) * 8 + top,
        })
    }

    /// The 56 bits from bit index `at` (little-endian bit order), zeros past the end.
    fn raw(&self, at: i64) -> u64 {
        let byte = (at / 8) as usize;
        let mut w = [0u8; 8];
        for (i, slot) in w.iter_mut().enumerate() {
            if let Some(b) = self.d.get(byte + i) {
                *slot = *b;
            }
        }
        u64::from_le_bytes(w) >> (at % 8)
    }

    /// The next `n` (at most 32) bits without taking them; bits before the start read as zero.
    fn peek(&self, n: u32) -> u64 {
        if n == 0 {
            return 0;
        }
        let mask = (1u64 << n) - 1;
        let p = self.pos - i64::from(n);
        if p >= 0 {
            self.raw(p) & mask
        } else if self.pos <= 0 {
            0
        } else {
            ((self.raw(0) & ((1u64 << self.pos) - 1)) << (-p)) & mask
        }
    }

    fn take(&mut self, n: u32) {
        self.pos -= i64::from(n);
    }

    fn read(&mut self, n: u32) -> u64 {
        let v = self.peek(n);
        self.take(n);
        v
    }
}

/// A forward bit reader (least significant bit first) for table descriptions.
struct Fwd<'a> {
    d: &'a [u8],
    bit: usize,
}

impl Fwd<'_> {
    fn peek(&self, n: u32) -> u32 {
        let byte = self.bit / 8;
        let mut w = [0u8; 8];
        for (i, slot) in w.iter_mut().enumerate() {
            if let Some(b) = self.d.get(byte + i) {
                *slot = *b;
            }
        }
        ((u64::from_le_bytes(w) >> (self.bit % 8)) & ((1u64 << n) - 1)) as u32
    }

    fn read(&mut self, n: u32) -> u32 {
        let v = self.peek(n);
        self.bit += n as usize;
        v
    }
}

// ---- FSE ----

#[derive(Clone, Copy, Default)]
struct FseEntry {
    sym: u8,
    nb: u8,
    base: u16,
}

#[derive(Clone)]
struct Fse {
    al: u32,
    t: Vec<FseEntry>,
}

/// Builds the decoding table of a normalised distribution (`-1` = less than one).
fn fse_table(counts: &[i32], al: u32) -> R<Fse> {
    let size = 1usize << al;
    let mut t = vec![FseEntry::default(); size];
    let mut next = vec![0u32; counts.len()];
    let mut high = size - 1;
    for (s, &c) in counts.iter().enumerate() {
        if c == -1 {
            t[high].sym = s as u8;
            next[s] = 1;
            high = high.wrapping_sub(1);
        } else if c > 0 {
            next[s] = c as u32;
        }
    }
    let total: i64 = counts.iter().map(|&c| if c == -1 { 1 } else { i64::from(c) }).sum();
    if total != size as i64 {
        return Err(bad("FSE distribution does not fill its table"));
    }
    let step = (size >> 1) + (size >> 3) + 3;
    let mask = size - 1;
    let mut pos = 0usize;
    for (s, &c) in counts.iter().enumerate() {
        for _ in 0..c.max(0) {
            t[pos].sym = s as u8;
            loop {
                pos = (pos + step) & mask;
                if pos <= high || high == usize::MAX {
                    break;
                }
            }
        }
    }
    for e in t.iter_mut() {
        let s = e.sym as usize;
        let n = next[s];
        next[s] += 1;
        let nb = al - (31 - n.leading_zeros());
        e.nb = nb as u8;
        e.base = ((n << nb) as usize - size) as u16;
    }
    Ok(Fse { al, t })
}

/// Reads a table description; returns the table and the bytes it took.
fn read_fse(src: &[u8], max_al: u32, max_sym: usize) -> R<(Fse, usize)> {
    let mut r = Fwd { d: src, bit: 0 };
    let al = r.read(4) + 5;
    if al > max_al {
        return Err(bad("FSE accuracy log too high"));
    }
    let size = 1i32 << al;
    let mut remaining = size + 1;
    let mut threshold = size;
    let mut nbits = al + 1;
    let mut counts: Vec<i32> = Vec::new();
    let mut prev0 = false;
    while remaining > 1 && counts.len() <= max_sym {
        if prev0 {
            loop {
                let rep = r.read(2);
                counts.extend(std::iter::repeat_n(0, rep as usize));
                if rep != 3 {
                    break;
                }
            }
            if counts.len() > max_sym + 1 {
                return Err(bad("FSE symbol count too high"));
            }
        }
        let max = 2 * threshold - 1 - remaining;
        let low = r.peek(nbits) as i32;
        let count = if (low & (threshold - 1)) < max {
            r.bit += (nbits - 1) as usize;
            low & (threshold - 1)
        } else {
            r.bit += nbits as usize;
            if low >= threshold { low - max } else { low }
        };
        let count = count - 1;
        remaining -= count.abs();
        counts.push(count);
        prev0 = count == 0;
        while remaining < threshold {
            nbits -= 1;
            threshold >>= 1;
        }
        if r.bit > src.len() * 8 {
            return Err(bad("FSE table description runs past its bytes"));
        }
    }
    if remaining != 1 || counts.len() > max_sym + 1 {
        return Err(bad("bad FSE distribution"));
    }
    let used = r.bit.div_ceil(8);
    if used > src.len() {
        return Err(bad("FSE table description runs past its bytes"));
    }
    Ok((fse_table(&counts, al)?, used))
}

// ---- Huffman ----

struct Huf {
    max_bits: u32,
    /// Indexed by the next `max_bits` bits: symbol and bits to take.
    t: Vec<(u8, u8)>,
}

/// Reads a Huffman tree description; returns the table and the bytes it took.
fn read_huf(data: &[u8]) -> R<(Huf, usize)> {
    let h = *data.first().ok_or_else(|| bad("truncated"))? as usize;
    let mut w: Vec<u8> = Vec::new();
    let used;
    if h >= 128 {
        let n = h - 127;
        let bytes = slice(data, 1, n.div_ceil(2))?;
        for i in 0..n {
            let b = bytes[i / 2];
            w.push(if i % 2 == 0 { b >> 4 } else { b & 15 });
        }
        used = 1 + n.div_ceil(2);
    } else {
        let comp = slice(data, 1, h)?;
        let (tab, took) = read_fse(comp, 6, 12)?;
        let mut rb = Rev::new(&comp[took..])?;
        let mut s1 = rb.read(tab.al) as usize;
        let mut s2 = rb.read(tab.al) as usize;
        loop {
            let e = tab.t[s1];
            w.push(e.sym);
            s1 = e.base as usize + rb.read(u32::from(e.nb)) as usize;
            if rb.pos < 0 {
                w.push(tab.t[s2].sym);
                break;
            }
            let e = tab.t[s2];
            w.push(e.sym);
            s2 = e.base as usize + rb.read(u32::from(e.nb)) as usize;
            if rb.pos < 0 {
                w.push(tab.t[s1].sym);
                break;
            }
            if w.len() > 255 {
                return Err(bad("too many Huffman weights"));
            }
        }
        used = 1 + h;
    }
    if w.len() > 255 {
        return Err(bad("too many Huffman weights"));
    }
    let mut total = 0u32;
    for &x in &w {
        if x > 11 {
            return Err(bad("Huffman weight over 11"));
        }
        if x > 0 {
            total += 1 << (x - 1);
        }
    }
    if total == 0 {
        return Err(bad("empty Huffman tree"));
    }
    let max_bits = 32 - total.leading_zeros();
    let left = (1u32 << max_bits) - total;
    if max_bits > 11 || !left.is_power_of_two() {
        return Err(bad("Huffman weights do not complete a tree"));
    }
    w.push((left.trailing_zeros() + 1) as u8);
    let mut t = vec![(0u8, 0u8); 1 << max_bits];
    let mut cursor = 0usize;
    for wt in 1..=max_bits as u8 {
        let span = 1usize << (wt - 1);
        for (sym, &sw) in w.iter().enumerate() {
            if sw == wt {
                for e in &mut t[cursor..cursor + span] {
                    *e = (sym as u8, max_bits as u8 + 1 - wt);
                }
                cursor += span;
            }
        }
    }
    Ok((Huf { max_bits, t }, used))
}

fn huf_stream(stream: &[u8], n: usize, huf: &Huf, out: &mut Vec<u8>) -> R<()> {
    let mut rb = Rev::new(stream)?;
    for _ in 0..n {
        let (sym, nb) = huf.t[rb.peek(huf.max_bits) as usize];
        rb.take(u32::from(nb));
        out.push(sym);
    }
    if rb.pos < 0 {
        return Err(bad("Huffman stream overrun"));
    }
    Ok(())
}

// ---- blocks ----

const LL_DIST: [i32; 36] = [
    4, 3, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 3, 2, 1, 1, 1, 1, 1,
    -1, -1, -1, -1,
];
const ML_DIST: [i32; 53] = [
    1, 4, 3, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, -1, -1, -1, -1, -1, -1, -1,
];
const OF_DIST: [i32; 29] = [
    1, 1, 1, 1, 1, 1, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, -1, -1, -1, -1, -1,
];

const LL_BASE: [u32; 36] = [
    0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 18, 20, 22, 24, 28, 32, 40, 48, 64,
    128, 256, 512, 1024, 2048, 4096, 8192, 16384, 32768, 65536,
];
const LL_BITS: [u8; 36] = [
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 3, 3, 4, 6, 7, 8, 9, 10, 11,
    12, 13, 14, 15, 16,
];
const ML_BASE: [u32; 53] = [
    3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27,
    28, 29, 30, 31, 32, 33, 34, 35, 37, 39, 41, 43, 47, 51, 59, 67, 83, 99, 131, 259, 515, 1027,
    2051, 4099, 8195, 16387, 32771, 65539,
];
const ML_BITS: [u8; 53] = [
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 1, 1, 1, 1, 2, 2, 3, 3, 4, 4, 5, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16,
];

/// The table of one symbol type from its mode (0 predefined, 1 RLE, 2 FSE-compressed, 3 repeat).
fn seq_table(
    mode: u8,
    data: &[u8],
    p: &mut usize,
    prev: &mut Option<Fse>,
    predefined: (&[i32], u32),
    max_al: u32,
    max_sym: usize,
) -> R<()> {
    match mode {
        0 => *prev = Some(fse_table(predefined.0, predefined.1)?),
        1 => {
            let s = *data.get(*p).ok_or_else(|| bad("truncated"))?;
            *p += 1;
            if s as usize > max_sym {
                return Err(bad("RLE symbol out of range"));
            }
            *prev = Some(Fse {
                al: 0,
                t: vec![FseEntry { sym: s, nb: 0, base: 0 }],
            });
        }
        2 => {
            let (t, used) = read_fse(&data[(*p).min(data.len())..], max_al, max_sym)?;
            *p += used;
            *prev = Some(t);
        }
        _ => {
            if prev.is_none() {
                return Err(bad("repeated table before any table"));
            }
        }
    }
    Ok(())
}

fn block(
    data: &[u8],
    st: &mut State,
    out: &mut Vec<u8>,
    start: usize,
    lits: &mut Vec<u8>,
    max_out: usize,
) -> R<()> {
    // ---- literals section ----
    let b0 = *data.first().ok_or_else(|| bad("empty block"))?;
    let typ = b0 & 3;
    let sf = (b0 >> 2) & 3;
    let mut p;
    lits.clear();
    if typ < 2 {
        let (regen, hdr) = match sf {
            0 | 2 => ((b0 >> 3) as usize, 1),
            1 => {
                let h = slice(data, 0, 2)?;
                ((h[0] as usize >> 4) | (h[1] as usize) << 4, 2)
            }
            _ => {
                let h = slice(data, 0, 3)?;
                (
                    (h[0] as usize >> 4) | (h[1] as usize) << 4 | (h[2] as usize) << 12,
                    3,
                )
            }
        };
        if regen > MAX_BLOCK {
            return Err(bad("literals over 128 KiB"));
        }
        if typ == 0 {
            lits.extend_from_slice(slice(data, hdr, regen)?);
            p = hdr + regen;
        } else {
            let b = *data.get(hdr).ok_or_else(|| bad("truncated"))?;
            lits.resize(regen, b);
            p = hdr + 1;
        }
    } else {
        let (streams4, hdr, regen, comp) = match sf {
            0 | 1 => {
                let h = slice(data, 0, 3)?;
                let v = u32::from(h[0]) | u32::from(h[1]) << 8 | u32::from(h[2]) << 16;
                (sf == 1, 3, ((v >> 4) & 0x3FF) as usize, ((v >> 14) & 0x3FF) as usize)
            }
            2 => {
                let h = slice(data, 0, 4)?;
                let v = u32::from_le_bytes([h[0], h[1], h[2], h[3]]);
                (true, 4, ((v >> 4) & 0x3FFF) as usize, ((v >> 18) & 0x3FFF) as usize)
            }
            _ => {
                let h = slice(data, 0, 5)?;
                let v = u64::from(h[0])
                    | u64::from(h[1]) << 8
                    | u64::from(h[2]) << 16
                    | u64::from(h[3]) << 24
                    | u64::from(h[4]) << 32;
                (true, 5, ((v >> 4) & 0x3FFFF) as usize, ((v >> 22) & 0x3FFFF) as usize)
            }
        };
        if regen > MAX_BLOCK {
            return Err(bad("literals over 128 KiB"));
        }
        let body = slice(data, hdr, comp)?;
        let mut q = 0;
        if typ == 2 {
            let (h, used) = read_huf(body)?;
            st.huf = Some(h);
            q = used;
        }
        let huf = st
            .huf
            .as_ref()
            .ok_or_else(|| bad("treeless literals before any Huffman table"))?;
        let streams = &body[q..];
        if !streams4 {
            huf_stream(streams, regen, huf, lits)?;
        } else {
            let jump = slice(streams, 0, 6)?;
            let s1 = u16::from_le_bytes([jump[0], jump[1]]) as usize;
            let s2 = u16::from_le_bytes([jump[2], jump[3]]) as usize;
            let s3 = u16::from_le_bytes([jump[4], jump[5]]) as usize;
            let rest = &streams[6..];
            let s4 = rest
                .len()
                .checked_sub(s1 + s2 + s3)
                .ok_or_else(|| bad("Huffman jump table past the block"))?;
            let seg = regen.div_ceil(4);
            let last = regen
                .checked_sub(3 * seg)
                .ok_or_else(|| bad("literals too short for four streams"))?;
            let mut a = 0;
            for (len, n) in [(s1, seg), (s2, seg), (s3, seg), (s4, last)] {
                huf_stream(&rest[a..a + len], n, huf, lits)?;
                a += len;
            }
        }
        p = hdr + comp;
    }

    // ---- sequences section ----
    let b = *data.get(p).ok_or_else(|| bad("missing sequences section"))?;
    p += 1;
    let n = if b == 0 {
        0
    } else if b < 128 {
        b as usize
    } else if b < 255 {
        let b1 = *data.get(p).ok_or_else(|| bad("truncated"))? as usize;
        p += 1;
        ((b as usize - 128) << 8) + b1
    } else {
        let h = slice(data, p, 2)?;
        p += 2;
        h[0] as usize + ((h[1] as usize) << 8) + 0x7F00
    };
    if n == 0 {
        if out.len() + lits.len() > max_out {
            return Err(ZipError::new("zstd.limit", "output over the limit"));
        }
        out.extend_from_slice(lits);
        return Ok(());
    }
    let modes = *data.get(p).ok_or_else(|| bad("truncated"))?;
    p += 1;
    if modes & 3 != 0 {
        return Err(bad("reserved sequence mode bits"));
    }
    seq_table(modes >> 6, data, &mut p, &mut st.ll, (&LL_DIST, 6), 9, 35)?;
    seq_table((modes >> 4) & 3, data, &mut p, &mut st.of, (&OF_DIST, 5), 8, 31)?;
    seq_table((modes >> 2) & 3, data, &mut p, &mut st.ml, (&ML_DIST, 6), 9, 52)?;
    let (Some(ll_t), Some(of_t), Some(ml_t)) = (&st.ll, &st.of, &st.ml) else {
        return Err(bad("missing sequence table"));
    };
    let mut rb = Rev::new(&data[p.min(data.len())..])?;
    let mut ll_s = rb.read(ll_t.al) as usize;
    let mut of_s = rb.read(of_t.al) as usize;
    let mut ml_s = rb.read(ml_t.al) as usize;
    let mut lp = 0usize;
    for i in 0..n {
        let ofc = of_t.t[of_s].sym as u32;
        let mlc = ml_t.t[ml_s].sym as usize;
        let llc = ll_t.t[ll_s].sym as usize;
        if mlc >= ML_BASE.len() || llc >= LL_BASE.len() {
            return Err(bad("sequence code out of range"));
        }
        let ov = (1usize << ofc) + rb.read(ofc) as usize;
        let ml = ML_BASE[mlc] as usize + rb.read(u32::from(ML_BITS[mlc])) as usize;
        let ll = LL_BASE[llc] as usize + rb.read(u32::from(LL_BITS[llc])) as usize;
        let rep = &mut st.rep;
        let offset = if ov > 3 {
            let o = ov - 3;
            *rep = [o, rep[0], rep[1]];
            o
        } else {
            match ov - 1 + usize::from(ll == 0) {
                0 => rep[0],
                1 => {
                    let o = rep[1];
                    *rep = [o, rep[0], rep[2]];
                    o
                }
                2 => {
                    let o = rep[2];
                    *rep = [o, rep[0], rep[1]];
                    o
                }
                _ => {
                    let o = rep[0].wrapping_sub(1);
                    *rep = [o, rep[0], rep[1]];
                    o
                }
            }
        };
        let lit = lits
            .get(lp..lp + ll)
            .ok_or_else(|| bad("sequence takes more literals than the block has"))?;
        if out.len() + ll + ml > max_out {
            return Err(ZipError::new("zstd.limit", "output over the limit"));
        }
        out.extend_from_slice(lit);
        lp += ll;
        if offset == 0 || offset > out.len() - start {
            return Err(bad("match offset before the start of the frame"));
        }
        let from = out.len() - offset;
        if offset >= ml {
            out.extend_from_within(from..from + ml);
        } else {
            for k in 0..ml {
                let byte = out[from + k];
                out.push(byte);
            }
        }
        if i + 1 < n {
            let e = ll_t.t[ll_s];
            ll_s = e.base as usize + rb.read(u32::from(e.nb)) as usize;
            let e = ml_t.t[ml_s];
            ml_s = e.base as usize + rb.read(u32::from(e.nb)) as usize;
            let e = of_t.t[of_s];
            of_s = e.base as usize + rb.read(u32::from(e.nb)) as usize;
            if rb.pos < 0 {
                return Err(bad("sequence stream overrun"));
            }
        }
    }
    if rb.pos < 0 {
        return Err(bad("sequence stream overrun"));
    }
    if out.len() + (lits.len() - lp) > max_out {
        return Err(ZipError::new("zstd.limit", "output over the limit"));
    }
    out.extend_from_slice(&lits[lp..]);
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn raw_rle_and_empty_frames() {
        // raw block "abc", last
        let f = [0x28, 0xB5, 0x2F, 0xFD, 0x20, 3, 0x19, 0, 0, b'a', b'b', b'c'];
        assert_eq!(decompress(&f, 100).unwrap(), b"abc");
        // RLE block of 5 'x', last (header 5 << 3 | 1 << 1 | 1)
        let r = [0x28, 0xB5, 0x2F, 0xFD, 0x20, 5, 0x2B, 0, 0, b'x'];
        assert_eq!(decompress(&r, 100).unwrap(), b"xxxxx");
        assert_eq!(decompress(&[], 100).unwrap(), b"");
        // two frames and a skippable one between them
        let mut two = f.to_vec();
        two.extend_from_slice(&[0x50, 0x2A, 0x4D, 0x18, 2, 0, 0, 0, 9, 9]);
        two.extend_from_slice(&r);
        assert_eq!(decompress(&two, 100).unwrap(), b"abcxxxxx");
        // the output cap
        assert!(decompress(&two, 4).is_err());
    }

    #[test]
    fn refuses_garbage_without_panicking() {
        assert!(decompress(b"not zstd at all", 100).is_err());
        let f = [0x28, 0xB5, 0x2F, 0xFD, 0x20, 3, 0x19, 0, 0, b'a', b'b', b'c'];
        for cut in 0..f.len() {
            let _ = decompress(&f[..cut], 100);
        }
        // every byte flipped in a compressed block never panics
        for i in 0..FIXTURE.len() {
            let mut z = FIXTURE.to_vec();
            z[i] ^= 0x55;
            let _ = decompress(&z, 1 << 20);
        }
    }

    // `zstd -19` of the text below (a repeated, varied sentence), generated once.
    const FIXTURE: &[u8] = include_bytes!("../tests/fixtures/sample.txt.zst");

    fn sample_text() -> Vec<u8> {
        let mut s = String::new();
        for i in 0..400 {
            s.push_str(&format!(
                "line {i}: the quick brown fox {} over the lazy dog {}\n",
                if i % 3 == 0 { "jumps" } else { "leaps" },
                i * 7919 % 1000
            ));
        }
        s.into_bytes()
    }

    #[test]
    fn huffman_literals_in_four_streams() {
        // 9000 skewed bytes (`zstd -3`): Huffman-compressed literals
        let z = include_bytes!("../tests/fixtures/skewed.zst");
        let raw = include_bytes!("../tests/fixtures/skewed.raw");
        assert_eq!(decompress(z, 1 << 20).unwrap(), raw);
    }

    #[test]
    fn a_real_compressed_frame() {
        assert_eq!(decompress(FIXTURE, 1 << 20).unwrap(), sample_text());
    }
}
