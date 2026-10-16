//! Bounded DEFLATE (RFC 1951) / zlib (RFC 1950) decoder and CRC-32.
//!
//! Written for this crate (no dependencies, no unsafe). Output is capped: exceeding the cap is an error, not an
//! allocation, which is the decompression-bomb guard.

use crate::error::{SldprtError, R};

/// CRC-32 (IEEE, as used by the container blocks and zlib/gzip).
pub fn crc32(data: &[u8]) -> u32 {
    let mut table = [0u32; 256];
    for (i, t) in table.iter_mut().enumerate() {
        let mut c = i as u32;
        for _ in 0..8 {
            c = if c & 1 != 0 { 0xEDB8_8320 ^ (c >> 1) } else { c >> 1 };
        }
        *t = c;
    }
    let mut c = !0u32;
    for &b in data {
        c = table[((c ^ b as u32) & 0xFF) as usize] ^ (c >> 8);
    }
    !c
}

fn err(msg: &str) -> SldprtError {
    SldprtError::new("inflate.corrupt", msg)
}

struct Bits<'a> {
    d: &'a [u8],
    pos: usize,
    bit: u32,
    nbits: u32,
}

impl<'a> Bits<'a> {
    fn need(&mut self, n: u32) -> R<()> {
        while self.nbits < n {
            let b = *self.d.get(self.pos).ok_or_else(|| err("unexpected end of deflate data"))?;
            self.pos += 1;
            self.bit |= (b as u32) << self.nbits;
            self.nbits += 8;
        }
        Ok(())
    }
    fn get(&mut self, n: u32) -> R<u32> {
        if n == 0 {
            return Ok(0);
        }
        self.need(n)?;
        let v = self.bit & ((1u32 << n) - 1);
        self.bit >>= n;
        self.nbits -= n;
        Ok(v)
    }
}

struct Huff {
    count: [u16; 16],
    symbol: Vec<u16>,
}

impl Huff {
    fn new(lengths: &[u8]) -> R<Huff> {
        let mut count = [0u16; 16];
        for &l in lengths {
            count[(l & 15) as usize] += 1;
        }
        count[0] = 0;
        let mut left: i32 = 1;
        for c in count.iter().skip(1) {
            left <<= 1;
            left -= *c as i32;
            if left < 0 {
                return Err(err("over-subscribed huffman code"));
            }
        }
        let mut offs = [0u16; 16];
        for len in 1..15 {
            offs[len + 1] = offs[len] + count[len];
        }
        let mut symbol = vec![0u16; lengths.len()];
        for (s, &l) in lengths.iter().enumerate() {
            if l != 0 {
                symbol[offs[l as usize] as usize] = s as u16;
                offs[l as usize] += 1;
            }
        }
        Ok(Huff { count, symbol })
    }
    fn decode(&self, b: &mut Bits) -> R<u16> {
        let (mut code, mut first, mut index) = (0i32, 0i32, 0i32);
        for len in 1..16 {
            code |= b.get(1)? as i32;
            let c = self.count[len] as i32;
            if code - c < first {
                return self.symbol.get((index + (code - first)) as usize).copied().ok_or_else(|| err("bad huffman symbol"));
            }
            index += c;
            first += c;
            first <<= 1;
            code <<= 1;
        }
        Err(err("invalid huffman code"))
    }
}

const LBASE: [u16; 29] = [3, 4, 5, 6, 7, 8, 9, 10, 11, 13, 15, 17, 19, 23, 27, 31, 35, 43, 51, 59, 67, 83, 99, 115, 131, 163, 195, 227, 258];
const LEXT: [u8; 29] = [0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3, 4, 4, 4, 4, 5, 5, 5, 5, 0];
const DBASE: [u16; 30] = [1, 2, 3, 4, 5, 7, 9, 13, 17, 25, 33, 49, 65, 97, 129, 193, 257, 385, 513, 769, 1025, 1537, 2049, 3073, 4097, 6145, 8193, 12289, 16385, 24577];
const DEXT: [u8; 30] = [0, 0, 0, 0, 1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 6, 6, 7, 7, 8, 8, 9, 9, 10, 10, 11, 11, 12, 12, 13, 13];

/// Inflate raw DEFLATE `data`; output larger than `max_out` is an error (`limit.inflate`).
/// Returns the output and the number of input bytes consumed.
pub fn inflate_raw(data: &[u8], max_out: usize) -> R<(Vec<u8>, usize)> {
    let mut b = Bits { d: data, pos: 0, bit: 0, nbits: 0 };
    let mut out: Vec<u8> = Vec::new();
    let over = || SldprtError::new("limit.inflate", "inflated size exceeds the limit");
    // ratio guard: more than 1 MiB out of far less than max_ratio times as many bytes in is a bomb
    let ratio_cap = data.len().saturating_mul(1032).saturating_add(1 << 20);
    loop {
        let last = b.get(1)?;
        match b.get(2)? {
            0 => {
                b.bit = 0;
                b.nbits = 0;
                let p = b.pos;
                if p + 4 > data.len() {
                    return Err(err("truncated stored block"));
                }
                let len = u16::from_le_bytes([data[p], data[p + 1]]) as usize;
                let nlen = u16::from_le_bytes([data[p + 2], data[p + 3]]) as usize;
                if len != (!nlen & 0xFFFF) {
                    return Err(err("stored block length check failed"));
                }
                if p + 4 + len > data.len() {
                    return Err(err("truncated stored block"));
                }
                if out.len() + len > max_out {
                    return Err(over());
                }
                out.extend_from_slice(&data[p + 4..p + 4 + len]);
                b.pos = p + 4 + len;
            }
            t @ (1 | 2) => {
                let (lit, dist) = if t == 1 {
                    let mut l = [8u8; 288];
                    for x in l.iter_mut().take(256).skip(144) {
                        *x = 9;
                    }
                    for x in l.iter_mut().take(280).skip(256) {
                        *x = 7;
                    }
                    (Huff::new(&l)?, Huff::new(&[5u8; 30])?)
                } else {
                    let nlen = b.get(5)? as usize + 257;
                    let ndist = b.get(5)? as usize + 1;
                    let ncode = b.get(4)? as usize + 4;
                    if nlen > 286 || ndist > 30 {
                        return Err(err("bad dynamic block counts"));
                    }
                    const ORDER: [usize; 19] = [16, 17, 18, 0, 8, 7, 9, 6, 10, 5, 11, 4, 12, 3, 13, 2, 14, 1, 15];
                    let mut cl = [0u8; 19];
                    for &o in ORDER.iter().take(ncode) {
                        cl[o] = b.get(3)? as u8;
                    }
                    let ch = Huff::new(&cl)?;
                    let mut lens = vec![0u8; nlen + ndist];
                    let mut i = 0;
                    while i < nlen + ndist {
                        let s = ch.decode(&mut b)?;
                        if s < 16 {
                            lens[i] = s as u8;
                            i += 1;
                        } else {
                            let (prev, rep) = match s {
                                16 => {
                                    if i == 0 {
                                        return Err(err("repeat with no previous length"));
                                    }
                                    (lens[i - 1], 3 + b.get(2)? as usize)
                                }
                                17 => (0, 3 + b.get(3)? as usize),
                                _ => (0, 11 + b.get(7)? as usize),
                            };
                            if i + rep > nlen + ndist {
                                return Err(err("too many code lengths"));
                            }
                            for _ in 0..rep {
                                lens[i] = prev;
                                i += 1;
                            }
                        }
                    }
                    if lens[256] == 0 {
                        return Err(err("no end-of-block code"));
                    }
                    (Huff::new(&lens[..nlen])?, Huff::new(&lens[nlen..])?)
                };
                loop {
                    let s = lit.decode(&mut b)? as usize;
                    if s < 256 {
                        if out.len() >= max_out {
                            return Err(over());
                        }
                        out.push(s as u8);
                    } else if s == 256 {
                        break;
                    } else {
                        let s = s - 257;
                        if s >= 29 {
                            return Err(err("bad length symbol"));
                        }
                        let len = LBASE[s] as usize + b.get(LEXT[s] as u32)? as usize;
                        let ds = dist.decode(&mut b)? as usize;
                        if ds >= 30 {
                            return Err(err("bad distance symbol"));
                        }
                        let d = DBASE[ds] as usize + b.get(DEXT[ds] as u32)? as usize;
                        if d > out.len() {
                            return Err(err("distance too far back"));
                        }
                        if out.len() + len > max_out {
                            return Err(over());
                        }
                        let start = out.len() - d;
                        for k in 0..len {
                            let c = out[start + k];
                            out.push(c);
                        }
                    }
                    if out.len() > ratio_cap {
                        return Err(SldprtError::new("limit.ratio", "compression ratio exceeds the limit (decompression bomb)"));
                    }
                }
            }
            _ => return Err(err("reserved block type")),
        }
        if last == 1 {
            break;
        }
    }
    Ok((out, b.pos))
}

/// Inflate a zlib stream (2-byte header, adler trailer ignored). Returns output and bytes consumed (header + deflate data,
/// not counting the 4 byte trailer).
pub fn inflate_zlib(data: &[u8], max_out: usize) -> R<(Vec<u8>, usize)> {
    if data.len() < 2 || data[0] & 0x0F != 8 || ((data[0] as u16) << 8 | data[1] as u16) % 31 != 0 {
        return Err(SldprtError::new("inflate.header", "not a zlib stream"));
    }
    let (out, used) = inflate_raw(&data[2..], max_out)?;
    Ok((out, 2 + used))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn crc_known() {
        assert_eq!(crc32(b"123456789"), 0xCBF4_3926);
    }

    #[test]
    fn stored_and_fixed() {
        let s = [0x01, 3, 0, 0xFC, 0xFF, b'a', b'b', b'c'];
        assert_eq!(inflate_raw(&s, 100).unwrap().0, b"abc");
        assert_eq!(inflate_raw(&[0x4b, 0x04, 0x00], 100).unwrap().0, b"a");
        assert!(inflate_raw(&s, 2).is_err());
        assert!(inflate_raw(&s[..4], 100).is_err());
    }
}
