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
    bit: u64,
    nbits: u32,
}

impl<'a> Bits<'a> {
    /// Top the bit buffer up to at least 57 bits (fewer at the end of the data).
    #[inline]
    fn refill(&mut self) {
        while self.nbits <= 56 {
            let Some(&b) = self.d.get(self.pos) else { break };
            self.pos += 1;
            self.bit |= (b as u64) << self.nbits;
            self.nbits += 8;
        }
    }
    #[inline]
    fn get(&mut self, n: u32) -> R<u32> {
        if n == 0 {
            return Ok(0);
        }
        if self.nbits < n {
            self.refill();
            if self.nbits < n {
                return Err(err("unexpected end of deflate data"));
            }
        }
        let v = (self.bit & ((1u64 << n) - 1)) as u32;
        self.bit >>= n;
        self.nbits -= n;
        Ok(v)
    }
    /// Drop the bits of the current byte and hand back the bytes read ahead: `pos` is then the next unread byte.
    fn align(&mut self) {
        self.pos -= (self.nbits / 8) as usize;
        self.bit = 0;
        self.nbits = 0;
    }
}

/// A canonical Huffman code as a lookup table indexed by the next `maxlen` stream bits (the code's first bit lowest):
/// `(symbol, code length)`, length 0 for a bit pattern that is no code.
struct Huff {
    table: Vec<(u16, u8)>,
    maxlen: u32,
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
        let maxlen = (1..16).rev().find(|&l| count[l] != 0).unwrap_or(0) as u32;
        // first canonical code of each length
        let mut next = [0u32; 17];
        let mut code = 0u32;
        for len in 1..16 {
            code = (code + count[len - 1] as u32) << 1;
            next[len] = code;
        }
        let mut table = vec![(0u16, 0u8); 1usize << maxlen];
        for (s, &l) in lengths.iter().enumerate() {
            let l = (l & 15) as u32;
            if l == 0 {
                continue;
            }
            let c = next[l as usize];
            next[l as usize] += 1;
            // (the code runs MSB first along the stream: the table index is its reversal)
            let r = c.reverse_bits() >> (32 - l);
            let mut i = r as usize;
            while i < table.len() {
                table[i] = (s as u16, l as u8);
                i += 1 << l;
            }
        }
        Ok(Huff { table, maxlen })
    }
    #[inline]
    fn decode(&self, b: &mut Bits) -> R<u16> {
        if b.nbits < self.maxlen {
            b.refill();
        }
        let (sym, len) = self.table.get((b.bit & ((1u64 << self.maxlen) - 1)) as usize).copied().unwrap_or((0, 0));
        if len == 0 || len as u32 > b.nbits {
            // (no code of this pattern: the data ended inside it, or it is invalid)
            return Err(if b.nbits < self.maxlen { err("unexpected end of deflate data") } else { err("invalid huffman code") });
        }
        b.bit >>= len;
        b.nbits -= len as u32;
        Ok(sym)
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
                b.align();
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
    b.align();
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

    /// A dynamic-Huffman block (a deflate stream of 3079 bytes of text) inflates to the same bytes, uses every input byte,
    /// and a truncated copy is an error.
    #[test]
    fn dynamic_block() {
        const DYN: [u8; 959] = [109, 86, 91, 114, 220, 48, 12, 187, 138, 143, 96, 189, 40, 233, 56, 155, 233, 78, 147, 73, 210, 238, 76, 243, 213, 211, 87, 38, 0, 218, 155, 233, 79, 156, 149, 101, 145, 4, 1, 80, 239, 183, 199, 227, 86, 242, 246, 247, 254, 117, 27, 99, 123, 251, 253, 117, 43, 219, 215, 235, 250, 53, 231, 246, 227, 254, 177, 86, 203, 118, 251, 120, 188, 222, 242, 190, 189, 172, 229, 218, 241, 186, 164, 109, 253, 181, 233, 139, 189, 96, 111, 194, 163, 229, 237, 254, 248, 243, 246, 241, 251, 87, 46, 199, 46, 126, 154, 250, 246, 126, 132, 235, 19, 71, 36, 219, 126, 222, 62, 63, 111, 59, 3, 24, 190, 206, 9, 203, 165, 123, 86, 185, 121, 86, 67, 111, 25, 170, 206, 136, 225, 219, 90, 193, 87, 105, 104, 125, 248, 122, 25, 12, 218, 16, 165, 27, 106, 213, 174, 218, 244, 159, 37, 196, 99, 253, 182, 227, 192, 254, 45, 78, 226, 57, 251, 81, 89, 181, 227, 111, 175, 88, 107, 29, 207, 169, 79, 9, 97, 106, 196, 101, 250, 17, 214, 240, 136, 147, 167, 0, 41, 200, 117, 98, 67, 225, 113, 173, 57, 124, 217, 244, 181, 111, 170, 68, 175, 150, 40, 120, 58, 82, 41, 197, 2, 16, 44, 147, 208, 48, 173, 153, 47, 37, 107, 197, 35, 173, 29, 126, 182, 13, 47, 14, 89, 207, 202, 128, 85, 159, 53, 127, 189, 144, 246, 143, 6, 17, 177, 2, 252, 152, 215, 100, 49, 201, 33, 74, 108, 79, 67, 105, 105, 178, 180, 72, 126, 157, 230, 24, 116, 38, 107, 198, 54, 0, 118, 139, 162, 10, 9, 216, 152, 123, 170, 78, 196, 129, 95, 99, 199, 219, 42, 58, 36, 210, 181, 58, 54, 171, 107, 199, 99, 97, 247, 2, 138, 147, 247, 158, 233, 96, 27, 103, 52, 102, 237, 59, 182, 175, 126, 10, 176, 128, 110, 158, 12, 98, 40, 177, 192, 16, 99, 6, 65, 107, 228, 222, 98, 205, 170, 135, 62, 49, 29, 168, 115, 122, 53, 107, 25, 213, 16, 148, 202, 214, 19, 218, 197, 28, 127, 230, 246, 13, 155, 181, 223, 235, 202, 174, 186, 68, 54, 77, 54, 213, 27, 17, 122, 136, 154, 212, 252, 117, 26, 234, 104, 207, 157, 200, 39, 180, 170, 79, 58, 5, 105, 58, 140, 129, 236, 50, 154, 64, 223, 163, 197, 168, 193, 43, 27, 36, 19, 115, 146, 230, 39, 180, 183, 75, 98, 194, 43, 251, 209, 52, 166, 218, 67, 122, 254, 113, 65, 181, 139, 16, 199, 91, 170, 170, 215, 139, 178, 65, 57, 247, 160, 158, 117, 6, 164, 198, 162, 186, 241, 165, 175, 6, 4, 44, 137, 29, 173, 8, 84, 9, 104, 71, 58, 9, 6, 51, 152, 77, 117, 81, 52, 74, 221, 202, 69, 99, 119, 183, 43, 197, 67, 238, 231, 251, 248, 175, 209, 105, 235, 127, 232, 38, 121, 20, 0, 66, 20, 27, 225, 147, 83, 172, 86, 28, 181, 81, 57, 180, 115, 101, 196, 126, 148, 83, 17, 223, 220, 64, 202, 234, 124, 154, 133, 29, 185, 255, 208, 150, 131, 248, 6, 0, 122, 44, 44, 201, 95, 6, 72, 5, 197, 71, 15, 31, 240, 248, 81, 92, 167, 84, 7, 192, 30, 104, 99, 150, 151, 58, 93, 4, 87, 8, 129, 78, 217, 246, 40, 28, 195, 10, 103, 20, 89, 210, 216, 158, 168, 48, 216, 95, 57, 97, 38, 6, 36, 22, 157, 75, 30, 210, 136, 174, 209, 164, 6, 99, 85, 228, 201, 38, 185, 198, 60, 206, 96, 82, 48, 21, 153, 125, 167, 22, 207, 105, 6, 226, 137, 8, 10, 70, 88, 41, 203, 44, 142, 39, 53, 250, 56, 3, 13, 40, 26, 97, 114, 214, 182, 93, 100, 234, 52, 75, 233, 9, 115, 99, 221, 149, 218, 96, 168, 112, 161, 68, 25, 14, 88, 22, 149, 36, 125, 229, 203, 40, 245, 215, 167, 101, 88, 112, 128, 150, 84, 40, 93, 7, 36, 179, 166, 73, 135, 24, 192, 87, 211, 101, 176, 142, 132, 26, 77, 100, 30, 49, 217, 1, 39, 106, 73, 34, 149, 155, 170, 163, 110, 209, 39, 152, 7, 224, 73, 228, 83, 62, 203, 148, 48, 125, 10, 70, 246, 171, 117, 224, 53, 114, 62, 110, 18, 126, 57, 113, 228, 66, 13, 67, 3, 222, 109, 131, 202, 47, 49, 18, 145, 1, 47, 54, 116, 208, 26, 182, 140, 113, 255, 84, 135, 31, 103, 39, 251, 61, 106, 37, 17, 27, 238, 81, 89, 190, 207, 145, 50, 149, 160, 168, 227, 72, 100, 177, 1, 123, 162, 23, 186, 34, 85, 54, 29, 3, 5, 163, 163, 238, 161, 12, 60, 237, 249, 150, 147, 226, 202, 1, 97, 200, 151, 153, 72, 13, 155, 22, 3, 113, 55, 132, 36, 80, 11, 225, 28, 253, 114, 249, 211, 244, 10, 30, 161, 67, 154, 32, 89, 64, 129, 0, 113, 21, 99, 39, 123, 220, 68, 239, 231, 184, 193, 32, 151, 55, 213, 42, 149, 189, 160, 199, 64, 74, 22, 200, 52, 115, 220, 80, 252, 84, 248, 160, 83, 169, 69, 182, 35, 206, 87, 110, 126, 237, 116, 89, 101, 4, 205, 83, 147, 238, 5, 65, 124, 113, 215, 68, 131, 236, 117, 179, 33, 113, 72, 2, 121, 202, 116, 129, 72, 53, 26, 189, 151, 30, 10, 34, 206, 24, 18, 162, 204, 40, 76, 195, 130, 234, 208, 149, 0, 110, 163, 217, 60, 144, 169, 162, 14, 50, 82, 14, 98, 236, 159, 198, 135, 253, 3];
        let (out, used) = inflate_raw(&DYN, 1 << 20).unwrap();
        assert_eq!((out.len(), used), (3079, DYN.len()));
        assert_eq!(out.iter().map(|&b| b as u64).sum::<u64>() % 65521, 59639);
        assert!(out.starts_with(b"") && out.iter().all(|b| b.is_ascii()));
        assert!(inflate_raw(&DYN[..DYN.len() - 3], 1 << 20).is_err());
        // trailing bytes after the stream are not consumed
        let mut more = DYN.to_vec();
        more.extend([1, 2, 3]);
        assert_eq!(inflate_raw(&more, 1 << 20).unwrap().1, DYN.len());
    }
}
