//! Reply encoding: `[u32 LE json_len][json][pad to 8][buffer 0][buffer 1]...`
//!
//! Pinned by the oracle (bracket.ckpart regen, debug on):
//! - total 149200 = align8(4 + 45002) + 104192 buffer bytes: buffers start at
//!   the 8-aligned base, are packed back to back, and nothing follows them;
//! - the JSON padding bytes are ASCII spaces (`20 20`), so the padded JSON
//!   is still valid JSON text;
//! - replies without buffers are still padded to a multiple of 8
//!   (`{"ok":true,"result":{"pong":true},"ms":0}` -> 48 bytes);
//! - buffer refs serialise as `{"$buf":i,"type":"f32","len":n,"offset":o}`
//!   in that key order; `i` is creation order (the silhouette is `$buf 4`
//!   but appears before the mesh in the JSON);
//! - debug replies carry `"buffers":{"count":n,"bytes":b}`.

use ck_json::{Json, Obj};

/// Element type of a binary buffer (`sd` table in app.pretty.js).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BufType {
    F32,
    F64,
    U32,
    I32,
    U8,
}

impl BufType {
    pub fn name(self) -> &'static str {
        match self {
            BufType::F32 => "f32",
            BufType::F64 => "f64",
            BufType::U32 => "u32",
            BufType::I32 => "i32",
            BufType::U8 => "u8",
        }
    }
    pub fn size(self) -> usize {
        match self {
            BufType::F64 => 8,
            BufType::U8 => 1,
            _ => 4,
        }
    }
}

/// Accumulates the binary area of one reply.
#[derive(Debug, Default)]
pub struct Buffers {
    data: Vec<u8>,
    count: usize,
}

impl Buffers {
    pub fn new() -> Buffers {
        Buffers::default()
    }

    fn push_raw(&mut self, ty: BufType, len: usize, bytes: impl Iterator<Item = u8>) -> Json {
        // GUESS: every observed reply only mixes 4-byte types, which pack with
        // no gaps. JS typed-array views need offset % element_size == 0, so
        // a u8 buffer followed by f32/f64 must be padded; zero-pad to the
        // element size.
        while self.data.len() % ty.size() != 0 {
            self.data.push(0);
        }
        let offset = self.data.len();
        self.data.extend(bytes);
        let i = self.count;
        self.count += 1;
        Obj::new().put("$buf", i).put("type", ty.name()).put("len", len).put("offset", offset).build()
    }

    pub fn f32(&mut self, v: &[f32]) -> Json {
        self.push_raw(BufType::F32, v.len(), v.iter().flat_map(|x| x.to_le_bytes()))
    }
    pub fn f64(&mut self, v: &[f64]) -> Json {
        self.push_raw(BufType::F64, v.len(), v.iter().flat_map(|x| x.to_le_bytes()))
    }
    pub fn u32(&mut self, v: &[u32]) -> Json {
        self.push_raw(BufType::U32, v.len(), v.iter().flat_map(|x| x.to_le_bytes()))
    }
    pub fn i32(&mut self, v: &[i32]) -> Json {
        self.push_raw(BufType::I32, v.len(), v.iter().flat_map(|x| x.to_le_bytes()))
    }
    pub fn u8(&mut self, v: &[u8]) -> Json {
        self.push_raw(BufType::U8, v.len(), v.iter().copied())
    }

    pub fn count(&self) -> usize {
        self.count
    }
    /// Size of the binary area in bytes (the debug `buffers.bytes`).
    pub fn bytes(&self) -> usize {
        self.data.len()
    }
    pub fn data(&self) -> &[u8] {
        &self.data
    }
}

/// Frame `json` + `bufs` into the reply byte layout.
pub fn encode(json: &str, bufs: &[u8]) -> Vec<u8> {
    let jl = json.len();
    let base = (4 + jl + 7) & !7;
    let mut out = Vec::with_capacity(base + bufs.len());
    out.extend_from_slice(&(jl as u32).to_le_bytes());
    out.extend_from_slice(json.as_bytes());
    out.resize(base, b' ');
    out.extend_from_slice(bufs);
    out
}

/// Decode a reply frame back into (json text, binary area) — for tests/tools.
pub fn decode(reply: &[u8]) -> Option<(&str, &[u8])> {
    let jl = u32::from_le_bytes(reply.get(0..4)?.try_into().ok()?) as usize;
    let json = std::str::from_utf8(reply.get(4..4 + jl)?).ok()?;
    let base = (4 + jl + 7) & !7;
    Some((json, reply.get(base..).unwrap_or(&[])))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ping_frame_is_48_bytes() {
        let j = r#"{"ok":true,"result":{"pong":true},"ms":0}"#;
        let r = encode(j, &[]);
        assert_eq!(r.len(), 48);
        assert_eq!(&r[4 + j.len()..], b"   ");
        assert_eq!(decode(&r), Some((j, &[][..])));
    }

    #[test]
    fn buffer_refs_pack_like_bracket_golden() {
        // bracket regen: positions/normals/mids f32×8280, triFace i32×920,
        // silhouette f32×288 at offsets 0/33120/66240/99360/103040.
        let mut b = Buffers::new();
        let z = vec![0f32; 8280];
        let r0 = b.f32(&z);
        let _ = b.f32(&z);
        let _ = b.f32(&z);
        let r3 = b.i32(&[0; 920]);
        let r4 = b.f32(&[0.0; 288]);
        assert_eq!(ck_json::to_string(&r0), r#"{"$buf":0,"type":"f32","len":8280,"offset":0}"#);
        assert_eq!(ck_json::to_string(&r3), r#"{"$buf":3,"type":"i32","len":920,"offset":99360}"#);
        assert_eq!(ck_json::to_string(&r4), r#"{"$buf":4,"type":"f32","len":288,"offset":103040}"#);
        assert_eq!(b.bytes(), 104192);
        assert_eq!(b.count(), 5);
        assert_eq!(encode(&"x".repeat(45002), b.data()).len(), 149200);
    }

    #[test]
    fn mixed_alignment() {
        let mut b = Buffers::new();
        let _ = b.u8(&[1, 2, 3]);
        let r = b.f64(&[1.0]);
        assert_eq!(r.get("offset").unwrap().as_f64(), Some(8.0));
    }
}
