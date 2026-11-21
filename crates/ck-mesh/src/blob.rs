//! The mesh a part holds in its `blobs` table (the `meshToSolid` feature): bytes in one of two layouts, base64 text on
//! the outside, each layout optionally deflated (raw DEFLATE, RFC 1951).
//!
//! 1. **CKM2**, the app's packed mesh (`packages/io/src/meshPack.ts`, little endian): `'CKM2'` | flags u8 (bit 0:
//!    lossless floats) | 3 pad | nVerts u32 | nTris u32 | step f64 | vLen u32 | iLen u32 | mLen u32 | vertex section |
//!    index section | material section. Vertices are zigzag varint differences of grid coordinates (`q * step`, rounded to
//!    f32 as the app's `Float32Array` holds them), or with the lossless flag the raw f32 bytes with the four byte planes
//!    apart. Triangle corners are zigzag varints of the distance below the highest vertex index used so far. The
//!    material section (per triangle material runs) is not read.
//! 2. **raw**: `nVerts u32 | nTris u32 | f32 x,y,z (nVerts times) | u32 a,b,c (nTris times)`, little endian, no magic;
//!    the byte count must be exactly `8 + 12 nVerts + 12 nTris`.
//!
//! Detection: bytes that start with `CKM2` are layout 1; else a byte count that fits layout 2 exactly is layout 2; else
//! the bytes are inflated (at most [`MAX_INFLATED`] bytes) and the same test is made once more. Anything else is
//! `meshBadInput`. [`encode_raw`] writes layout 2.

use crate::Refusal;

/// The most bytes a blob may inflate to (a damaged or hostile stream cannot grow without bound).
pub const MAX_INFLATED: usize = 256 << 20;
const HEADER: usize = 36;

/// A decoded mesh: xyz per vertex, three corners per triangle.
#[derive(Debug, Clone, PartialEq)]
pub struct MeshData {
    pub positions: Vec<f64>,
    pub indices: Vec<u32>,
}

fn bad(what: impl Into<String>) -> Refusal {
    Refusal::new("meshBadInput", what, "the blob must be the app's packed mesh (CKM2, optionally deflated) or `u32 nVerts, u32 nTris, f32 xyz.., u32 indices..` (little endian), as base64 text")
}

fn u32_at(b: &[u8], p: usize) -> u32 {
    u32::from_le_bytes([b[p], b[p + 1], b[p + 2], b[p + 3]])
}

/// The raw layout: `u32 nVerts | u32 nTris | f32 xyz.. | u32 indices..`.
pub fn encode_raw(positions: &[f32], indices: &[u32]) -> Vec<u8> {
    let mut o = Vec::with_capacity(8 + 4 * positions.len() + 4 * indices.len());
    o.extend_from_slice(&((positions.len() / 3) as u32).to_le_bytes());
    o.extend_from_slice(&((indices.len() / 3) as u32).to_le_bytes());
    positions.iter().for_each(|v| o.extend_from_slice(&v.to_le_bytes()));
    indices.iter().for_each(|v| o.extend_from_slice(&v.to_le_bytes()));
    o
}

fn raw_fits(b: &[u8]) -> bool {
    if b.len() < 8 {
        return false;
    }
    let (nv, nt) = (u32_at(b, 0) as u64, u32_at(b, 4) as u64);
    8 + 12 * nv + 12 * nt == b.len() as u64
}

fn decode_raw(b: &[u8]) -> MeshData {
    let (nv, nt) = (u32_at(b, 0) as usize, u32_at(b, 4) as usize);
    let positions = (0..3 * nv).map(|i| f32::from_le_bytes([b[8 + 4 * i], b[9 + 4 * i], b[10 + 4 * i], b[11 + 4 * i]]) as f64).collect();
    let at = 8 + 12 * nv;
    let indices = (0..3 * nt).map(|i| u32_at(b, at + 4 * i)).collect();
    MeshData { positions, indices }
}

struct Varints<'a> {
    b: &'a [u8],
    p: usize,
}

impl Varints<'_> {
    fn get(&mut self) -> Result<u64, Refusal> {
        let mut v = 0u64;
        for k in 0..8 {
            let c = *self.b.get(self.p).ok_or_else(|| bad("the packed mesh is cut short"))?;
            self.p += 1;
            v |= ((c & 0x7f) as u64) << (7 * k);
            if c < 0x80 {
                return Ok(v);
            }
        }
        Err(bad("the packed mesh has a number out of range"))
    }
}

fn unzig(z: u64) -> i64 {
    if z % 2 == 0 {
        (z / 2) as i64
    } else {
        -((z as i64 + 1) / 2)
    }
}

fn decode_ckm2(b: &[u8]) -> Result<MeshData, Refusal> {
    if b.len() < HEADER {
        return Err(bad("the packed mesh is cut short"));
    }
    let lossless = b[4] & 1 == 1;
    let (nv, nt) = (u32_at(b, 8) as usize, u32_at(b, 12) as usize);
    let step = f64::from_le_bytes(b[16..24].try_into().unwrap());
    let (vl, il, ml) = (u32_at(b, 24) as usize, u32_at(b, 28) as usize, u32_at(b, 32) as usize);
    if HEADER.checked_add(vl).and_then(|x| x.checked_add(il)).and_then(|x| x.checked_add(ml)).map_or(true, |t| t > b.len()) {
        return Err(bad("the packed mesh is cut short"));
    }
    if nv > 0x1fff_ffff || nt > 0x1fff_ffff {
        return Err(bad("the packed mesh is too large"));
    }
    let mut positions = Vec::with_capacity(3 * nv.min(vl));
    if lossless {
        if vl != nv * 12 {
            return Err(bad("the packed mesh's vertex section has the wrong size"));
        }
        let n = nv * 3;
        for i in 0..n {
            let q = |plane: usize| b[HEADER + plane * n + i];
            positions.push(f32::from_le_bytes([q(0), q(1), q(2), q(3)]) as f64);
        }
    } else {
        if !(step > 0.0) || !step.is_finite() {
            return Err(bad("the packed mesh has no grid step"));
        }
        // every number takes at least a byte: a count the section cannot hold is damage, not an allocation
        if vl < nv.saturating_mul(3) {
            return Err(bad("the packed mesh's vertex section is cut short"));
        }
        let mut r = Varints { b: &b[..HEADER + vl], p: HEADER };
        let mut px = [0i64; 3];
        for _ in 0..nv {
            for q in px.iter_mut() {
                *q += unzig(r.get()?);
                positions.push(((*q as f64) * step) as f32 as f64);
            }
        }
    }
    if il < nt.saturating_mul(3) {
        return Err(bad("the packed mesh's index section is cut short"));
    }
    let mut indices = Vec::with_capacity(3 * nt);
    let mut r = Varints { b: &b[..HEADER + vl + il], p: HEADER + vl };
    let mut hwm = 0i64;
    for _ in 0..3 * nt {
        let id = hwm - unzig(r.get()?);
        if !(id >= 0 && id < nv as i64) {
            return Err(bad("the packed mesh has a triangle corner that is no vertex"));
        }
        indices.push(id as u32);
        if id >= hwm {
            hwm = id + 1;
        }
    }
    Ok(MeshData { positions, indices })
}

/// The mesh the bytes of a blob hold (see the module comment for the two layouts and the detection).
pub fn decode_bytes(bytes: &[u8]) -> Result<MeshData, Refusal> {
    let direct = |b: &[u8]| -> Option<Result<MeshData, Refusal>> {
        if b.starts_with(b"CKM2") {
            Some(decode_ckm2(b))
        } else if raw_fits(b) {
            Some(Ok(decode_raw(b)))
        } else {
            None
        }
    };
    if let Some(r) = direct(bytes) {
        return r;
    }
    let (inflated, _) = ck_sldprt::inflate::inflate_raw(bytes, MAX_INFLATED).map_err(|e| bad(format!("the blob is neither a mesh layout nor deflated data ({})", e.message)))?;
    direct(&inflated).unwrap_or_else(|| Err(bad("the blob inflates to something that is neither the packed mesh nor the raw layout")))
}

/// The mesh of a blob's base64 text.
pub fn decode_base64(text: &str) -> Result<MeshData, Refusal> {
    let bytes = crate::request::b64(text, "blob")?;
    decode_bytes(&bytes)
}
