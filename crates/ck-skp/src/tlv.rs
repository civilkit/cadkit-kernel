//! TLV tree of a VFF `model.dat`: 2-byte tag, 4-byte little-endian length, payload; a fixed set of container tags is
//! descended into. Arena based, bounded in depth and node count, never indexes out of range.
//!
//! Part of ck-skp, a Rust reimplementation of the SketchUp reader of OpenSKP (MIT, (c) Ahsan Mehmood), see NOTICE.
//! Follows `parse_tlv_recursive` / `_flat_headers` of OpenSKP (`_core.py`, `tlv.cpp`), including its leniency: a
//! record that does not fit its parent ends the parse of that range silently (the caller counts it via `truncated`).

use crate::error::{SkpError, R};

/// Tags (as the two bytes in file order, first byte high) that are descended into. From OpenSKP's `CONTAINER_TAGS`.
pub const CONTAINER_TAGS: &[u16] = &[
    0xF401, 0xF701, 0xD430, 0xD530, 0xC832, 0x7C15, 0x8813, 0x8913, 0x8A13, 0x8B13, 0x8C13, 0x8D13, 0x4C1D, 0x6419, 0xF901, 0x7017, 0x7117, 0xD007, 0xC409, 0x9411, 0x9511,
    0x0F01, 0x384A, 0xB80B, 0x9713, 0x2C4C, 0xAC0D, 0xAE0D, 0xF601, 0xF801, 0x983A, 0x993A, 0x8C3C, 0x8D3C, 0x9013, 0x401F, 0x9113, 0x9213, 0x6942, 0x6C42, 0x6842,
];

pub fn is_container(tag: u16) -> bool {
    CONTAINER_TAGS.contains(&tag)
}

#[derive(Debug, Clone, Copy)]
pub struct Node {
    /// First byte in the high half, second in the low half (OpenSKP's `C409` hex spelling reads as `0xC409`).
    pub tag: u16,
    /// Offset of the record header in the data.
    pub off: usize,
    /// Payload range.
    pub start: usize,
    pub end: usize,
    pub first: u32,
    pub nkids: u32,
}

pub type Id = u32;

pub struct Tlv<'a> {
    pub data: &'a [u8],
    pub nodes: Vec<Node>,
    pub max_nodes: usize,
    pub max_depth: usize,
    /// Count of ranges that ended on a record that did not fit.
    pub truncated: usize,
}

#[inline]
pub fn rd_u32(d: &[u8], o: usize) -> Option<u32> {
    Some(u32::from_le_bytes(d.get(o..o.checked_add(4)?)?.try_into().ok()?))
}
#[inline]
pub fn rd_f64(d: &[u8], o: usize) -> Option<f64> {
    Some(f64::from_le_bytes(d.get(o..o.checked_add(8)?)?.try_into().ok()?))
}

/// Little-endian unsigned integer of `len` bytes at `off`; `None` when it does not fit. More than 8 bytes keeps the low 8.
pub fn var_int(d: &[u8], off: usize, len: usize) -> Option<u64> {
    let end = off.checked_add(len)?;
    let s = d.get(off..end)?;
    let mut v = 0u64;
    for (i, &b) in s.iter().take(8).enumerate() {
        v |= (b as u64) << (8 * i);
    }
    Some(v)
}

/// Flat (non recursive) record headers `(tag, off, size)` of `[start, end)`.
pub fn flat_headers(d: &[u8], start: usize, end: usize) -> Vec<(u16, usize, usize)> {
    let mut out = vec![];
    let end = end.min(d.len());
    let mut pos = start;
    while pos + 6 <= end {
        let tag = u16::from_be_bytes([d[pos], d[pos + 1]]);
        let size = rd_u32(d, pos + 2).unwrap_or(0) as usize;
        if pos + 6 + size > end {
            break;
        }
        out.push((tag, pos, size));
        pos += 6 + size;
    }
    out
}

/// Flat TLV walk over a leaf payload (`_tlv_flat`): `(tag, body)` pairs.
pub fn flat(payload: &[u8]) -> Vec<(u16, &[u8])> {
    let mut out = vec![];
    let mut pos = 0;
    while pos + 6 <= payload.len() {
        let tag = u16::from_be_bytes([payload[pos], payload[pos + 1]]);
        let size = rd_u32(payload, pos + 2).unwrap_or(0) as usize;
        if pos + 6 + size > payload.len() {
            break;
        }
        out.push((tag, &payload[pos + 6..pos + 6 + size]));
        pos += 6 + size;
    }
    out
}

impl<'a> Tlv<'a> {
    pub fn new(data: &'a [u8], max_nodes: usize, max_depth: usize) -> Tlv<'a> {
        Tlv { data, nodes: Vec::new(), max_nodes, max_depth, truncated: 0 }
    }

    pub fn clear(&mut self) {
        self.nodes.clear();
    }

    /// Parse `[start, end)` as a run of records; returns the id range of the direct children.
    pub fn parse_range(&mut self, start: usize, end: usize, depth: usize) -> R<(u32, u32)> {
        if depth > self.max_depth {
            return Err(SkpError::at("limit.depth", start, format!("TLV nesting deeper than {}", self.max_depth)));
        }
        let end = end.min(self.data.len());
        let first = self.nodes.len() as u32;
        let mut pos = start;
        while pos + 6 <= end {
            let tag = u16::from_be_bytes([self.data[pos], self.data[pos + 1]]);
            let size = rd_u32(self.data, pos + 2).unwrap_or(0) as usize;
            if pos + 6 + size > end {
                // a container tag whose payload is a plain string (layer names, ...) fails on its first record: not a defect
                if self.nodes.len() as u32 > first {
                    self.truncated += 1;
                }
                break;
            }
            if self.nodes.len() >= self.max_nodes {
                return Err(SkpError::at("limit.nodes", pos, format!("more than {} TLV nodes", self.max_nodes)));
            }
            self.nodes.push(Node { tag, off: pos, start: pos + 6, end: pos + 6 + size, first: 0, nkids: 0 });
            pos += 6 + size;
        }
        let count = self.nodes.len() as u32 - first;
        for i in first..first + count {
            let n = self.nodes[i as usize];
            if is_container(n.tag) && n.end > n.start {
                let (f, c) = self.parse_range(n.start, n.end, depth + 1)?;
                let m = &mut self.nodes[i as usize];
                m.first = f;
                m.nkids = c;
            }
        }
        Ok((first, count))
    }

    pub fn node(&self, id: Id) -> &Node {
        &self.nodes[id as usize]
    }

    pub fn kids(&self, id: Id) -> std::ops::Range<Id> {
        let n = &self.nodes[id as usize];
        n.first..n.first + n.nkids
    }

    /// Payload bytes; empty for a container that has children (OpenSKP semantics).
    pub fn payload(&self, id: Id) -> &'a [u8] {
        let n = &self.nodes[id as usize];
        if n.nkids > 0 {
            &[]
        } else {
            &self.data[n.start..n.end]
        }
    }

    /// Direct child with `tag`.
    pub fn child(&self, id: Id, tag: u16) -> Option<Id> {
        self.kids(id).find(|&k| self.nodes[k as usize].tag == tag)
    }

    /// First node with `tag` among `ids` and their descendants, pre-order (`find_child_tag`).
    pub fn find_in(&self, ids: std::ops::Range<Id>, tag: u16) -> Option<Id> {
        for k in ids {
            if self.nodes[k as usize].tag == tag {
                return Some(k);
            }
            if let Some(r) = self.find_in(self.kids(k), tag) {
                return Some(r);
            }
        }
        None
    }

    /// First descendant of `id` with `tag`.
    pub fn find(&self, id: Id, tag: u16) -> Option<Id> {
        self.find_in(self.kids(id), tag)
    }

    /// Every node with `tag` among `ids` and descendants, pre-order, matched nodes still descended (`find_all_nodes_rec`).
    pub fn find_all(&self, ids: std::ops::Range<Id>, tag: u16, out: &mut Vec<Id>) {
        for k in ids {
            if self.nodes[k as usize].tag == tag {
                out.push(k);
            }
            self.find_all(self.kids(k), tag, out);
        }
    }

    /// Entity id (`extract_entity_id`): a direct `DE05`, or a `DC05` wrapping `DE05`, else the first found in a child.
    pub fn entity_id(&self, id: Id) -> Option<u64> {
        for k in self.kids(id) {
            let n = &self.nodes[k as usize];
            if n.tag == 0xDE05 {
                let p = self.payload(k);
                return var_int(p, 0, p.len());
            }
            if n.tag == 0xDC05 {
                let p = self.payload(k);
                if p.starts_with(&[0xDE, 0x05]) {
                    if let Some(len) = rd_u32(p, 2) {
                        if let Some(v) = var_int(p, 6, len as usize) {
                            return Some(v);
                        }
                    }
                }
            }
        }
        for k in self.kids(id) {
            if let Some(v) = self.entity_id(k) {
                return Some(v);
            }
        }
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn rec(tag: u16, payload: &[u8]) -> Vec<u8> {
        let mut v = tag.to_be_bytes().to_vec();
        v.extend((payload.len() as u32).to_le_bytes());
        v.extend(payload);
        v
    }

    #[test]
    fn nested_and_boundary() {
        // a record whose header exactly fills the range is kept
        let d = rec(0x0300, &[]);
        let mut t = Tlv::new(&d, 100, 8);
        let (_, c) = t.parse_range(0, d.len(), 0).unwrap();
        assert_eq!(c, 1);
        let inner = [rec(0xDE05, &[7]), rec(0x1234, &[1, 2])].concat();
        let d = rec(0xC409, &inner);
        let mut t = Tlv::new(&d, 100, 8);
        let (f, c) = t.parse_range(0, d.len(), 0).unwrap();
        assert_eq!((f, c), (0, 1));
        assert_eq!(t.entity_id(0), Some(7));
        assert_eq!(t.kids(0).len(), 2);
        assert!(t.payload(0).is_empty());
    }

    #[test]
    fn limits() {
        let mut d = rec(0x1234, &[]);
        for _ in 0..30 {
            d = rec(0xC409, &d);
        }
        let mut t = Tlv::new(&d, 1000, 8);
        assert_eq!(t.parse_range(0, d.len(), 0).unwrap_err().code, "limit.depth");
        let mut t = Tlv::new(&d, 5, 100);
        assert_eq!(t.parse_range(0, d.len(), 0).unwrap_err().code, "limit.nodes");
        // oversized length: stops quietly
        let bad = [0x12u8, 0x34, 0xFF, 0xFF, 0xFF, 0xFF, 1, 2];
        let mut t = Tlv::new(&bad, 10, 10);
        assert_eq!(t.parse_range(0, bad.len(), 0).unwrap().1, 0);
        assert_eq!(t.truncated, 0);
        let two = [rec(0x0100, &[1]), bad.to_vec()].concat();
        let mut t = Tlv::new(&two, 10, 10);
        assert_eq!(t.parse_range(0, two.len(), 0).unwrap().1, 1);
        assert_eq!(t.truncated, 1);
    }
}
