//! Legacy container (SketchUp 3 to 2020): one uncompressed MFC `CArchive` object stream with a single global store map,
//! walked class by class. This file holds the archive machinery and the per-class readers; [`crate::legacy`] drives the
//! walk and builds the [`crate::model::Model`].
//!
//! Part of ck-skp, a Rust reimplementation of the SketchUp reader of OpenSKP (MIT, (c) Ahsan Mehmood), see NOTICE.
//! The byte layouts, the version gates and the recovery heuristics (burned store-map indices, the SketchUp 2020 filler
//! record, slot-base calibration) are OpenSKP's reverse-engineered findings, ported from `legacy.py` / `legacy.cpp`.
//! Differences: every read is bounds checked and returns a value instead of raising, nesting is depth-limited, and
//! attribute dictionaries / annotations are parsed for alignment only (their values are dropped).

use std::collections::HashMap;

pub const STR_MARKER: [u8; 3] = [0xFF, 0xFE, 0xFF];
pub const FIRST_V4: u32 = 4;

/// A legacy walk failure. `eof` marks reads past the end of the data (Python's `struct.error`/`IndexError` in the
/// reference, which some recovery paths deliberately do not catch).
#[derive(Debug, Clone)]
pub struct LErr {
    pub eof: bool,
    pub msg: String,
    pub pos: usize,
}
pub type LR<T> = Result<T, LErr>;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Cls {
    Vertex,
    Edge,
    Curve,
    ArcCurve,
    EdgeUse,
    Loop,
    Face,
    Layer,
    Material,
    Dib,
    AttrContainer,
    AttrNamed,
    Camera,
    Thumbnail,
    Relationship,
    Definition,
    Image,
    Instance,
    Group,
    Ftc,
    CLine,
    CPoint,
    SectionPlane,
    Font,
    DimLinear,
    Text,
}

impl Cls {
    pub fn from_name(n: &str) -> Option<Cls> {
        Some(match n {
            "CVertex" => Cls::Vertex,
            "CEdge" => Cls::Edge,
            "CCurve" => Cls::Curve,
            "CArcCurve" => Cls::ArcCurve,
            "CEdgeUse" => Cls::EdgeUse,
            "CLoop" => Cls::Loop,
            "CFace" => Cls::Face,
            "CLayer" => Cls::Layer,
            "CMaterial" => Cls::Material,
            "CDib" => Cls::Dib,
            "CAttributeContainer" => Cls::AttrContainer,
            "CAttributeNamed" => Cls::AttrNamed,
            "CCamera" => Cls::Camera,
            "CThumbnail" => Cls::Thumbnail,
            "CRelationship" => Cls::Relationship,
            "CComponentDefinition" => Cls::Definition,
            "CImage" => Cls::Image,
            "CComponentInstance" => Cls::Instance,
            "CGroup" => Cls::Group,
            "CFaceTextureCoords" => Cls::Ftc,
            "CConstructionLine" => Cls::CLine,
            "CConstructionPoint" => Cls::CPoint,
            "CSectionPlane" => Cls::SectionPlane,
            "CSkFont" => Cls::Font,
            "CDimensionLinear" => Cls::DimLinear,
            "CText" => Cls::Text,
            _ => return None,
        })
    }
    pub fn name(self) -> &'static str {
        match self {
            Cls::Vertex => "CVertex",
            Cls::Edge => "CEdge",
            Cls::Curve => "CCurve",
            Cls::ArcCurve => "CArcCurve",
            Cls::EdgeUse => "CEdgeUse",
            Cls::Loop => "CLoop",
            Cls::Face => "CFace",
            Cls::Layer => "CLayer",
            Cls::Material => "CMaterial",
            Cls::Dib => "CDib",
            Cls::AttrContainer => "CAttributeContainer",
            Cls::AttrNamed => "CAttributeNamed",
            Cls::Camera => "CCamera",
            Cls::Thumbnail => "CThumbnail",
            Cls::Relationship => "CRelationship",
            Cls::Definition => "CComponentDefinition",
            Cls::Image => "CImage",
            Cls::Instance => "CComponentInstance",
            Cls::Group => "CGroup",
            Cls::Ftc => "CFaceTextureCoords",
            Cls::CLine => "CConstructionLine",
            Cls::CPoint => "CConstructionPoint",
            Cls::SectionPlane => "CSectionPlane",
            Cls::Font => "CSkFont",
            Cls::DimLinear => "CDimensionLinear",
            Cls::Text => "CText",
        }
    }
}

/// What a store-map slot holds, as the name of the result of `read_object`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Name {
    /// Null object.
    None,
    /// A burned (phantom) store-map index.
    Reserved,
    /// A reference into the unwalked pre-model region.
    Premodel,
    Cls(Cls),
}

#[derive(Debug, Clone)]
enum Entry {
    Class { name: String, cls: Option<Cls> },
    Obj { cls: Cls, val: Option<usize> },
}

pub type ValIdx = usize;

#[derive(Debug, Clone, Copy, Default)]
pub struct Db {
    pub mat: u16,
    pub hidden: u8,
    pub soft: u8,
    pub smooth: u8,
    pub layer: u16,
}

#[derive(Debug, Clone)]
pub struct TexBlock {
    pub rgba: [u8; 4],
    pub opacity: f64,
    pub use_opacity: u8,
    pub tex_dib: Option<u64>,
    pub tex_data: Option<(usize, usize)>,
    pub tex_w: f64,
    pub tex_h: f64,
    pub tex_file: String,
    pub colorized: bool,
}

#[derive(Debug, Clone)]
pub enum Val {
    Vertex([f64; 3]),
    Edge { db: Db, curve: Option<u64>, curve_val: Option<ValIdx>, v1: Option<u64>, v2: Option<u64> },
    Curve,
    ArcCurve,
    EdgeUse { edge: Option<u64>, sense: u8 },
    Loop { uses: Vec<Option<(Option<u64>, u8)>> },
    Face { db: Db, plane: [f64; 4], loops: Vec<Option<Vec<Option<(Option<u64>, u8)>>>>, back_mat: u16, attrs: Option<ValIdx> },
    Layer { name: String, hidden: u8, rgba: [u8; 4] },
    Material { name: String, rgba: [u8; 4], opacity: f64, use_opacity: u8, tex: Option<TexBlock> },
    Dib { start: usize, len: usize },
    Attrs { children: Vec<Option<ValIdx>> },
    AttrNamed,
    Ftc { front: [f64; 9], back: [f64; 9] },
    Camera,
    Thumbnail,
    Relationship,
    Definition { name: String, guid: String, ents: Vec<(Option<u64>, Option<ValIdx>)>, faces_camera: bool, shadows_face_sun: bool },
    Instance { db: Db, def: Option<u64>, xf: [f64; 13], name: String, guid: String },
    Image { db: Db, def: Option<u64>, xform: [f64; 12], guid: String },
    Other,
}

#[derive(Debug, Clone, Copy)]
pub struct ObjRef {
    pub slot: Option<u64>,
    pub name: Name,
    pub val: Option<ValIdx>,
}

const NONE_REF: ObjRef = ObjRef { slot: None, name: Name::None, val: None };

pub struct Ar<'a> {
    pub data: &'a [u8],
    pub pos: usize,
    pub ver: u32,
    pub has_pid: bool,
    /// Walked slots `[base, base + objs.len())`.
    base: u64,
    objs: Vec<Entry>,
    /// Classes (and placeholders) learned outside the walked range.
    learned: HashMap<u64, Entry>,
    pub class_slot: HashMap<String, u64>,
    pub walk_base: u64,
    current_obj_slot: Option<u64>,
    current_loop: Option<u64>,
    burns: Vec<(u64, u64)>,
    cum_delta: u64,
    annot_watermark: Option<u64>,
    burn_stack: Vec<u64>,
    cline_tail: Option<usize>,
    pub vals: Vec<Val>,
    depth: usize,
    max_depth: usize,
    max_objects: usize,
}

fn cap_slice(d: &[u8], a: usize, b: usize) -> &[u8] {
    let a = a.min(d.len());
    let b = b.min(d.len());
    if a >= b {
        &[]
    } else {
        &d[a..b]
    }
}

/// `bytes.find(sub, start, end)`: first index `i >= start` with `i + sub.len() <= end`.
pub fn find_in(d: &[u8], sub: &[u8], start: usize, end: usize) -> Option<usize> {
    let end = end.min(d.len());
    if sub.is_empty() || start + sub.len() > end {
        return None;
    }
    let first = sub[0];
    let mut i = start;
    let last = end - sub.len();
    while i <= last {
        match d[i..=last].iter().position(|&b| b == first) {
            None => return None,
            Some(p) => i += p,
        }
        if &d[i..i + sub.len()] == sub {
            return Some(i);
        }
        i += 1;
    }
    None
}

impl<'a> Ar<'a> {
    pub fn new(data: &'a [u8], ver: u32, max_depth: usize, max_objects: usize) -> Ar<'a> {
        Ar {
            data,
            pos: 0,
            ver,
            has_pid: ver >= 17,
            base: 0,
            objs: vec![],
            learned: HashMap::new(),
            class_slot: HashMap::new(),
            walk_base: 0,
            current_obj_slot: None,
            current_loop: None,
            burns: vec![],
            cum_delta: 0,
            annot_watermark: None,
            burn_stack: vec![],
            cline_tail: None,
            vals: vec![],
            depth: 0,
            max_depth,
            max_objects,
        }
    }

    /// Set the slot of the next allocation (call once, before anything is allocated).
    pub fn set_base(&mut self, base: u64) {
        self.base = base;
        self.walk_base = base;
    }

    pub fn next_slot(&self) -> u64 {
        self.base + self.objs.len() as u64
    }

    // ---- primitives
    fn err(&self, msg: impl Into<String>) -> LErr {
        LErr { eof: false, msg: msg.into(), pos: self.pos }
    }
    fn eof(&self) -> LErr {
        LErr { eof: true, msg: "read past the end of the data".into(), pos: self.pos }
    }
    pub fn ctx_err(&self, msg: &str) -> LErr {
        self.err(msg)
    }
    pub fn u8(&mut self) -> LR<u8> {
        let v = *self.data.get(self.pos).ok_or_else(|| self.eof())?;
        self.pos += 1;
        Ok(v)
    }
    pub fn u16(&mut self) -> LR<u16> {
        let v = self.peek_u16()?;
        self.pos += 2;
        Ok(v)
    }
    pub fn u32(&mut self) -> LR<u32> {
        let s = self.data.get(self.pos..self.pos + 4).ok_or_else(|| self.eof())?;
        let v = u32::from_le_bytes([s[0], s[1], s[2], s[3]]);
        self.pos += 4;
        Ok(v)
    }
    pub fn f64(&mut self) -> LR<f64> {
        let s = self.data.get(self.pos..self.pos + 8).ok_or_else(|| self.eof())?;
        let mut b = [0u8; 8];
        b.copy_from_slice(s);
        self.pos += 8;
        Ok(f64::from_le_bytes(b))
    }
    pub fn f64n<const N: usize>(&mut self) -> LR<[f64; N]> {
        let mut o = [0.0; N];
        let need = 8 * N;
        if self.pos.checked_add(need).map_or(true, |e| e > self.data.len()) {
            return Err(self.eof());
        }
        for v in o.iter_mut() {
            *v = self.f64()?;
        }
        Ok(o)
    }
    /// Like Python slicing: returns what is available, always advances by `n`.
    pub fn raw(&mut self, n: usize) -> &'a [u8] {
        let end = self.pos.saturating_add(n).min(self.data.len());
        let s = cap_slice(self.data, self.pos, end);
        self.pos = end.max(self.pos);
        s
    }
    pub fn peek(&self, n: usize) -> &'a [u8] {
        cap_slice(self.data, self.pos, self.pos.saturating_add(n))
    }
    pub fn peek_u16(&self) -> LR<u16> {
        let s = self.data.get(self.pos..self.pos + 2).ok_or_else(|| self.eof())?;
        Ok(u16::from_le_bytes([s[0], s[1]]))
    }
    pub fn utf16(&mut self) -> LR<String> {
        if self.peek(3) != STR_MARKER {
            return Err(self.err("expected a string record"));
        }
        self.pos += 3;
        let mut n = self.u8()? as usize;
        if n == 0xFF {
            n = self.u16()? as usize;
            if n == 0xFFFF {
                n = self.u32()? as usize;
            }
        }
        let b = self.raw(n.saturating_mul(2));
        let units: Vec<u16> = b.chunks_exact(2).map(|c| u16::from_le_bytes([c[0], c[1]])).collect();
        Ok(char::decode_utf16(units).map(|r| r.unwrap_or('\u{FFFD}')).collect())
    }

    // ---- store map
    fn alloc(&mut self, e: Entry) -> LR<u64> {
        if self.objs.len() >= self.max_objects {
            return Err(self.err(format!("more than {} archive objects", self.max_objects)));
        }
        self.objs.push(e);
        Ok(self.base + self.objs.len() as u64 - 1)
    }

    fn entry(&self, slot: u64) -> Option<&Entry> {
        if slot >= self.base && slot - self.base < self.objs.len() as u64 {
            return self.objs.get((slot - self.base) as usize);
        }
        self.learned.get(&slot)
    }

    /// `(class name, cls)` when `slot` is a class entry.
    fn class_entry(&self, slot: u64) -> Option<(&str, Option<Cls>)> {
        match self.entry(slot)? {
            Entry::Class { name, cls } => Some((name.as_str(), *cls)),
            _ => None,
        }
    }

    /// The object kind at `slot` when it is a walked object.
    pub fn obj_cls(&self, slot: u64) -> Option<Cls> {
        match self.entry(slot)? {
            Entry::Obj { cls, .. } => Some(*cls),
            _ => None,
        }
    }
    pub fn obj_val(&self, slot: u64) -> Option<ValIdx> {
        match self.entry(slot)? {
            Entry::Obj { val, .. } => *val,
            _ => None,
        }
    }
    pub fn is_class_slot(&self, slot: u64) -> bool {
        matches!(self.entry(slot), Some(Entry::Class { .. }))
    }
    pub fn has_slot(&self, slot: u64) -> bool {
        self.entry(slot).is_some()
    }

    /// Every walked object in slot order.
    pub fn walked(&self) -> impl Iterator<Item = (u64, Cls, Option<ValIdx>)> + '_ {
        self.objs.iter().enumerate().filter_map(move |(i, e)| match e {
            Entry::Obj { cls, val } => Some((self.base + i as u64, *cls, *val)),
            _ => None,
        })
    }

    pub fn read_object(&mut self, expect: Option<Cls>) -> LR<ObjRef> {
        let tag = self.u16()?;
        if tag == 0 {
            return Ok(NONE_REF);
        }
        if tag == 0x7FFF {
            let big = self.u32()?;
            if big & 0x8000_0000 != 0 {
                return self.new_of_class((big & 0x7FFF_FFFF) as u64, expect);
            }
            return self.backref(big as u64);
        }
        if tag == 0xFFFF {
            let schema = self.u16()?;
            let _ = schema;
            let namelen = self.u16()? as usize;
            if namelen > 40 {
                return Err(self.err("implausible class name length"));
            }
            let raw = self.raw(namelen);
            if raw.len() < namelen || !raw.is_ascii() {
                // Python: `.decode('ascii')` on short or non-ASCII bytes
                return Err(if raw.len() < namelen { self.eof() } else { self.err("class name is not ASCII") });
            }
            let name = String::from_utf8_lossy(raw).into_owned();
            let cls = Cls::from_name(&name);
            let slot = self.alloc(Entry::Class { name: name.clone(), cls })?;
            self.class_slot.insert(name.clone(), slot);
            return self.new_obj(cls, &name);
        }
        if tag & 0x8000 != 0 {
            return self.new_of_class((tag & 0x7FFF) as u64, expect);
        }
        self.backref(tag as u64)
    }

    fn new_of_class(&mut self, cslot: u64, expect: Option<Cls>) -> LR<ObjRef> {
        let ent = self.class_entry(cslot).map(|(n, c)| (n.to_string(), c));
        let (name, cls) = match ent {
            Some(x) => x,
            None => {
                if self.entry(cslot).is_some() {
                    return Err(self.err(format!("class-ref to non-class slot {cslot}")));
                }
                let Some(exp) = expect else {
                    return Err(self.err(format!("class-ref to unknown slot {cslot}")));
                };
                // a class defined in the unwalked pre-model region: learn it from context
                self.learned.insert(cslot, Entry::Class { name: exp.name().into(), cls: Some(exp) });
                self.class_slot.insert(exp.name().into(), cslot);
                (exp.name().to_string(), Some(exp))
            }
        };
        self.new_obj(cls, &name)
    }

    fn new_obj(&mut self, cls: Option<Cls>, name: &str) -> LR<ObjRef> {
        let slot = self.alloc(Entry::Obj { cls: cls.unwrap_or(Cls::Curve), val: None })?;
        let Some(cls) = cls else {
            return Err(self.err(format!("no reader for class {name}")));
        };
        if self.depth >= self.max_depth {
            return Err(self.err(format!("objects nested deeper than {}", self.max_depth)));
        }
        let prev_slot = self.current_obj_slot;
        self.current_obj_slot = Some(slot);
        self.depth += 1;
        let r = self.read_value(cls);
        self.depth -= 1;
        self.current_obj_slot = prev_slot;
        let v = r?;
        let vi = self.vals.len();
        self.vals.push(v);
        let idx = (slot - self.base) as usize;
        self.objs[idx] = Entry::Obj { cls, val: Some(vi) };
        if matches!(cls, Cls::DimLinear | Cls::Text) {
            self.annot_watermark = Some(self.next_slot());
        }
        Ok(ObjRef { slot: Some(slot), name: Name::Cls(cls), val: Some(vi) })
    }

    /// File store-map index -> walker numbering through the burn bands; `None` for an index inside a band.
    fn translate_ref(&self, slot: u64) -> Option<u64> {
        let mut offset = 0;
        for &(start, width) in &self.burns {
            if slot < start {
                break;
            }
            if slot < start + width {
                return None;
            }
            offset += width;
        }
        Some(slot - offset)
    }

    fn backref(&mut self, slot: u64) -> LR<ObjRef> {
        let mut slot = slot;
        if !self.burns.is_empty() && slot >= self.burns[0].0 {
            match self.translate_ref(slot) {
                None => return Ok(ObjRef { slot: Some(slot), name: Name::Reserved, val: None }),
                Some(w) => slot = w,
            }
        }
        match self.entry(slot) {
            None => {
                if slot < self.walk_base {
                    return Ok(ObjRef { slot: Some(slot), name: Name::Premodel, val: None });
                }
                Err(self.err(format!("back-ref to unwalked slot {slot}")))
            }
            Some(Entry::Class { .. }) => Err(self.err(format!("back-ref to class slot {slot}"))),
            Some(Entry::Obj { cls, val }) => Ok(ObjRef { slot: Some(slot), name: Name::Cls(*cls), val: *val }),
        }
    }

    // ---- CLayer helpers
    pub fn is_class_ref(&self, p: usize, slot: u64) -> bool {
        let d = self.data;
        if slot < 0x7FFF {
            return p + 2 <= d.len() && u16::from_le_bytes([d[p], d[p + 1]]) as u64 == (0x8000 | slot);
        }
        p + 6 <= d.len() && u16::from_le_bytes([d[p], d[p + 1]]) == 0x7FFF && u32::from_le_bytes([d[p + 2], d[p + 3], d[p + 4], d[p + 5]]) as u64 == (0x8000_0000u64 | slot)
    }

    pub fn clayer_record_at(&self, at: usize, unmatched_class_ref: bool) -> bool {
        let d = self.data;
        if at + 2 > d.len() {
            return false;
        }
        let tag = u16::from_le_bytes([d[at], d[at + 1]]);
        if tag == 0xFFFF && at + 12 <= d.len() && u16::from_le_bytes([d[at + 4], d[at + 5]]) == 6 && &d[at + 6..at + 12] == b"CLayer" {
            return true;
        }
        if let Some(&cs) = self.class_slot.get("CLayer") {
            if self.is_class_ref(at, cs) {
                return true;
            }
        }
        unmatched_class_ref && (tag & 0x8000) != 0 && tag != 0xFFFF
    }

    fn skip_clayer_colour_ext(&mut self) {
        let (pad, ext) = (12usize, 16usize);
        if self.pos + ext + 2 > self.data.len() {
            return;
        }
        if (0..pad).any(|i| self.data[self.pos + i] != 0) {
            return;
        }
        if self.clayer_record_at(self.pos + ext, true) {
            self.pos += ext;
        }
    }

    fn skip_clayer_parent_ref(&mut self, self_slot: Option<u64>) {
        let Some(self_slot) = self_slot else { return };
        if self.pos + 4 > self.data.len() {
            return;
        }
        let a = u16::from_le_bytes([self.data[self.pos], self.data[self.pos + 1]]) as u64;
        let b = u16::from_le_bytes([self.data[self.pos + 2], self.data[self.pos + 3]]) as u64;
        if a == 0 || (a & 0x8000) != 0 || a == 0x7FFF || a == self_slot {
            return;
        }
        if b == 0 || (b & 0x8000) != 0 || b == 0x7FFF {
            return;
        }
        if self.obj_cls(a) != Some(Cls::Layer) || self.obj_cls(b) != Some(Cls::Layer) {
            return;
        }
        self.pos += 2;
    }

    /// Layer records past the declared count (v18: count=1 custom tag, then Layer0). Stops at the definition-list anchor.
    pub fn collect_trailing_layers(&mut self, layers_out: Option<&mut Vec<(u64, ValIdx)>>, slots_out: Option<&mut Vec<u64>>, unmatched_class_ref: bool) -> LR<()> {
        let mut layers_out = layers_out;
        let mut slots_out = slots_out;
        while self.pos + 2 <= self.data.len() {
            let tag = self.peek_u16()?;
            if tag == 0 {
                self.pos += 2;
                continue;
            }
            if !self.clayer_record_at(self.pos, unmatched_class_ref) {
                break;
            }
            let o = self.read_object(Some(Cls::Layer))?;
            if let (Some(s), Some(v)) = (o.slot, o.val) {
                if let Some(so) = slots_out.as_deref_mut() {
                    so.push(s);
                }
                if let Some(lo) = layers_out.as_deref_mut() {
                    lo.push((s, v));
                }
            }
        }
        Ok(())
    }

    fn plausible_list_tag(&self, at: usize) -> bool {
        if at + 2 > self.data.len() {
            return false;
        }
        let t = u16::from_le_bytes([self.data[at], self.data[at + 1]]);
        if matches!(t, 0x0000 | 0x7FFF | 0xFFFF) {
            return true;
        }
        if t & 0x8000 != 0 {
            return self.is_class_slot((t & 0x7FFF) as u64);
        }
        (t as u64) < self.next_slot()
    }

    /// SketchUp 2020 writes an empty-string filler record ahead of some counts; returns the corrected count and moves
    /// the cursor, or `None` when this is not that layout.
    pub fn retry_count_after_v20_filler(&mut self, count_pos: usize, limit: u32, check: bool) -> Option<u32> {
        let d = self.data;
        let mut marker_at = None;
        let hi = count_pos.saturating_add(12).min(d.len().saturating_sub(3));
        let mut i = count_pos;
        while i < hi {
            if d[i..i + 3] == STR_MARKER {
                marker_at = Some(i);
                break;
            }
            i += 1;
        }
        let m = marker_at?;
        if *d.get(m + 3)? != 0 {
            return None;
        }
        let mut at = m + 4;
        while at < d.len() && d[at] == 0 {
            at += 1;
        }
        for back in 0..4usize {
            if at < back {
                continue;
            }
            let at2 = at - back;
            if at2 < m + 4 || at2 + 4 > d.len() {
                continue;
            }
            let count = u32::from_le_bytes([d[at2], d[at2 + 1], d[at2 + 2], d[at2 + 3]]);
            if !(0 < count && count <= limit) {
                continue;
            }
            if check && !self.plausible_list_tag(at2 + 4) {
                continue;
            }
            self.pos = at2 + 4;
            return Some(count);
        }
        None
    }

    // ---- shared record blocks
    fn preamble(&mut self) -> LR<(Option<ValIdx>, Option<u64>)> {
        let o = self.read_object(Some(Cls::AttrContainer))?;
        if self.has_pid {
            let mask = self.u8()?;
            for bit in 0..8 {
                if mask & (1 << bit) != 0 {
                    self.u8()?;
                }
            }
        }
        Ok((o.val, o.slot))
    }

    fn early_preamble(&mut self) -> LR<()> {
        if self.ver < FIRST_V4 {
            return Ok(());
        }
        self.preamble().map(|_| ())
    }

    fn drawbase(&mut self) -> LR<Db> {
        let b = if self.ver < FIRST_V4 { self.raw(5) } else { self.raw(8) };
        let lay_cls = self.class_slot.get("CLayer").copied();
        let tag = self.peek_u16()? as u64;
        let layer: u16;
        if lay_cls.map_or(false, |c| tag == (0x8000 | c)) {
            self.read_object(Some(Cls::Layer))?;
            layer = 0;
        } else if tag == 0x7FFF {
            self.u16()?;
            let big = self.u32()?;
            if big & 0x8000_0000 != 0 {
                return Err(self.err("drawbase layer: unexpected class"));
            }
            layer = 0;
        } else {
            layer = self.u16()?;
        }
        if self.ver < FIRST_V4 {
            return Ok(Db { mat: 0, hidden: 0, soft: 0, smooth: 0, layer });
        }
        if b.len() < 7 {
            return Err(self.eof());
        }
        Ok(Db { mat: u16::from_le_bytes([b[0], b[1]]), hidden: b[2], soft: b[5], smooth: b[6], layer })
    }

    fn read_value(&mut self, cls: Cls) -> LR<Val> {
        match cls {
            Cls::Vertex => {
                self.early_preamble()?;
                Ok(Val::Vertex(self.f64n::<3>()?))
            }
            Cls::Edge => self.read_edge(),
            Cls::Curve => {
                self.preamble()?;
                self.u8()?;
                self.u32()?;
                Ok(Val::Curve)
            }
            Cls::ArcCurve => {
                self.preamble()?;
                self.raw(5);
                self.f64n::<14>()?;
                Ok(Val::ArcCurve)
            }
            Cls::EdgeUse => self.read_edgeuse(),
            Cls::Loop => self.read_loop(),
            Cls::Face => self.read_face(),
            Cls::Layer => self.read_layer(),
            Cls::Material => self.read_material(),
            Cls::Dib => self.read_dib().map(|(start, len)| Val::Dib { start, len }),
            Cls::AttrContainer => self.read_attr_container(),
            Cls::AttrNamed => self.read_attr_named(),
            Cls::Camera => {
                self.read_camera()?;
                Ok(Val::Camera)
            }
            Cls::Thumbnail => self.read_thumbnail(),
            Cls::Relationship => {
                self.preamble()?;
                self.entity_ref()?;
                self.entity_ref()?;
                Ok(Val::Relationship)
            }
            Cls::Definition => self.read_definition(),
            Cls::Image => self.read_image(),
            Cls::Instance | Cls::Group => self.read_instance(),
            Cls::Ftc => self.read_ftc(),
            Cls::CLine => self.read_constructionline(),
            Cls::CPoint => {
                self.preamble()?;
                self.drawbase()?;
                self.f64n::<3>()?;
                self.f64n::<3>()?;
                self.u8()?;
                Ok(Val::Other)
            }
            Cls::SectionPlane => self.read_sectionplane(),
            Cls::Font => {
                self.read_object(Some(Cls::AttrContainer))?;
                if self.has_pid {
                    let mask = self.u8()?;
                    self.raw(mask.count_ones() as usize);
                }
                self.utf16()?;
                self.raw(15);
                Ok(Val::Other)
            }
            Cls::DimLinear => self.read_dimlinear(),
            Cls::Text => self.read_text(),
        }
    }

    fn read_edge(&mut self) -> LR<Val> {
        self.preamble()?;
        let db = self.drawbase()?;
        let s1 = self.read_object(Some(Cls::Vertex))?;
        let s2 = self.read_object(Some(Cls::Vertex))?;
        let c = self.read_object(None)?;
        match c.name {
            Name::None | Name::Cls(Cls::Curve) | Name::Cls(Cls::ArcCurve) => {}
            _ => return Err(self.err("edge curve pointer resolved to something that is not a curve")),
        }
        Ok(Val::Edge { db, curve: c.slot, curve_val: c.val, v1: s1.slot, v2: s2.slot })
    }

    fn read_edgeuse(&mut self) -> LR<Val> {
        self.early_preamble()?;
        let es = self.read_object(Some(Cls::Edge))?;
        let sense = self.u8()?;
        let p0 = self.pos;
        let tag = self.u16()?;
        let ps: Option<u64>;
        if tag == 0x7FFF {
            let v = self.u32()?;
            if v & 0x8000_0000 != 0 {
                return Err(self.err("edge-use parent is a new object"));
            }
            ps = Some(v as u64);
        } else if tag == 0xFFFF || tag & 0x8000 != 0 {
            return Err(self.err("edge-use parent is a new object"));
        } else {
            ps = if tag != 0 { Some(tag as u64) } else { None };
        }
        let expected = self.current_loop.map(|l| l + self.cum_delta);
        if ps != expected {
            let delta = match (ps, expected) {
                (Some(p), Some(e)) if p > e => p - e,
                _ => 0,
            };
            if 0 < delta && delta <= 4096 && self.annot_watermark.is_some() {
                self.register_burn(delta);
            } else {
                self.pos = p0;
                return Err(self.err(format!("edge-use parent slot {ps:?} != current loop {expected:?}")));
            }
        }
        Ok(Val::EdgeUse { edge: es.slot, sense })
    }

    fn register_burn(&mut self, delta: u64) {
        let wm = self.annot_watermark.unwrap_or(0);
        self.burns.push((wm + self.cum_delta, delta));
        self.cum_delta += delta;
        self.annot_watermark = None;
        if let Some(l) = self.burn_stack.last_mut() {
            *l += 1;
        }
    }

    fn read_loop(&mut self) -> LR<Val> {
        let my_slot = self.next_slot() - 1;
        let prev = self.current_loop;
        self.current_loop = Some(my_slot);
        self.early_preamble()?;
        self.raw(2);
        let mut uses = vec![];
        loop {
            if self.peek_u16()? == 0 {
                self.pos += 2;
                break;
            }
            let o = self.read_object(Some(Cls::EdgeUse))?;
            uses.push(match o.val.map(|v| &self.vals[v]) {
                Some(Val::EdgeUse { edge, sense }) => Some((*edge, *sense)),
                _ => None,
            });
        }
        self.current_loop = prev;
        Ok(Val::Loop { uses })
    }

    fn read_face(&mut self) -> LR<Val> {
        let (attrs0, attrs_slot) = self.preamble()?;
        let mut db = self.drawbase()?;
        let mut attrs = attrs0;
        if self.ver < FIRST_V4 {
            if let Some(s) = attrs_slot {
                if self.obj_cls(s) == Some(Cls::Material) {
                    // SketchUp 3: the pointer that opens a face is its FRONT MATERIAL
                    db.mat = s as u16;
                    attrs = None;
                }
            }
        }
        let plane = self.f64n::<4>()?;
        let nloops = self.u32()?;
        if nloops > 10000 {
            return Err(self.err(format!("implausible loop count {nloops}")));
        }
        let mut loops = vec![];
        for _ in 0..nloops {
            let o = self.read_object(Some(Cls::Loop))?;
            loops.push(match o.val.map(|v| &self.vals[v]) {
                Some(Val::Loop { uses }) => Some(uses.clone()),
                _ => None,
            });
        }
        let back_mat = self.u16()?;
        Ok(Val::Face { db, plane, loops, back_mat, attrs })
    }

    fn read_attr_container(&mut self) -> LR<Val> {
        self.preamble()?;
        let mut children = vec![];
        loop {
            if self.peek_u16()? == 0 {
                self.pos += 2;
                break;
            }
            let o = self.read_object(Some(Cls::AttrNamed))?;
            children.push(o.val);
        }
        Ok(Val::Attrs { children })
    }

    fn read_typed(&mut self, t: u8, depth: usize) -> LR<()> {
        match t {
            0x00 => {}
            0x04 => {
                self.raw(4);
                if self.pos > self.data.len() {
                    return Err(self.eof());
                }
            }
            0x06 | 0x0C => {
                self.f64()?;
            }
            0x07 => {
                self.u8()?;
            }
            0x09 => {
                self.u32()?;
            }
            0x0A => {
                self.utf16()?;
            }
            0x0B => {
                let n = self.u32()?;
                if n > 100000 {
                    return Err(self.err("implausible attr array count"));
                }
                if depth >= 32 {
                    return Err(self.err("attribute arrays nested too deeply"));
                }
                for _ in 0..n {
                    let t2 = self.u8()?;
                    self.read_typed(t2, depth + 1)?;
                }
            }
            0x11 | 0x12 => {
                self.f64n::<3>()?;
            }
            _ => return Err(self.err(format!("unknown attribute value type {t:#x}"))),
        }
        Ok(())
    }

    fn read_attr_named(&mut self) -> LR<Val> {
        self.preamble()?;
        self.raw(4);
        let _dictname = self.utf16()?;
        loop {
            let key = self.utf16()?;
            if key.is_empty() {
                break;
            }
            let t = self.u8()?;
            self.read_typed(t, 0)?;
        }
        if self.ver >= 7 {
            self.u32()?;
        }
        Ok(Val::AttrNamed)
    }

    fn read_layer(&mut self) -> LR<Val> {
        self.early_preamble()?;
        let name = self.utf16()?;
        let mut mid: Vec<u8> = vec![];
        while mid.len() < 8 && self.peek(3) != STR_MARKER {
            // OpenSKP loops forever here at the end of the data; this is an error
            let b = self.u8()?;
            mid.push(b);
        }
        self.utf16()?;
        let flags = self.u16()?;
        let hidden = mid.first().copied().unwrap_or(0);
        if flags & 0x00FF != 0 {
            let tex = self.texture_block()?;
            self.raw(4);
            return Ok(Val::Layer { name, hidden, rgba: tex.rgba });
        }
        let rgba = self.raw(4);
        if rgba.len() < 4 {
            return Err(self.eof());
        }
        let rgba = [rgba[0], rgba[1], rgba[2], rgba[3]];
        self.utf16()?;
        self.raw(if self.ver < FIRST_V4 { 17 } else { 21 });
        self.skip_clayer_colour_ext();
        let cs = self.current_obj_slot;
        self.skip_clayer_parent_ref(cs);
        Ok(Val::Layer { name, hidden, rgba })
    }

    fn texture_block(&mut self) -> LR<TexBlock> {
        let (s, dib_data): (Option<u64>, Option<(usize, usize)>);
        if self.ver < FIRST_V4 {
            let (a, l) = self.read_dib()?;
            s = None;
            dib_data = Some((a, l));
        } else {
            self.pos = self.pos.checked_sub(1).ok_or_else(|| self.err("texture block at the start"))?;
            self.preamble()?;
            let o = self.read_object(Some(Cls::Dib))?;
            match o.val.map(|v| &self.vals[v]) {
                Some(Val::Dib { .. }) => {}
                _ => return Err(self.err("texture object is not a dib")),
            }
            s = o.slot;
            dib_data = None;
        }
        let m = find_in(self.data, &STR_MARKER, self.pos, self.pos + 28);
        let delta = m.map(|x| x as i64 - self.pos as i64).unwrap_or(-1 - self.pos as i64);
        if delta == 20 {
            self.u32()?;
        } else if delta != 16 {
            return Err(self.err("texture size block misaligned"));
        }
        let w = self.f64()?;
        let h = self.f64()?;
        let fname = self.utf16()?;
        let avg = self.raw(9);
        self.utf16()?;
        let blob = self.raw(8);
        let opacity = self.f64()?;
        let use_op = self.u8()?;
        if avg.len() < 4 || blob.len() < 5 {
            return Err(self.eof());
        }
        let colorized = blob[4] != 0 || avg[3] == 0xFF;
        Ok(TexBlock { rgba: [avg[0], avg[1], avg[2], avg[3]], opacity, use_opacity: use_op, tex_dib: s, tex_data: dib_data, tex_w: w, tex_h: h, tex_file: fname, colorized })
    }

    fn read_material(&mut self) -> LR<Val> {
        self.early_preamble()?;
        let name = self.utf16()?;
        let texflag = self.u16()?;
        if texflag == 0 {
            let rgba = self.raw(4);
            if rgba.len() < 4 {
                return Err(self.eof());
            }
            let rgba = [rgba[0], rgba[1], rgba[2], rgba[3]];
            self.utf16()?;
            self.raw(8);
            let opacity = self.f64()?;
            let use_op = self.u8()?;
            Ok(Val::Material { name, rgba, opacity, use_opacity: use_op, tex: None })
        } else {
            let t = self.texture_block()?;
            Ok(Val::Material { name, rgba: t.rgba, opacity: t.opacity, use_opacity: t.use_opacity, tex: Some(t) })
        }
    }

    pub fn read_dib(&mut self) -> LR<(usize, usize)> {
        let _subtype = self.u32()?;
        let length = self.u32()? as usize;
        if length > self.data.len() {
            return Err(self.err(format!("implausible dib length {length}")));
        }
        let start = self.pos;
        let b = self.raw(length);
        Ok((start, b.len()))
    }

    fn read_ftc(&mut self) -> LR<Val> {
        self.preamble()?;
        self.u32()?;
        let ks = self.f64n::<24>()?;
        for _ in 0..2 {
            let n = self.u32()? as usize;
            let need = n.checked_mul(32).ok_or_else(|| self.eof())?;
            if self.pos.checked_add(need).map_or(true, |e| e > self.data.len()) {
                return Err(self.eof());
            }
            self.pos += need;
        }
        self.u32()?;
        self.u32()?;
        let mut front = [0.0; 9];
        let mut back = [0.0; 9];
        front.copy_from_slice(&ks[0..9]);
        back.copy_from_slice(&ks[12..21]);
        Ok(Val::Ftc { front, back })
    }

    fn read_camera(&mut self) -> LR<()> {
        self.raw(137);
        self.u16()?;
        self.utf16()?;
        self.raw(33);
        Ok(())
    }

    fn read_thumbnail(&mut self) -> LR<Val> {
        self.preamble()?;
        if self.ver < FIRST_V4 {
            self.raw(129);
            return Ok(Val::Thumbnail);
        }
        self.read_object(Some(Cls::Camera))?;
        self.read_object(Some(Cls::Dib))?;
        Ok(Val::Thumbnail)
    }

    fn read_image(&mut self) -> LR<Val> {
        self.preamble()?;
        let db = self.drawbase()?;
        let d = self.read_object(None)?;
        let xform = self.f64n::<12>()?;
        self.f64()?;
        self.utf16()?;
        let guid = hex(self.raw(16));
        Ok(Val::Image { db, def: d.slot, xform, guid })
    }

    /// A reference-to-entity tag that tolerates a slot the walk has not reached yet (forward references).
    fn entity_ref(&mut self) -> LR<Option<u64>> {
        let tag = self.u16()?;
        if tag == 0 {
            return Ok(None);
        }
        if tag == 0x7FFF {
            let big = self.u32()?;
            if big & 0x8000_0000 != 0 {
                return Err(self.err("entity ref is a new object"));
            }
            return Ok(Some(big as u64));
        }
        if tag == 0xFFFF || tag & 0x8000 != 0 {
            return Err(self.err("entity ref is a new object"));
        }
        Ok(Some(tag as u64))
    }

    fn strict_next_tag(&self, at: usize, allow_null: bool) -> bool {
        if at + 2 > self.data.len() {
            return false;
        }
        let t = u16::from_le_bytes([self.data[at], self.data[at + 1]]);
        if t == 0 {
            return allow_null;
        }
        if t == 0x7FFF || t == 0xFFFF {
            return true;
        }
        if t & 0x8000 != 0 {
            return self.is_class_slot((t & 0x7FFF) as u64);
        }
        false
    }

    fn read_constructionline(&mut self) -> LR<Val> {
        self.preamble()?;
        self.drawbase()?;
        self.f64n::<3>()?;
        self.f64n::<3>()?;
        self.f64n::<2>()?;
        let k = match self.cline_tail {
            Some(k) => k,
            None => {
                let default = if self.ver == 17 { 7 } else { 4 };
                let mut order = vec![default];
                order.extend([0usize, 4, 7].into_iter().filter(|&c| c != default));
                let mut k = None;
                for cand in order {
                    if self.strict_next_tag(self.pos + cand, false) {
                        k = Some(cand);
                        break;
                    }
                }
                if k.is_none() {
                    for cand in [4usize, 0, 7] {
                        if self.strict_next_tag(self.pos + cand, true) {
                            k = Some(cand);
                            break;
                        }
                    }
                }
                let k = k.unwrap_or(4);
                self.cline_tail = Some(k);
                k
            }
        };
        self.raw(k);
        Ok(Val::Other)
    }

    fn read_sectionplane(&mut self) -> LR<Val> {
        self.preamble()?;
        self.drawbase()?;
        let first = self.f64_at(self.pos)?;
        if !(first.abs() <= 1.0001) {
            self.read_object(None)?;
        }
        self.f64n::<4>()?;
        if self.peek(3) == STR_MARKER {
            self.utf16()?;
            self.utf16()?;
        }
        Ok(Val::Other)
    }

    fn f64_at(&self, p: usize) -> LR<f64> {
        let s = self.data.get(p..p + 8).ok_or_else(|| self.eof())?;
        let mut b = [0u8; 8];
        b.copy_from_slice(s);
        Ok(f64::from_le_bytes(b))
    }

    fn read_dimlinear(&mut self) -> LR<Val> {
        self.preamble()?;
        self.drawbase()?;
        self.utf16()?;
        self.read_object(Some(Cls::Font))?;
        self.raw(37);
        self.entity_ref()?;
        self.connection_paths()?;
        self.raw(32);
        self.entity_ref()?;
        self.connection_paths()?;
        self.raw(72);
        Ok(Val::Other)
    }

    fn connection_paths(&mut self) -> LR<()> {
        self.entity_ref()?;
        for _ in 0..2 {
            let count = self.u32()?;
            if count > 1000 {
                return Err(self.err("implausible dimension path length"));
            }
            for _ in 0..count {
                self.path_ref()?;
            }
        }
        Ok(())
    }

    fn path_ref(&mut self) -> LR<Option<u64>> {
        let tag = self.peek_u16()?;
        let mut new = tag == 0xFFFF || (tag != 0x7FFF && tag & 0x8000 != 0);
        if tag == 0x7FFF {
            let s = self.data.get(self.pos + 2..self.pos + 6).ok_or_else(|| self.eof())?;
            let big = u32::from_le_bytes([s[0], s[1], s[2], s[3]]);
            new = big & 0x8000_0000 != 0;
        }
        if new {
            let o = self.read_object(None)?;
            return Ok(o.slot);
        }
        self.entity_ref()
    }

    fn read_text(&mut self) -> LR<Val> {
        self.preamble()?;
        self.drawbase()?;
        self.read_object(Some(Cls::Font))?;
        let start = self.pos;
        let mut p = start;
        let idx;
        loop {
            let Some(i) = find_in(self.data, &STR_MARKER, p, start + 512) else {
                return Err(self.err("text delimiter not found"));
            };
            if i < 11 {
                return Err(self.eof());
            }
            let blk = &self.data[i - 11..i];
            let arrow = u32::from_le_bytes([blk[6], blk[7], blk[8], blk[9]]);
            if blk[..4] == [1, 0, 0, 0] && arrow <= 8 && blk[10] == 1 {
                idx = i;
                break;
            }
            p = i + 3;
        }
        self.raw(idx - self.pos);
        self.utf16()?;
        self.raw(5);
        while self.peek(2) == [0xFF, 0x7F] {
            let s = self.data.get(self.pos + 2..self.pos + 6).ok_or_else(|| self.eof())?;
            let val = u32::from_le_bytes([s[0], s[1], s[2], s[3]]);
            if val & 0x8000_0000 != 0 {
                break;
            }
            self.raw(6);
        }
        Ok(Val::Other)
    }

    pub fn read_entity_list(&mut self, count: usize, owner: Owner) -> LR<Vec<(Option<u64>, Option<ValIdx>)>> {
        self.burn_stack.push(0);
        let mut ents = vec![];
        let r = self.read_entity_list_inner(count, owner, &mut ents);
        self.burn_stack.pop();
        r.map(|_| ents)
    }

    fn read_entity_list_inner(&mut self, count: usize, owner: Owner, ents: &mut Vec<(Option<u64>, Option<ValIdx>)>) -> LR<()> {
        while ents.len() < count {
            let p = self.pos;
            if owner == Owner::Def && self.burn_stack.last().copied().unwrap_or(0) != 0 {
                let d = self.data;
                let z = d.get(p..p + 4).map_or(false, |s| s == [0, 0, 0, 0]);
                if z && d.get(p + 22..p + 25) == Some(&STR_MARKER[..]) {
                    break;
                }
            }
            match self.read_object(None) {
                Ok(o) => ents.push((o.slot, o.val)),
                Err(e) if !e.eof => {
                    if owner == Owner::Root {
                        self.pos = p;
                        break;
                    }
                    if owner == Owner::Def && self.burn_stack.last().copied().unwrap_or(0) != 0 {
                        self.pos = p;
                        break;
                    }
                    return Err(e);
                }
                Err(e) => return Err(e),
            }
        }
        Ok(())
    }

    fn reanchor_on_string_marker(&self, pos: usize) -> usize {
        let d = self.data;
        let at = |a: usize| d.get(a + 16..a + 19) == Some(&STR_MARKER[..]);
        if at(pos) {
            return pos;
        }
        for skip in [-1i64, 1, -2, 2, -3, 3, -4, 4] {
            let a = pos as i64 + skip;
            if a >= 0 && at(a as usize) {
                return a as usize;
            }
        }
        pos
    }

    fn read_definition(&mut self) -> LR<Val> {
        self.preamble()?;
        self.raw(if self.ver >= 17 {
            22
        } else if self.ver >= FIRST_V4 {
            20
        } else {
            13
        });
        let nlayers = self.u32()?;
        if nlayers > 10000 {
            return Err(self.err("implausible def layer count"));
        }
        let mut got = 0;
        while got < nlayers {
            if self.peek_u16()? == 0 {
                self.pos += 2;
                continue;
            }
            self.read_object(Some(Cls::Layer))?;
            got += 1;
        }
        let decl = self.u16()?;
        if decl == 0x7FFF {
            self.u32()?;
        }
        let mut count = None;
        if self.ver >= 20 {
            count = self.retry_count_after_v20_filler(self.pos, 5_000_000, true);
        }
        let mut count = match count {
            Some(c) => c,
            None => {
                self.u32()?;
                self.u32()?
            }
        };
        if count > 5_000_000 || count == 0 {
            if let Some(r) = self.retry_count_after_v20_filler(self.pos.wrapping_sub(4), 5_000_000, true) {
                count = r;
            }
        }
        if count > 5_000_000 {
            return Err(self.err("implausible def entity count"));
        }
        let ents = self.read_entity_list(count as usize, Owner::Def)?;
        let mut nrel = self.u32()?;
        if nrel > 100000 {
            if let Some(r) = self.retry_count_after_v20_filler(self.pos.wrapping_sub(4), 100_000, true) {
                nrel = r;
            }
        }
        if nrel > 100000 {
            return Err(self.err("definition list misaligned"));
        }
        for _ in 0..nrel {
            self.read_object(Some(Cls::Relationship))?;
        }
        self.u16()?;
        self.pos = self.reanchor_on_string_marker(self.pos);
        let guid = hex(self.raw(16));
        let name = self.utf16()?;
        self.utf16()?;
        self.utf16()?;
        self.u32()?;
        let mut tpos = None;
        let thumb_slot = self.class_slot.get("CThumbnail").copied();
        for off in 0..96usize {
            let p = self.pos + off;
            let d = self.data;
            if d.get(p..p + 2) == Some(&[0xFF, 0xFF][..]) && d.get(p + 4..p + 6) == Some(&[0x0A, 0x00][..]) && d.get(p + 6..p + 16) == Some(&b"CThumbnail"[..]) {
                tpos = Some(p);
                break;
            }
            if let Some(ts) = thumb_slot {
                if self.is_class_ref(p, ts) {
                    tpos = Some(p);
                    break;
                }
            }
        }
        let Some(tpos) = tpos else {
            return Err(self.err("definition tail: thumbnail not found"));
        };
        let gap = self.raw(tpos - self.pos);
        let behavior = if gap.len() >= 9 { gap[gap.len() - 9] } else { 0 };
        self.read_object(Some(Cls::Thumbnail))?;
        Ok(Val::Definition { name, guid, ents, faces_camera: behavior & 1 != 0, shadows_face_sun: behavior & 2 != 0 })
    }

    fn read_instance(&mut self) -> LR<Val> {
        self.preamble()?;
        let db = self.drawbase()?;
        let d = self.read_object(Some(Cls::Definition))?;
        if d.name != Name::Cls(Cls::Definition) {
            return Err(self.err("instance definition ref is not a component definition"));
        }
        let xf = self.f64n::<13>()?;
        let name = if self.ver >= FIRST_V4 || self.peek(3) == STR_MARKER { self.utf16()? } else { String::new() };
        let guid = if self.ver >= 14 { hex(self.raw(16)) } else { String::new() };
        Ok(Val::Instance { db, def: d.slot, xf, name, guid })
    }

    /// Run the model prefix of the walk: used by the slot-base probe and the real walk.
    pub fn set_cursor(&mut self, pos: usize) {
        self.pos = pos;
    }
}

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum Owner {
    Def,
    Root,
}

pub fn hex(b: &[u8]) -> String {
    let mut s = String::with_capacity(b.len() * 2);
    for x in b {
        s.push_str(&format!("{x:02X}"));
    }
    s
}
