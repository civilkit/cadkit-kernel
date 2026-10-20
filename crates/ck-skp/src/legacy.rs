//! Legacy container driver (SketchUp 3 to 2020): finds the anchor of the model part of the archive, calibrates the
//! store-map slot base, walks the material list, layer list, definitions and root entity list, and builds the
//! [`Model`].
//!
//! Part of ck-skp, a Rust reimplementation of the SketchUp reader of OpenSKP (MIT, (c) Ahsan Mehmood), see NOTICE.
//! Follows `_walk`, `_probe_layer_anchor_bases`, `_walk_model` and `full_parse_legacy` of OpenSKP's `legacy.py`.
//! Scenes (pages), dimensions, texts, section planes and construction geometry are walked for alignment but not kept.

use crate::error::{Limits, SkpError, R};
use crate::legacy_ar::*;
use crate::model::*;
use crate::vff::{header_version, version_major};
use std::collections::{HashMap, HashSet};

/// True for a classic MFC container (`CVersionMap` in the header, no ZIP signature near the start).
pub fn is_legacy(data: &[u8]) -> bool {
    if !data.starts_with(&crate::vff::MAGIC) {
        return false;
    }
    if find_in(data, b"PK\x03\x04", 0, 0x100).is_some() {
        return false;
    }
    find_in(data, b"CVersionMap", 0, 0x200).is_some()
}

fn lerr(e: LErr) -> SkpError {
    SkpError::at(if e.eof { "legacy.eof" } else { "legacy.parse" }, e.pos, e.msg)
}

/// First `FF FF ?? ??` + u16 `len` + `name` match.
fn find_class_record(data: &[u8], len: u16, name: &[u8]) -> Option<usize> {
    let mut from = 0;
    while let Some(i) = find_in(data, name, from, data.len()) {
        if i >= 6 && data[i - 6] == 0xFF && data[i - 5] == 0xFF && u16::from_le_bytes([data[i - 2], data[i - 1]]) == len {
            return Some(i - 6);
        }
        from = i + 1;
    }
    None
}

fn header_ver(data: &[u8]) -> Option<u32> {
    let head: Vec<u8> = data[..data.len().min(0x60)].iter().copied().filter(|&b| b != 0).collect();
    let mut i = 0;
    while i < head.len() {
        if head[i] == b'{' {
            let mut j = i + 1;
            while j < head.len() && head[j].is_ascii_digit() {
                j += 1;
            }
            if j > i + 1 && j < head.len() && head[j] == b'.' {
                return std::str::from_utf8(&head[i + 1..j]).ok()?.parse().ok();
            }
        }
        i += 1;
    }
    None
}

struct Walked<'a> {
    ar: Ar<'a>,
    root: Vec<(Option<u64>, Option<ValIdx>)>,
    layers: Vec<(u64, ValIdx)>,
    materials: Vec<(u64, ValIdx)>,
}

fn bootstrap_two_materials(data: &[u8], ver: u32, mat_hdr: usize, lim: &Limits) -> Result<u64, LErr> {
    let mut boot = Ar::new(data, ver, lim.max_obj_depth, lim.max_objects);
    boot.set_base(1 << 20);
    boot.set_cursor(mat_hdr);
    boot.read_object(Some(Cls::Material))?;
    let tag = boot.peek_u16()?;
    if tag == 0xFFFF || tag & 0x8000 == 0 {
        return Err(boot.ctx_err("cannot bootstrap the slot base"));
    }
    Ok((tag & 0x7FFF) as u64)
}

fn probe_layer_anchor_bases(data: &[u8], ver: u32, start: usize, mat_count: u32, lim: &Limits) -> Result<Vec<u64>, LErr> {
    let mut boot = Ar::new(data, ver, lim.max_obj_depth, lim.max_objects);
    let b0: u64 = 1 << 20;
    boot.set_base(b0);
    boot.set_cursor(start);
    for _ in 0..mat_count {
        boot.read_object(Some(Cls::Material))?;
    }
    boot.u32()?;
    if ver >= 17 {
        boot.u8()?;
    }
    let layer_count = if ver < FIRST_V4 {
        0
    } else {
        let c = boot.u32()?;
        if !(1..=100000).contains(&c) {
            return Err(boot.ctx_err("implausible layer count in base probe"));
        }
        c
    };
    let mut layer_slots: Vec<u64> = vec![];
    for _ in 0..layer_count {
        let o = boot.read_object(Some(Cls::Layer))?;
        if let Some(s) = o.slot {
            layer_slots.push(s);
        }
    }
    boot.collect_trailing_layers(None, Some(&mut layer_slots), true)?;
    let o = boot.read_object(None)?;
    if o.name != Name::Premodel {
        return Err(boot.ctx_err("base probe: anchor did not resolve to the pre-model region"));
    }
    let s = o.slot.unwrap_or(0);
    Ok(layer_slots.iter().filter_map(|&rel| (s + b0).checked_sub(rel).filter(|&b| 0 < b && b < b0)).collect())
}

fn walk_model<'a>(data: &'a [u8], ver: u32, start: usize, mat_count: u32, base: u64, lim: &Limits) -> Result<Walked<'a>, LErr> {
    let mut ar = Ar::new(data, ver, lim.max_obj_depth, lim.max_objects);
    ar.set_base(base);
    ar.set_cursor(start);
    let mut materials = vec![];
    for _ in 0..mat_count {
        let o = ar.read_object(Some(Cls::Material))?;
        if let (Some(s), Some(v)) = (o.slot, o.val) {
            materials.push((s, v));
        }
    }
    ar.u32()?;
    if ver >= 17 {
        ar.u8()?;
    }
    let layer_count: u64 = if ver < FIRST_V4 { 1 << 30 } else { ar.u32()? as u64 };
    if layer_count > 100000 && ver >= FIRST_V4 {
        return Err(ar.ctx_err("implausible layer count"));
    }
    let mut layers: Vec<(u64, ValIdx)> = vec![];
    while (layers.len() as u64) < layer_count {
        let tag = ar.peek_u16()?;
        if tag == 0 {
            ar.pos += 2;
            continue;
        }
        if tag != 0xFFFF && tag & 0x8000 == 0 {
            break;
        }
        let o = ar.read_object(Some(Cls::Layer))?;
        match (o.slot, o.val) {
            (Some(s), Some(v)) => layers.push((s, v)),
            _ => continue,
        }
    }
    ar.collect_trailing_layers(Some(&mut layers), None, false)?;

    let o = ar.read_object(None)?;
    if o.name != Name::Cls(Cls::Layer) {
        return Err(ar.ctx_err("definition-list anchor is not a layer"));
    }
    let mut def_count = ar.u32()?;
    if def_count > 1_000_000 {
        if let Some(r) = ar.retry_count_after_v20_filler(ar.pos.wrapping_sub(4), 1_000_000, true) {
            def_count = r;
        }
    }
    if def_count > 1_000_000 {
        return Err(ar.ctx_err("implausible definition count"));
    }
    for _ in 0..def_count {
        ar.read_object(Some(Cls::Definition))?;
    }
    // trailing definitions, back-to-back
    let def_cls = ar.class_slot.get("CComponentDefinition").copied();
    loop {
        let tag = ar.peek_u16()?;
        let mut is_def = def_cls.map_or(false, |c| tag as u64 == (0x8000 | c));
        if !is_def && tag == 0xFFFF && ar.peek(26).get(6..26) == Some(&b"CComponentDefinition"[..]) {
            is_def = true;
        }
        if !is_def {
            break;
        }
        ar.read_object(None)?;
    }
    let mut root_count = ar.u32()?;
    if root_count > 5_000_000 {
        if let Some(r) = ar.retry_count_after_v20_filler(ar.pos.wrapping_sub(4), 5_000_000, true) {
            root_count = r;
        }
    }
    if root_count > 5_000_000 {
        return Err(ar.ctx_err("implausible root entity count"));
    }
    let root = ar.read_entity_list(root_count as usize, Owner::Root)?;
    Ok(Walked { ar, root, layers, materials })
}

fn walk<'a>(data: &'a [u8], lim: &Limits) -> R<(Walked<'a>, u32)> {
    let ver = header_ver(data).ok_or_else(|| SkpError::new("legacy.parse", "no version string in header"))?;
    let mc = find_class_record(data, 9, b"CMaterial");
    let (start, mat_count);
    if let Some(s) = mc {
        start = s;
        let c = s.checked_sub(4).and_then(|p| data.get(p..p + 4)).ok_or_else(|| SkpError::at("legacy.eof", s, "material count before the start of the file"))?;
        mat_count = u32::from_le_bytes([c[0], c[1], c[2], c[3]]);
        if mat_count > 100000 {
            return Err(SkpError::at("legacy.parse", s, "implausible material count"));
        }
    } else {
        let l = find_class_record(data, 6, b"CLayer").ok_or_else(|| SkpError::new("legacy.parse", "no CMaterial or CLayer class record found"))?;
        mat_count = 0;
        start = l.checked_sub(if ver >= 17 { 9 } else { 8 }).ok_or_else(|| SkpError::at("legacy.eof", l, "layer list before the start of the file"))?;
    }
    let bases: Result<Vec<u64>, LErr> = if mat_count >= 2 { bootstrap_two_materials(data, ver, start, lim).map(|b| vec![b]) } else { probe_layer_anchor_bases(data, ver, start, mat_count, lim) };
    let bases = bases.map_err(lerr)?;
    let mut last: Option<LErr> = None;
    for base in bases {
        match walk_model(data, ver, start, mat_count, base, lim) {
            Ok(w) => return Ok((w, ver)),
            Err(e) => last = Some(e),
        }
    }
    match last {
        Some(e) => Err(lerr(e)),
        None => Err(SkpError::new("legacy.parse", "no viable slot base candidate")),
    }
}

struct Filler<'w, 'a> {
    w: &'w Walked<'a>,
}

impl Filler<'_, '_> {
    fn add_edge(&self, def: &mut Definition, slot: u64, val: ValIdx) {
        if def.edges.contains(slot) {
            return;
        }
        let Val::Edge { db, curve, v1, v2, .. } = &self.w.ar.vals[val] else { return };
        for vs in [v1, v2].into_iter().flatten() {
            if !def.vertices.contains(*vs) {
                if let Some(Val::Vertex(p)) = self.w.ar.obj_val(*vs).map(|v| &self.w.ar.vals[v]) {
                    def.vertices.insert(*vs, *p);
                }
            }
        }
        let mut flags = 0u8;
        if db.soft != 0 {
            flags |= 0x08;
        }
        if db.smooth != 0 {
            flags |= 0x10;
        }
        if db.hidden != 0 {
            flags |= 0x01;
        }
        def.edges.insert(slot, Edge { v1: v1.unwrap_or(NO_ID), v2: v2.unwrap_or(NO_ID), flags, layer: if db.layer != 0 { Some(db.layer as u64) } else { None }, curve: *curve });
    }

    fn fill(&self, def: &mut Definition, ents: &[(Option<u64>, Option<ValIdx>)]) {
        let ar = &self.w.ar;
        for &(s, v) in ents {
            let (Some(s), Some(v)) = (s, v) else { continue };
            match &ar.vals[v] {
                Val::Edge { .. } => self.add_edge(def, s, v),
                Val::Face { db, plane, loops, back_mat, attrs } => {
                    let mut out_loops: Vec<Vec<CoEdge>> = vec![];
                    for lp in loops {
                        let mut l: Vec<CoEdge> = vec![];
                        let Some(lp) = lp else {
                            out_loops.push(l);
                            continue;
                        };
                        for u in lp.iter().flatten() {
                            let (Some(es), sense) = (u.0, u.1) else { continue };
                            let Some(ev) = ar.obj_val(es) else { continue };
                            self.add_edge(def, es, ev);
                            l.push((es, if sense != 0 { -1 } else { 1 }));
                        }
                        out_loops.push(l);
                    }
                    let (mut uf, mut ub) = (None, None);
                    if let Some(a) = attrs {
                        if let Val::Attrs { children } = &ar.vals[*a] {
                            for c in children.iter().flatten() {
                                if let Val::Ftc { front, back } = &ar.vals[*c] {
                                    uf = Some(*front);
                                    ub = Some(*back);
                                }
                            }
                        }
                    }
                    def.faces.insert(
                        s,
                        Face {
                            loops: out_loops,
                            normal: [plane[0], plane[1], plane[2]],
                            material_id: if db.mat != 0 { Some(db.mat as u64) } else { None },
                            back_material_id: if *back_mat != 0 { Some(*back_mat as u64) } else { None },
                            layer: if db.layer != 0 { Some(db.layer as u64) } else { None },
                            uv_front: uf,
                            uv_back: ub,
                            hidden: db.hidden != 0,
                        },
                    );
                }
                Val::Instance { db, def: d, xf, name, guid } => {
                    def.instances.push(Instance { name: name.clone(), def: *d, guid: guid.clone(), matrix: Some(*xf), material_id: if db.mat != 0 { Some(db.mat as u64) } else { None }, hidden: db.hidden != 0, layer: if db.layer != 0 { Some(db.layer as u64) } else { None } });
                }
                Val::Image { db, def: d, xform, guid } => {
                    let mut m = [1.0f64; 13];
                    m[..12].copy_from_slice(xform);
                    def.instances.push(Instance { name: String::new(), def: *d, guid: guid.clone(), matrix: Some(m), material_id: if db.mat != 0 { Some(db.mat as u64) } else { None }, hidden: db.hidden != 0, layer: if db.layer != 0 { Some(db.layer as u64) } else { None } });
                }
                _ => {}
            }
        }
    }
}

/// Parse a legacy (MFC) file.
pub fn parse(data: &[u8], lim: &Limits) -> R<Model> {
    let mut m = Model::new(Container::Legacy);
    m.version = header_version(data);
    m.version_major = version_major(&m.version);
    let (w, ver) = walk(data, lim)?;
    if m.version_major.is_none() {
        m.version_major = Some(ver);
    }
    let ar = &w.ar;
    let mut tex_bytes = 0usize;
    // materials
    for &(s, v) in &w.materials {
        let Val::Material { name, rgba, opacity, use_opacity, tex } = &ar.vals[v] else { continue };
        let op = if *use_opacity != 0 { (1.0 - opacity).clamp(0.0, 1.0) } else { 1.0 };
        let op = if op.is_nan() { 1.0 } else { op };
        let mut texture = None;
        let mut colorized = false;
        if let Some(t) = tex {
            colorized = t.colorized;
            let range = match t.tex_dib.and_then(|ds| ar.obj_val(ds)).map(|dv| &ar.vals[dv]) {
                Some(Val::Dib { start, len }) => Some((*start, *len)),
                _ => t.tex_data,
            };
            let bytes = range.and_then(|(a, l)| data.get(a..a + l)).filter(|b| tex_bytes + b.len() <= lim.max_texture_bytes);
            if let Some(b) = bytes {
                tex_bytes += b.len();
            }
            let ext = if bytes.map_or(false, |b| b.starts_with(&[0x89, b'P', b'N', b'G'])) { ".png" } else { ".jpg" };
            let fname = if !t.tex_file.is_empty() { t.tex_file.clone() } else { format!("{name}{ext}") };
            texture = Some(Texture { filename: fname, x_scale: if t.tex_w.is_finite() { t.tex_w } else { 0.0 }, y_scale: if t.tex_h.is_finite() { t.tex_h } else { 0.0 }, data: bytes.map(|b| b.to_vec()) });
        }
        let idx = m.materials.len();
        m.materials.push(Material { name: name.clone(), rgba: *rgba, opacity: op, colorized, colorize_type: if colorized { 1 } else { 0 }, texture });
        m.material_by_name.insert(name.clone(), idx);
        m.material_id_to_name.insert(s, name.clone());
    }
    // layers
    let mut seen: HashSet<String> = HashSet::new();
    let mut order: Vec<String> = vec![];
    let mut by_name: HashMap<String, Layer> = HashMap::new();
    for &(s, v) in &w.layers {
        let Val::Layer { name, hidden, rgba } = &ar.vals[v] else { continue };
        if seen.insert(name.clone()) {
            order.push(name.clone());
        }
        by_name.insert(name.clone(), Layer { name: name.clone(), rgb: [rgba[0], rgba[1], rgba[2]], hidden: *hidden != 0 });
        m.layer_id_to_name.insert(s, name.clone());
    }
    if !seen.contains("Layer0") {
        order.push("Layer0".into());
        by_name.insert("Layer0".into(), Layer { name: "Layer0".into(), rgb: [136, 136, 136], hidden: false });
    }
    m.layers = order.into_iter().filter_map(|n| by_name.remove(&n)).collect();

    // definitions: every walked CComponentDefinition, in slot order
    let image_defs: HashSet<u64> = ar.walked().filter_map(|(_, c, v)| if c == Cls::Image { v } else { None }).filter_map(|v| if let Val::Image { def, .. } = &ar.vals[v] { *def } else { None }).collect();
    let filler = Filler { w: &w };
    for (s, cls, v) in ar.walked() {
        if cls != Cls::Definition {
            continue;
        }
        let Some(v) = v else { continue };
        let Val::Definition { name, guid, ents, faces_camera, shadows_face_sun } = &ar.vals[v] else { continue };
        let mut d = Definition { id: Some(s), guid: guid.clone(), name: name.clone(), always_faces_camera: *faces_camera, shadows_face_sun: *shadows_face_sun, is_image: image_defs.contains(&s), ..Default::default() };
        filler.fill(&mut d, ents);
        m.definitions.insert(s, d);
    }
    let mut root = Definition::default();
    filler.fill(&mut root, &w.root);
    m.root = root;
    Ok(m)
}
