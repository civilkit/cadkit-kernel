//! Modern container (SketchUp 2021+, "VFF"): binary header, embedded ZIP with `model.dat` (TLV), `materials/*/material.xml`,
//! `styles/*/style.xml`, `meta/meta.dat`.
//!
//! Part of ck-skp, a Rust reimplementation of the SketchUp reader of OpenSKP (MIT, (c) Ahsan Mehmood), see NOTICE.
//! Follows `full_parse` and `_extract_geometry_from_nodes` of OpenSKP's Python package (`_core.py`) and `core.cpp`;
//! tag meanings are from OpenSKP's `docs/BINARY_FORMAT.md` (reverse engineered by that project).

use crate::error::{Limits, SkpError, R};
use crate::model::*;
use crate::tlv::*;
use crate::xml;
use crate::zip::{find_local_header, Zip, ZipEntry};
use std::collections::HashMap;

pub const MAGIC: [u8; 4] = [0xFF, 0xFE, 0xFF, 0x0E];

/// Version text between the second `FF FE FF` marker and the braces, e.g. `{25.0.575}`.
pub fn header_version(data: &[u8]) -> String {
    let head = &data[..data.len().min(512)];
    let mut second = None;
    let mut i = 4;
    while i + 3 <= head.len() {
        if head[i..i + 3] == [0xFF, 0xFE, 0xFF] {
            second = Some(i);
            break;
        }
        i += 1;
    }
    if let Some(s) = second {
        let tail = &head[(s + 4).min(head.len())..];
        let units: Vec<u16> = tail.chunks_exact(2).map(|c| u16::from_le_bytes([c[0], c[1]])).collect();
        let text: String = char::decode_utf16(units.iter().copied()).filter_map(|r| r.ok()).collect();
        if let (Some(a), Some(b)) = (text.find('{'), text.find('}')) {
            if b > a {
                return text[a..=b].to_string();
            }
        }
    }
    "unknown".into()
}

/// First number of a version string (`{25.0.575}` -> 25).
pub fn version_major(v: &str) -> Option<u32> {
    let s = v.trim_start_matches('{');
    s.split(|c: char| !c.is_ascii_digit()).next()?.parse().ok()
}

fn utf8(b: &[u8]) -> String {
    String::from_utf8_lossy(b).into_owned()
}

fn hex_upper(b: &[u8]) -> String {
    let mut s = String::with_capacity(b.len() * 2);
    for x in b {
        s.push_str(&format!("{x:02X}"));
    }
    s
}

fn f64s<const N: usize>(p: &[u8]) -> Option<[f64; N]> {
    let mut o = [0.0; N];
    for (i, v) in o.iter_mut().enumerate() {
        *v = rd_f64(p, i * 8)?;
    }
    Some(o)
}

fn vint(p: &[u8]) -> u64 {
    var_int(p, 0, p.len()).unwrap_or(0)
}

/// Per-face texture-mapping matrices from the `DC05` entity-info blob: DD05 -> B136 -> B236 -> 1027 -> 1127/1227 -> 1327
/// -> 1527 (72 bytes, 3x3 row-major).
fn uv_transforms(dc05: &[u8]) -> (Option<[f64; 9]>, Option<[f64; 9]>) {
    fn find<'a>(seq: &[(u16, &'a [u8])], tag: u16) -> Option<&'a [u8]> {
        seq.iter().find(|(t, _)| *t == tag).map(|(_, b)| *b)
    }
    let go = || -> Option<(Option<[f64; 9]>, Option<[f64; 9]>)> {
        let dd05 = find(&flat(dc05), 0xDD05)?;
        let b136 = find(&flat(dd05), 0xB136)?;
        let b236 = find(&flat(b136), 0xB236)?;
        let t1027 = find(&flat(b236), 0x1027)?;
        let sides = flat(t1027);
        let side = |tag: u16| -> Option<[f64; 9]> {
            let s = find(&sides, tag)?;
            let t1327 = find(&flat(s), 0x1327)?;
            let t1527 = find(&flat(t1327), 0x1527)?;
            if t1527.len() == 72 {
                f64s::<9>(t1527)
            } else {
                None
            }
        };
        Some((side(0x1127), side(0x1227)))
    };
    go().unwrap_or((None, None))
}

struct Ctx<'a, 'm> {
    t: &'a Tlv<'a>,
    m: &'m mut Model,
}

impl Ctx<'_, '_> {
    /// Port of `_extract_geometry_from_nodes` over the node ids `ids`.
    fn geometry(&mut self, ids: std::ops::Range<Id>, def: &mut Definition) {
        let t = self.t;
        for el in ids {
            let tag = t.node(el).tag;
            match tag {
                0xC409 => {
                    let id = t.entity_id(el);
                    let c = t.find(el, 0xC509).map(|c| t.payload(c));
                    if let (Some(id), Some(p)) = (id, c) {
                        if p.len() >= 24 {
                            if let Some(v) = f64s::<3>(p) {
                                def.vertices.insert(id, v);
                            }
                        }
                    }
                }
                0xB80B => {
                    if let Some(id) = t.entity_id(el) {
                        let v1 = t.find(el, 0xB90B).map(|n| vint(t.payload(n))).unwrap_or(NO_ID);
                        let v2 = t.find(el, 0xBA0B).map(|n| vint(t.payload(n))).unwrap_or(NO_ID);
                        let mut e = Edge { v1, v2, flags: 0, layer: None, curve: None };
                        if let Some(d007) = t.child(el, 0xD007) {
                            if let Some(d307) = t.child(d007, 0xD307) {
                                if let Some(&b) = t.payload(d307).first() {
                                    e.flags = b;
                                }
                            }
                            if let Some(d207) = t.child(d007, 0xD207) {
                                let p = t.payload(d207);
                                if !p.is_empty() {
                                    e.layer = Some(vint(p));
                                }
                            }
                        }
                        if let Some(bb) = t.find(el, 0xBB0B) {
                            let p = t.payload(bb);
                            if !p.is_empty() {
                                let c = vint(p);
                                if c != 0 {
                                    e.curve = Some(c);
                                }
                            }
                        }
                        def.edges.insert(id, e);
                    }
                }
                0xAC0D => {
                    if let Some(id) = t.entity_id(el) {
                        let mut normal = [0.0, 0.0, 1.0];
                        if let Some(n) = t.find(el, 0xAD0D) {
                            let p = t.payload(n);
                            if p.len() >= 24 {
                                if let Some(v) = f64s::<3>(p) {
                                    normal = v;
                                }
                            }
                        }
                        let mut loops: Vec<Vec<CoEdge>> = vec![];
                        if let Some(ae) = t.find(el, 0xAE0D) {
                            let mut lns = vec![];
                            t.find_all(t.kids(ae), 0x9411, &mut lns);
                            for ln in lns {
                                let mut cos = vec![];
                                t.find_all(t.kids(ln), 0xA00F, &mut cos);
                                let mut co_edges: Vec<CoEdge> = vec![];
                                for cn in cos {
                                    let p = t.payload(cn);
                                    let (mut edge_id, mut orient) = (None, None);
                                    let mut sp = 0usize;
                                    while sp + 6 < p.len() {
                                        let st = u16::from_be_bytes([p[sp], p[sp + 1]]);
                                        let ss = rd_u32(p, sp + 2).unwrap_or(0) as usize;
                                        if sp + 6 + ss <= p.len() {
                                            let val = var_int(p, sp + 6, ss).unwrap_or(0);
                                            if st == 0xA10F {
                                                edge_id = Some(val);
                                            } else if st == 0xA20F {
                                                orient = Some(val);
                                            }
                                        }
                                        sp += 6 + ss;
                                    }
                                    if let (Some(e), Some(o)) = (edge_id, orient) {
                                        co_edges.push((e, if o == 0 { 1 } else { -1 }));
                                    }
                                }
                                if !co_edges.is_empty() {
                                    loops.push(co_edges);
                                }
                            }
                        }
                        let (mut mat, mut layer, mut hidden) = (None, None, false);
                        let (mut uf, mut ub) = (None, None);
                        if let Some(d007) = t.child(el, 0xD007) {
                            if let Some(d107) = t.child(d007, 0xD107) {
                                mat = Some(vint(t.payload(d107)));
                            }
                            if let Some(d207) = t.child(d007, 0xD207) {
                                let p = t.payload(d207);
                                if !p.is_empty() {
                                    layer = Some(vint(p));
                                }
                            }
                            if let Some(dc05) = t.child(d007, 0xDC05) {
                                let (a, b) = uv_transforms(t.payload(dc05));
                                uf = a;
                                ub = b;
                            }
                            if let Some(d307) = t.child(d007, 0xD307) {
                                if let Some(&b) = t.payload(d307).first() {
                                    hidden = b & 1 != 0;
                                }
                            }
                        }
                        let mut back = None;
                        if let Some(af) = t.child(el, 0xAF0D) {
                            let p = t.payload(af);
                            if !p.is_empty() {
                                back = Some(vint(p));
                            }
                        }
                        def.faces.insert(id, Face { loops, normal, material_id: mat, back_material_id: back, layer, uv_front: uf, uv_back: ub, hidden });
                    }
                }
                0x6419 => {
                    let search = if t.node(el).nkids > 0 { t.kids(el) } else { el..el + 1 };
                    let guid = t.find_in(search.clone(), 0x6819).map(|n| t.payload(n)).filter(|p| p.len() == 16).map(hex_upper).unwrap_or_default();
                    let defref = t.find_in(search.clone(), 0x6719).map(|n| vint(t.payload(n)));
                    let name = t.find_in(search.clone(), 0x6519).map(|n| utf8(t.payload(n))).unwrap_or_default();
                    let matrix = t.find_in(search, 0x6619).map(|n| t.payload(n)).filter(|p| p.len() >= 104).and_then(f64s::<13>);
                    let (mut mat, mut layer, mut hidden) = (None, None, false);
                    if let Some(d007) = t.child(el, 0xD007) {
                        if let Some(d107) = t.child(d007, 0xD107) {
                            mat = Some(vint(t.payload(d107)));
                        }
                        if let Some(d207) = t.child(d007, 0xD207) {
                            let p = t.payload(d207);
                            if !p.is_empty() {
                                layer = Some(vint(p));
                            }
                        }
                        if let Some(d307) = t.child(d007, 0xD307) {
                            if let Some(&b) = t.payload(d307).first() {
                                hidden = b & 1 != 0;
                            }
                        }
                    }
                    def.instances.push(Instance { name, def: defref, guid, matrix, material_id: mat, hidden, layer });
                }
                _ => {
                    if t.node(el).nkids > 0 {
                        self.geometry(t.kids(el), def);
                    }
                }
            }
        }
    }

    fn collect_layers(&mut self, ids: std::ops::Range<Id>, hidden: &mut HashMap<String, bool>) {
        let t = self.t;
        for el in ids {
            if t.node(el).tag == 0x993A {
                for child in t.kids(el) {
                    if t.node(child).tag != 0x8C3C {
                        continue;
                    }
                    let dc05 = t.find(child, 0xDC05);
                    let name = t.find(child, 0x8D3C);
                    if let (Some(dc05), Some(name)) = (dc05, name) {
                        let p = t.payload(dc05);
                        let lid = if p.starts_with(&[0xDE, 0x05]) { rd_u32(p, 2).and_then(|l| var_int(p, 6, l as usize)) } else { var_int(p, 0, p.len()) };
                        let nm = utf8(t.payload(name));
                        if let Some(lid) = lid {
                            self.m.layer_id_to_name.insert(lid, nm.clone());
                        }
                        if let Some(h) = t.find(child, 0x8E3C) {
                            if let Some(&b) = t.payload(h).first() {
                                hidden.insert(nm, b == 1);
                            }
                        }
                    }
                }
            }
            self.collect_layers(t.kids(el), hidden);
        }
    }

    fn collect_material_ids(&mut self, ids: std::ops::Range<Id>) {
        let t = self.t;
        for el in ids {
            if t.node(el).tag == 0xC832 {
                let dc05 = t.find(el, 0xDC05);
                let name = t.find(el, 0xCC32);
                if let (Some(dc05), Some(name)) = (dc05, name) {
                    let p = t.payload(dc05);
                    let mid = if p.starts_with(&[0xDE, 0x05]) { rd_u32(p, 2).and_then(|l| var_int(p, 6, l as usize)) } else { var_int(p, 0, p.len()) };
                    if let Some(mid) = mid {
                        self.m.material_id_to_name.insert(mid, utf8(t.payload(name)));
                    }
                }
            }
            self.collect_material_ids(t.kids(el));
        }
    }

    fn collect_defs(&mut self, ids: std::ops::Range<Id>) {
        let t = self.t;
        for el in ids {
            if t.node(el).tag == 0x7C15 {
                let mut d = Definition::default();
                for child in t.kids(el) {
                    let p = t.payload(child);
                    match t.node(child).tag {
                        0x7D15 if p.len() == 16 => d.guid = hex_upper(p),
                        0x7E15 => d.name = utf8(p),
                        0x581B => {
                            let mut pos = 0;
                            while pos + 6 <= p.len() {
                                let st = u16::from_be_bytes([p[pos], p[pos + 1]]);
                                let ss = rd_u32(p, pos + 2).unwrap_or(0) as usize;
                                if pos + 6 + ss > p.len() {
                                    break;
                                }
                                if st == 0x5D1B && ss >= 1 {
                                    d.always_faces_camera = var_int(p, pos + 6, ss) == Some(1);
                                } else if st == 0x5E1B && ss >= 1 {
                                    d.shadows_face_sun = var_int(p, pos + 6, ss) == Some(1);
                                }
                                pos += 6 + ss;
                            }
                        }
                        0x8315 if !p.is_empty() => d.is_image = vint(p) == 2,
                        _ => {}
                    }
                }
                let id = t.entity_id(el).unwrap_or(NO_ID);
                d.id = Some(id);
                self.geometry(t.kids(el), &mut d);
                self.m.definitions.insert(id, d);
            }
            self.collect_defs(t.kids(el));
        }
    }
}

fn read_meta_units(meta: &[u8]) -> Option<String> {
    let mut pos = 0;
    while pos + 6 <= meta.len() {
        let tag = u16::from_be_bytes([meta[pos], meta[pos + 1]]);
        let size = rd_u32(meta, pos + 2)? as usize;
        if pos + 6 + size > meta.len() {
            break;
        }
        if tag == 0x6400 {
            return read_meta_units(&meta[pos + 6..pos + 6 + size]);
        }
        if tag == 0x6D00 {
            return Some(utf8(&meta[pos + 6..pos + 6 + size]));
        }
        pos += 6 + size;
    }
    None
}

fn is_image_ext(n: &str) -> bool {
    let l = n.to_ascii_lowercase();
    !l.ends_with(".xml")
}

fn read_entry(zip: &Zip, e: &ZipEntry, lim: &Limits, total: &mut usize) -> R<Vec<u8>> {
    if total.saturating_add(e.size) > lim.max_total {
        return Err(SkpError::new("limit.total", format!("inflated ZIP entries exceed {} bytes in total", lim.max_total)));
    }
    let d = zip.read(e, lim)?;
    *total += d.len();
    Ok(d)
}

fn parse_materials(zip: &Zip, lim: &Limits, total: &mut usize, tex_bytes: &mut usize, m: &mut Model) -> HashMap<String, (u8, u8, u8)> {
    let mut layer_colors: HashMap<String, (u8, u8, u8)> = HashMap::new();
    let mut layer_order: Vec<String> = vec![];
    for e in &zip.entries {
        if !(e.name.ends_with("material.xml") && e.name.starts_with("materials/")) {
            continue;
        }
        let Ok(xml_data) = read_entry(zip, e, lim, total) else {
            m.warn("zip.material", format!("cannot read {}", e.name), None);
            continue;
        };
        let text = String::from_utf8_lossy(&xml_data);
        let tags = xml::tags(&text, 10_000);
        let Some(mi) = tags.iter().position(|t| t.name == "material") else { continue };
        let me = &tags[mi];
        let name = me.attr("name").unwrap_or("unknown").to_string();
        let geti = |k: &str| -> Option<i64> { match me.attr(k) { None => Some(128), Some(v) => v.trim().parse::<i64>().ok() } };
        let (Some(r), Some(g), Some(b)) = (geti("colorRed"), geti("colorGreen"), geti("colorBlue")) else {
            m.warn("vff.material", format!("bad colour in {}", e.name), None);
            continue;
        };
        let c8 = |v: i64| v.clamp(0, 255) as u8;
        let opacity = if me.attr("useTrans") == Some("1") {
            match me.attr("trans") {
                None => 1.0,
                Some(v) => match v.trim().parse::<f64>() {
                    Ok(x) if x.is_finite() => (1.0 - x).clamp(0.0, 1.0),
                    _ => continue,
                },
            }
        } else {
            1.0
        };
        let folder = e.name.split('/').nth(1).unwrap_or("").to_string();
        let colorized = me.attr("type") == Some("2");
        let colorize_type = me.attr("colorizeType").and_then(|v| v.trim().parse::<i32>().ok()).unwrap_or(0);
        // texture
        let mut texture = None;
        if let Some(ti) = tags[mi + 1..].iter().position(|t| t.name == "texture").map(|x| x + mi + 1) {
            let te = &tags[ti];
            let mut filename = te.attr("textureFilename").unwrap_or("").to_string();
            let fnum = |k: &str| te.attr(k).and_then(|v| v.trim().parse::<f64>().ok()).filter(|x| x.is_finite()).unwrap_or(0.0);
            let (xs, ys) = (fnum("xScale"), fnum("yScale"));
            let xml_folder = e.name.rsplit_once('/').map_or("", |(f, _)| f).to_string();
            let mut data = None;
            let cand = if filename.is_empty() { None } else { Some(format!("{xml_folder}/{filename}")) };
            let try_read = |ent: &ZipEntry, total: &mut usize, tex_bytes: &mut usize| -> Option<Vec<u8>> {
                if tex_bytes.saturating_add(ent.size) > lim.max_texture_bytes {
                    return None;
                }
                let d = read_entry(zip, ent, lim, total).ok()?;
                *tex_bytes += d.len();
                Some(d)
            };
            if let Some(ent) = cand.as_deref().and_then(|c| zip.find(c)) {
                data = try_read(ent, total, tex_bytes);
            } else {
                for ent in &zip.entries {
                    if ent.name.starts_with(&format!("{xml_folder}/")) && ent.name != e.name && is_image_ext(&ent.name) {
                        data = try_read(ent, total, tex_bytes);
                        if filename.is_empty() {
                            filename = ent.name.rsplit('/').next().unwrap_or("").to_string();
                        }
                        break;
                    }
                }
            }
            if data.is_none() {
                let ip = tags[ti + 1..].iter().find(|t| t.name == "image").and_then(|t| t.attr("path")).unwrap_or("");
                let ip = ip.trim_start_matches(|c| c == '.' || c == '/');
                for c in [ip.to_string(), format!("{xml_folder}/{ip}")] {
                    if !c.is_empty() {
                        if let Some(ent) = zip.find(&c) {
                            data = try_read(ent, total, tex_bytes);
                            if filename.is_empty() {
                                filename = c.rsplit('/').next().unwrap_or("").to_string();
                            }
                            break;
                        }
                    }
                }
            }
            texture = Some(Texture { filename, x_scale: xs, y_scale: ys, data });
        }
        let mat = Material { name: name.clone(), rgba: [c8(r), c8(g), c8(b), 255], opacity, colorized, colorize_type, texture };
        let idx = m.materials.len();
        m.materials.push(mat);
        m.material_by_name.insert(name.clone(), idx);
        if !folder.is_empty() {
            m.material_by_folder.insert(folder, idx);
        }
        if let Some(rest) = name.strip_prefix("Layer_") {
            if !layer_colors.contains_key(rest) {
                layer_order.push(rest.to_string());
            }
            layer_colors.insert(rest.to_string(), (c8(r), c8(g), c8(b)));
        }
    }
    // layer order is kept through `Model::layers` by the caller
    m.layers = layer_order.iter().map(|n| { let c = layer_colors[n]; Layer { name: n.clone(), rgb: [c.0, c.1, c.2], hidden: false } }).collect();
    layer_colors
}

fn parse_styles(zip: &Zip, lim: &Limits, total: &mut usize, m: &mut Model) {
    for e in &zip.entries {
        if !(e.name.starts_with("styles/") && e.name.ends_with("style.xml")) {
            continue;
        }
        let Ok(d) = read_entry(zip, e, lim, total) else { continue };
        let text = String::from_utf8_lossy(&d);
        let tags = xml::tags(&text, 20_000);
        let Some(si) = tags.iter().position(|t| t.name == "style") else { continue };
        let mut front = None;
        let mut back = None;
        let mut i = si + 1;
        while i < tags.len() {
            if tags[i].name == "item" {
                let iid = tags[i].attr("id").unwrap_or("");
                if (iid == "2002" || iid == "2003") && i + 1 < tags.len() && tags[i + 1].name == "variant" {
                    if let Ok(v) = tags[i + 1].text.trim().parse::<i64>() {
                        let v = (v & 0xFFFF_FFFF) as u32;
                        let c = [(v & 255) as u8, ((v >> 8) & 255) as u8, ((v >> 16) & 255) as u8];
                        if iid == "2002" { front = Some(c) } else { back = Some(c) }
                    }
                }
            }
            i += 1;
        }
        m.styles.push(Style { name: tags[si].attr("name").unwrap_or("").to_string(), front_color: front, back_color: back });
    }
}

/// Parse a VFF file (header + ZIP).
pub fn parse(data: &[u8], lim: &Limits) -> R<Model> {
    if !data.starts_with(&MAGIC) {
        return Err(SkpError::at("container.magic", 0, "not a SketchUp file (bad header magic)"));
    }
    let mut m = Model::new(Container::Vff);
    m.version = header_version(data);
    m.version_major = version_major(&m.version);
    let pk = find_local_header(&data[..data.len().min(4096)], 0).ok_or_else(|| SkpError::new("container.no_zip", "no ZIP container found in the header"))?;
    let zip = Zip::open(&data[pk..], lim).map_err(|mut e| {
        e.offset = e.offset.map(|o| o + pk);
        e
    })?;
    let mut total = 0usize;
    let mut tex_bytes = 0usize;
    let layer_colors = parse_materials(&zip, lim, &mut total, &mut tex_bytes, &mut m);
    if let Some(e) = zip.find("meta/meta.dat") {
        if let Ok(d) = read_entry(&zip, e, lim, &mut total) {
            m.units = read_meta_units(&d);
        }
    }
    parse_styles(&zip, lim, &mut total, &mut m);

    let me = zip.find("model.dat").ok_or_else(|| SkpError::new("container.no_model", "ZIP archive has no model.dat entry"))?;
    let model_dat = read_entry(&zip, me, lim, &mut total)?;
    let mut layer_hidden: HashMap<String, bool> = HashMap::new();

    // top-level record headers: a lone F401 wrapper is opened, F901 definition lists are unwrapped (F901 -> 7017 -> 7117 -> 7C15...)
    let mut headers = flat_headers(&model_dat, 0, model_dat.len());
    if headers.len() == 1 && headers[0].0 == 0xF401 {
        let (_, o, s) = headers[0];
        headers = flat_headers(&model_dat, o + 6, o + 6 + s);
    }
    let mut expanded = vec![];
    for &(tag, off, size) in &headers {
        if tag == 0xF901 {
            let mut level = flat_headers(&model_dat, off + 6, off + 6 + size);
            let mut ok = true;
            for want in [0x7017u16, 0x7117] {
                match level.iter().find(|h| h.0 == want).copied() {
                    Some((_, o, s)) => level = flat_headers(&model_dat, o + 6, o + 6 + s),
                    None => {
                        ok = false;
                        break;
                    }
                }
            }
            if ok {
                let defs: Vec<_> = level.into_iter().filter(|h| h.0 == 0x7C15).collect();
                if !defs.is_empty() {
                    expanded.extend(defs);
                    continue;
                }
            }
        }
        expanded.push((tag, off, size));
    }

    let mut nodes_total = 0usize;
    let mut root = Definition::default();
    let mut truncated = 0usize;
    for &(_, off, size) in &expanded {
        let mut tlv = Tlv::new(&model_dat, lim.max_nodes.saturating_sub(nodes_total), lim.max_tlv_depth);
        let (first, count) = tlv.parse_range(off, off + 6 + size, 0).map_err(|mut e| {
            e.message = format!("model.dat: {}", e.message);
            e
        })?;
        nodes_total += tlv.nodes.len();
        truncated += tlv.truncated;
        if count == 0 {
            continue;
        }
        let el = first;
        let mut cx = Ctx { t: &tlv, m: &mut m };
        cx.collect_layers(el..el + 1, &mut layer_hidden);
        cx.collect_material_ids(el..el + 1);
        cx.collect_defs(el..el + 1);
        if tlv.node(el).tag == 0xF601 {
            let mut cx = Ctx { t: &tlv, m: &mut m };
            cx.geometry(tlv.kids(el), &mut root);
        }
    }
    if truncated > 0 {
        m.warn("tlv.truncated", format!("{truncated} TLV range(s) ended on a record that did not fit"), None);
    }
    root.id = None;
    m.root = root;

    // layers: id table entries without a Layer_ material are kept with the default colour; the default layer always exists
    if !m.layer_id_to_name.contains_key(&1) {
        m.layer_id_to_name.insert(1, "Layer0".into());
    }
    let mut names: Vec<String> = m.layers.iter().map(|l| l.name.clone()).collect();
    let mut ids: Vec<_> = m.layer_id_to_name.iter().map(|(k, v)| (*k, v.clone())).collect();
    ids.sort();
    for (_, n) in ids {
        if !names.contains(&n) {
            names.push(n.clone());
            m.layers.push(Layer { name: n, rgb: [136, 136, 136], hidden: false });
        }
    }
    if !layer_colors.contains_key("Layer0") && m.layer_by_name("Layer0").is_none() {
        m.layers.push(Layer { name: "Layer0".into(), rgb: [136, 136, 136], hidden: false });
    }
    for l in m.layers.iter_mut() {
        l.hidden = layer_hidden.get(&l.name).copied().unwrap_or(false);
    }
    Ok(m)
}
