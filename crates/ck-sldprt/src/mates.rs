//! `.SLDASM` mates: the mate list stream `Contents/Config-N-MatesList` of an assembly, and a geometric check of the mates it
//! can check.
//!
//! The stream is an object archive (class declarations `ff ff <schema u16> <len u16> <name>`, class references `0x8000 | index`,
//! strings as `ff fe ff <len u8> <UTF-16LE>`), the same family of encoding as the part feature streams. The layout below was
//! derived from our own 16 sample assemblies (14 mate lists, 700 mate records) and checked against the component tree; the
//! cadmpeg project's SolidWorks notes (`docs/formats/sldprt.md`, section 11.3, CC BY 4.0) were the pointer to the stream and the
//! `moMate*` class names. Nothing is copied from them.
//!
//! * `N` of `Config-N-MatesList` is the `swConfigurationId` of a configuration's assembly model in `COMPINSTANCETREE`
//!   (`AsmConfig::id`). A configuration without a stream has no mates of its own. In OLE2 files the stream may be `__ZLB` wrapped.
//! * One record per mate (and per mate folder). A record starts with its class: a declaration (the first record of each class)
//!   or a `u16` class reference (`0x8000 | index`, the same value for every record of the class), then a `u16` `0x80xx` that is
//!   constant in the stream, then the mate name as a string, then a fixed 19 byte header that identifies the record:
//!   `00 | flag | 00 x5 | b | id | 00 x6 | ff fe ff` where `b` is `0x40` or `0xc0`, `id` is the 24-bit feature id (the `u32` at `b`
//!   shifted right by 8; it rises through the stream) and `flag` is `00` or `01` (meaning not decoded). The byte at header offset
//!   32 (counted from the end of the name) is `01` for a mate that is suppressed in this configuration: the same mate differs in
//!   exactly this byte between two configuration lists of one file (in the sample with ten configurations, a "free" configuration
//!   flags 34 mates, a "left" one 7).
//! * The class (declared, or the reference value shared by all records of one class) gives the type: `moMateCoincident`,
//!   `moMateConcentric`, `moMateParallel`, `moMatePerpendicular`, `moMateTangent`, `moMateSymmetric`, `moMateCoordinate`,
//!   `moMateDistanceDim_c` (distance), `moMatePlanarAngleDim_c` (angle). A reference value is mapped to a type by the names of
//!   the records that share it (`Coincident12`, `Distance3`, ...); a renamed mate takes the type of its class.
//!   `moMisalignedMateFolder_c` / `moFtrFolder_c` records (`Misaligned2`, `Misaligned2___EndTag___`) are folders, not mates.
//! * Operands are read from the strings of the record: a reference string `moXxxSurfIdRep_c,<feature>,<n>,...` (a face of the
//!   part, named by the feature that made it; the part's geometry is needed to resolve it) is followed by the component
//!   `<instance>@<assembly>`; a named reference (`Front Plane@<instance>@<assembly>`) is preceded by its component. The
//!   assembly part of a component string is the path from the top assembly (`Top/Sub-1`): the component's path in the tree is
//!   `Sub-1/<instance>`. A component that was already written earlier in the stream is stored as an object back-reference, not
//!   as a string: that operand is left unresolved (about one operand in six in the samples).
//! * Not decoded: the alignment (aligned / anti-aligned) and lock flags, the distance and angle values (the dimension objects
//!   follow the operands, but no field was confirmed against a mate whose geometry could be recomputed), mate limits, the mate
//!   entity geometry (a face reference carries local geometry and the component's transform, but its layout varies by face kind).
//!
//! [`check_mates`] recomputes the conditions it can from the tree's transforms: coincident, parallel and perpendicular between
//! two principal planes (`Front Plane`, `Top Plane`, `Right Plane`) of placed components. Every other mate is `unchecked` with a
//! reason; a mate flagged suppressed is never checked.
//!
//! Never panics on malformed input; every read is bounds checked and the counts are limited ([`MAX_MATES`], [`MAX_OPERANDS`]).

use crate::asm::{AsmComponent, AsmTree};
use crate::container::Container;
use crate::error::Limits;
use std::collections::HashMap;

/// Most mate records kept from one list; the rest are counted in [`MateList::dropped`].
pub const MAX_MATES: usize = 20_000;
/// Most operands kept per mate.
pub const MAX_OPERANDS: usize = 8;
/// Longest reference string kept (characters).
pub const MAX_REF_CHARS: usize = 200;

/// One side of a mate.
#[derive(Debug, Clone, PartialEq)]
pub struct MateOperand {
    /// Path of the component in the assembly tree (`Sub-1/Part-2`), `None` when the operand names no component (the component was
    /// written earlier in the stream and is only referred to).
    pub component: Option<String>,
    /// The reference: a plane name (`Front Plane`), another named reference, or a face reference (`moFromSktEntSurfIdRep_c,163,4`).
    pub reference: Option<String>,
    /// `plane` (a named plane), `face` (a face reference), `named` (another named reference, e.g. an origin) or `component`.
    pub kind: &'static str,
}

#[derive(Debug, Clone, PartialEq)]
pub struct Mate {
    pub name: String,
    /// `coincident`, `concentric`, `parallel`, `perpendicular`, `tangent`, `symmetric`, `coordinate`, `distance`, `angle` or `unknown`.
    pub kind: &'static str,
    /// The class declared in the stream (only the first record of a class declares it).
    pub class: Option<String>,
    /// Feature id.
    pub id: u32,
    /// Suppressed in this configuration.
    pub suppressed: bool,
    pub operands: Vec<MateOperand>,
}

/// The mates of one configuration.
#[derive(Debug, Clone, PartialEq)]
pub struct MateList {
    /// `N` of `Config-N-MatesList` (the configuration id).
    pub config_id: u32,
    pub mates: Vec<Mate>,
    /// Mate and mate-folder records past [`MAX_MATES`].
    pub dropped: usize,
    /// Folder records (`Misaligned..`), not mates.
    pub folders: usize,
    pub notes: Vec<String>,
}

/// `Config-12-MatesList` (also `Contents/...`, `...__ZLB`) -> 12.
fn config_of(stream: &str) -> Option<u32> {
    let s = stream.strip_suffix("__ZLB").unwrap_or(stream);
    let s = s.rsplit('/').next()?;
    s.strip_prefix("Config-")?.strip_suffix("-MatesList")?.parse().ok()
}

/// The mate lists of an assembly container, in stream order. A list that is empty or has no mate record is kept (empty).
pub fn read_container(c: &Container, limits: &Limits) -> Vec<MateList> {
    let mut out = vec![];
    for s in &c.streams {
        let Some(id) = config_of(&s.name) else { continue };
        let data = if s.name.ends_with("__ZLB") { c.data(s.name.strip_suffix("__ZLB").unwrap_or(&s.name), limits) } else { Some(s.data.clone()) };
        let Some(d) = data else { continue };
        out.push(parse_list(&d, id));
    }
    out.sort_by_key(|l| l.config_id);
    out.dedup_by_key(|l| l.config_id);
    out
}

/// The list of configuration `id` (the `AsmConfig::id` of the configuration read).
pub fn list_for(lists: &[MateList], id: Option<u32>) -> Option<&MateList> {
    lists.iter().find(|l| Some(l.config_id) == id)
}

struct Rec {
    /// Offset of the name string's `ff fe ff`.
    at: usize,
    /// End of the name string.
    end: usize,
    name: String,
    flags_b: u8,
    id: u32,
    suppressed: bool,
    class: Option<String>,
    tag: Option<u16>,
}

/// The CString at `at` (`ff fe ff <len>`): its text and end offset.
fn cstring(d: &[u8], at: usize) -> Option<(String, usize)> {
    if d.get(at..at + 3)? != [0xff, 0xfe, 0xff] {
        return None;
    }
    let n = *d.get(at + 3)? as usize;
    if n == 0xff {
        return None;
    }
    let end = at + 4 + 2 * n;
    let b = d.get(at + 4..end)?;
    let u: Vec<u16> = b.chunks_exact(2).map(|p| u16::from_le_bytes([p[0], p[1]])).collect();
    Some((String::from_utf16_lossy(&u), end))
}

/// The class declaration that ends at `p` (`ff ff <schema> <len u16> <name>`), when there is one.
fn declared_class(d: &[u8], p: usize) -> Option<String> {
    for len in 4..40usize {
        let q = p.checked_sub(len)?;
        if q < 6 {
            return None;
        }
        if d.get(q - 6..q - 4)? == [0xff, 0xff] && d.get(q - 2..q)? == (len as u16).to_le_bytes() {
            let name = d.get(q..p)?;
            if name.iter().all(|b| b.is_ascii_alphanumeric() || *b == b'_') {
                return Some(String::from_utf8_lossy(name).into_owned());
            }
        }
    }
    None
}

fn records(d: &[u8]) -> Vec<Rec> {
    let mut out = vec![];
    let mut i = 4usize;
    while i + 4 <= d.len() {
        if d[i] == 0xff && d[i + 1] == 0xfe && d[i + 2] == 0xff && d[i - 1] >= 0x80 {
            if let Some((name, end)) = cstring(d, i) {
                if let Some(h) = d.get(end..end + 19) {
                    let ok = h[0] == 0 && h[1] <= 1 && h[2..7].iter().all(|&b| b == 0) && (h[7] == 0x40 || h[7] == 0xc0) && h[10..16].iter().all(|&b| b == 0) && h[16..19] == [0xff, 0xfe, 0xff];
                    if ok {
                        let p = i - 2;
                        let class = declared_class(d, p);
                        let tag = if class.is_none() && p >= 2 { Some(u16::from_le_bytes([d[p - 2], d[p - 1]])) } else { None };
                        let id = u32::from_le_bytes([h[7], h[8], h[9], h[10]]) >> 8;
                        out.push(Rec { at: i, end, name, flags_b: h[7], id, suppressed: d.get(end + 32) == Some(&1), class, tag });
                        i = end + 19;
                        continue;
                    }
                }
            }
        }
        i += 1;
    }
    out
}

/// Type from a mate class name (`moMateDistanceDim_c` -> distance).
fn kind_of_class(class: &str) -> Option<&'static str> {
    Some(match class {
        "moMateCoincident" => "coincident",
        "moMateConcentric" => "concentric",
        "moMateParallel" => "parallel",
        "moMatePerpendicular" => "perpendicular",
        "moMateTangent" => "tangent",
        "moMateSymmetric" => "symmetric",
        "moMateCoordinate" => "coordinate",
        "moMateDistanceDim_c" => "distance",
        "moMatePlanarAngleDim_c" => "angle",
        _ => return None,
    })
}

fn is_folder_class(class: &str) -> bool {
    class == "moMisalignedMateFolder_c" || class == "moFtrFolder_c"
}

/// Type from the leading letters of a default mate name (`Coincident12`, `Misaligned3`, `Angle2`).
fn kind_of_name(name: &str) -> Option<&'static str> {
    let l = name.trim_end_matches(|c: char| c.is_ascii_digit());
    if l.len() == name.len() && !name.contains("EndTag") {
        return None;
    }
    Some(match l {
        "Coincident" => "coincident",
        "Concentric" => "concentric",
        "Parallel" => "parallel",
        "Perpendicular" => "perpendicular",
        "Tangent" => "tangent",
        "Symmetric" => "symmetric",
        "Coordinate" => "coordinate",
        "Distance" => "distance",
        "Angle" => "angle",
        "Misaligned" => "folder",
        _ if name.contains("___EndTag___") => "folder",
        _ => return None,
    })
}

/// Parse one mate list stream.
pub fn parse_list(d: &[u8], config_id: u32) -> MateList {
    let recs = records(d);
    // class reference value -> type, by the default names of the records that share it
    let mut votes: HashMap<u16, HashMap<&'static str, usize>> = HashMap::new();
    for r in &recs {
        if let (Some(t), Some(k)) = (r.tag, kind_of_name(&r.name)) {
            *votes.entry(t).or_default().entry(k).or_default() += 1;
        }
    }
    let by_tag: HashMap<u16, &'static str> = votes.into_iter().filter_map(|(t, v)| v.into_iter().max_by_key(|(k, n)| (*n, *k)).map(|(k, _)| (t, k))).collect();
    let mut mates = vec![];
    let (mut dropped, mut folders, mut unknown) = (0usize, 0usize, 0usize);
    let mut notes = vec![];
    for (k, r) in recs.iter().enumerate() {
        let kind: &'static str = match (&r.class, r.tag) {
            (Some(c), _) if is_folder_class(c) => "folder",
            (Some(c), _) => kind_of_class(c).unwrap_or("unknown"),
            (None, Some(t)) => by_tag.get(&t).copied().or_else(|| kind_of_name(&r.name)).unwrap_or("unknown"),
            _ => "unknown",
        };
        if kind == "folder" || r.name.contains("___EndTag___") {
            folders += 1;
            continue;
        }
        if kind == "unknown" {
            unknown += 1;
        }
        if mates.len() >= MAX_MATES {
            dropped += 1;
            continue;
        }
        let span_end = recs.get(k + 1).map_or(d.len(), |n| n.at.saturating_sub(4)).max(r.end);
        let operands = operands_of(d, r.end + 19, span_end);
        let _ = r.flags_b;
        mates.push(Mate { name: r.name.clone(), kind, class: r.class.clone(), id: r.id, suppressed: r.suppressed, operands });
    }
    if dropped > 0 {
        notes.push(format!("the list has more than {MAX_MATES} mates: {dropped} were not read"));
    }
    if unknown > 0 {
        notes.push(format!("{unknown} mate(s) have a class this reader does not know (type `unknown`)"));
    }
    MateList { config_id, mates, dropped, folders, notes }
}

fn clean_ref(s: &str) -> String {
    let one = s.lines().next().unwrap_or("").trim();
    one.chars().take(MAX_REF_CHARS).collect()
}

/// `inst@Top/Sub-1` -> `Sub-1/inst`; `inst@Top` -> `inst`.
fn component_path(inst_and_asm: &str) -> Option<String> {
    let (inst, asm) = inst_and_asm.rsplit_once('@')?;
    if inst.is_empty() {
        return None;
    }
    let mut parts: Vec<&str> = asm.split('/').skip(1).filter(|p| !p.is_empty()).collect();
    parts.push(inst);
    Some(parts.join("/"))
}

fn operands_of(d: &[u8], from: usize, to: usize) -> Vec<MateOperand> {
    let mut out: Vec<MateOperand> = vec![];
    let mut pending_face: Option<String> = None;
    let mut pending_comp: Option<String> = None;
    let mut i = from;
    let to = to.min(d.len());
    let flush_comp = |out: &mut Vec<MateOperand>, c: &mut Option<String>| {
        if let Some(c) = c.take() {
            if out.len() < MAX_OPERANDS {
                out.push(MateOperand { component: Some(c), reference: None, kind: "component" });
            }
        }
    };
    while i + 4 <= to {
        if d[i] == 0xff && d[i + 1] == 0xfe && d[i + 2] == 0xff {
            if let Some((s, end)) = cstring(d, i) {
                i = end;
                if s.is_empty() || s.contains(":\\") || s.contains('\\') {
                    continue;
                }
                if s.contains("SurfIdRep") || s.contains("IdRep_c") {
                    // a face reference; a component waiting for its reference is complete without one
                    flush_comp(&mut out, &mut pending_comp);
                    if let Some(prev) = pending_face.take() {
                        if out.len() < MAX_OPERANDS {
                            out.push(MateOperand { component: None, reference: Some(prev), kind: "face" });
                        }
                    }
                    pending_face = Some(clean_ref(&s));
                    continue;
                }
                let ats = s.matches('@').count();
                if ats >= 2 {
                    // `<reference>@<instance>@<assembly>`
                    let (name, rest) = s.split_once('@').unwrap_or((&s, ""));
                    let comp = component_path(rest);
                    // the component string written just before this one belongs to the same operand
                    if pending_comp.as_deref() == comp.as_deref() {
                        pending_comp = None;
                    } else {
                        flush_comp(&mut out, &mut pending_comp);
                    }
                    let kind = if name.ends_with("Plane") { "plane" } else { "named" };
                    if out.len() < MAX_OPERANDS {
                        out.push(MateOperand { component: comp, reference: Some(clean_ref(name)), kind });
                    }
                } else if ats == 1 {
                    let comp = component_path(&s);
                    if let (Some(f), Some(c)) = (pending_face.take(), comp.clone()) {
                        if out.len() < MAX_OPERANDS {
                            out.push(MateOperand { component: Some(c), reference: Some(f), kind: "face" });
                        }
                    } else {
                        flush_comp(&mut out, &mut pending_comp);
                        pending_comp = comp;
                    }
                }
                continue;
            }
        }
        i += 1;
    }
    flush_comp(&mut out, &mut pending_comp);
    if let Some(f) = pending_face.take() {
        if out.len() < MAX_OPERANDS {
            out.push(MateOperand { component: None, reference: Some(f), kind: "face" });
        }
    }
    out
}

// ------------------------------------------------------------------ geometric check

/// Result of checking one mate against the tree's transforms.
#[derive(Debug, Clone, PartialEq)]
pub struct MateCheck {
    /// `holds`, `violated` or `unchecked`.
    pub status: &'static str,
    /// Why a mate is unchecked: `suppressed`, `operand-unresolved`, `component-missing`, `component-suppressed`, `face-operand`,
    /// `value-not-decoded`, `type-not-checked`.
    pub reason: Option<&'static str>,
    /// For a checked mate: the plane distance (coincident, mm) or the angle off the required angle (parallel, perpendicular,
    /// degrees).
    pub residual: Option<f64>,
}

/// Tolerances: distance (mm) and angle (radians).
pub const TOL_DIST: f64 = 1e-3;
pub const TOL_ANGLE: f64 = 1e-6;

fn mul(a: &[f64; 16], b: &[f64; 16]) -> [f64; 16] {
    let mut m = [0.0; 16];
    for r in 0..4 {
        for c in 0..4 {
            m[r * 4 + c] = (0..4).map(|k| a[r * 4 + k] * b[k * 4 + c]).sum();
        }
    }
    m
}

/// World transform (column vectors, mm) and suppressed flag of every component, by tree path (`Sub-1/Part-2`).
pub fn component_worlds(tree: &AsmTree) -> HashMap<String, ([f64; 16], bool)> {
    fn go(c: &[AsmComponent], parent: &[f64; 16], path: &str, sup: bool, out: &mut HashMap<String, ([f64; 16], bool)>) {
        for x in c {
            let p = if path.is_empty() { x.name.clone() } else { format!("{path}/{}", x.name) };
            let w = mul(parent, &x.transform);
            let s = sup || x.suppressed;
            out.insert(p.clone(), (w, s));
            go(&x.children, &w, &p, s, out);
        }
    }
    let id = [1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0];
    let mut out = HashMap::new();
    go(&tree.components, &id, "", false, &mut out);
    out
}

/// Principal plane of a part in its own frame: (normal) of `Front` (z), `Top` (y), `Right` (x); every plane passes the origin.
fn principal_normal(name: &str) -> Option<[f64; 3]> {
    match name {
        "Front Plane" => Some([0.0, 0.0, 1.0]),
        "Top Plane" => Some([0.0, 1.0, 0.0]),
        "Right Plane" => Some([1.0, 0.0, 0.0]),
        _ => None,
    }
}

fn plane_world(w: &[f64; 16], n: [f64; 3]) -> ([f64; 3], [f64; 3]) {
    let o = [w[3], w[7], w[11]];
    let v = [w[0] * n[0] + w[1] * n[1] + w[2] * n[2], w[4] * n[0] + w[5] * n[1] + w[6] * n[2], w[8] * n[0] + w[9] * n[1] + w[10] * n[2]];
    let l = (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt();
    if l > 0.0 {
        (o, [v[0] / l, v[1] / l, v[2] / l])
    } else {
        (o, v)
    }
}

fn unchecked(reason: &'static str) -> MateCheck {
    MateCheck { status: "unchecked", reason: Some(reason), residual: None }
}

/// Check every mate of `list` against `tree` (same order as `list.mates`).
pub fn check_mates(tree: &AsmTree, list: &MateList) -> Vec<MateCheck> {
    let worlds = component_worlds(tree);
    list.mates.iter().map(|m| check_one(&worlds, m)).collect()
}

fn check_one(worlds: &HashMap<String, ([f64; 16], bool)>, m: &Mate) -> MateCheck {
    if m.suppressed {
        return unchecked("suppressed");
    }
    if m.operands.len() < 2 || m.operands.iter().take(2).any(|o| o.component.is_none()) {
        return unchecked("operand-unresolved");
    }
    let (a, b) = (&m.operands[0], &m.operands[1]);
    for o in [a, b] {
        match worlds.get(o.component.as_deref().unwrap_or("")) {
            None => return unchecked("component-missing"),
            Some((_, true)) => return unchecked("component-suppressed"),
            _ => {}
        }
    }
    let plane = |o: &MateOperand| o.reference.as_deref().and_then(principal_normal).filter(|_| o.kind == "plane");
    let (Some(na), Some(nb)) = (plane(a), plane(b)) else { return unchecked("face-operand") };
    if !matches!(m.kind, "coincident" | "parallel" | "perpendicular") {
        return unchecked(if matches!(m.kind, "distance" | "angle") { "value-not-decoded" } else { "type-not-checked" });
    }
    let (wa, wb) = (&worlds[a.component.as_deref().unwrap_or("")].0, &worlds[b.component.as_deref().unwrap_or("")].0);
    let ((oa, va), (ob, vb)) = (plane_world(wa, na), plane_world(wb, nb));
    let dot: f64 = va.iter().zip(vb.iter()).map(|(x, y)| x * y).sum();
    let ang_par = dot.abs().min(1.0).acos(); // 0 when parallel (either alignment)
    let ang_perp = (std::f64::consts::FRAC_PI_2 - dot.abs().min(1.0).acos()).abs();
    match m.kind {
        "coincident" => {
            let dist: f64 = (0..3).map(|i| (ob[i] - oa[i]) * va[i]).sum::<f64>().abs();
            let ok = ang_par < TOL_ANGLE && dist < TOL_DIST;
            MateCheck { status: if ok { "holds" } else { "violated" }, reason: None, residual: Some(dist) }
        }
        "parallel" => MateCheck { status: if ang_par < TOL_ANGLE { "holds" } else { "violated" }, reason: None, residual: Some(ang_par.to_degrees()) },
        _ => MateCheck { status: if ang_perp < TOL_ANGLE { "holds" } else { "violated" }, reason: None, residual: Some(ang_perp.to_degrees()) },
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cstr(s: &str) -> Vec<u8> {
        let mut v = vec![0xff, 0xfe, 0xff, s.chars().count() as u8];
        for u in s.encode_utf16() {
            v.extend_from_slice(&u.to_le_bytes());
        }
        v
    }

    /// One record: class (declared or referenced), the constant, the name, the 19 byte header, then operand strings.
    fn rec(out: &mut Vec<u8>, class: Result<&str, u16>, name: &str, id: u32, flag: u8, suppressed: bool, operands: &[&str]) {
        match class {
            Ok(c) => {
                out.extend_from_slice(&[0xff, 0xff, 1, 0]);
                out.extend_from_slice(&(c.len() as u16).to_le_bytes());
                out.extend_from_slice(c.as_bytes());
            }
            Err(t) => out.extend_from_slice(&(0x8000u16 | t).to_le_bytes()),
        }
        out.extend_from_slice(&[0x20, 0x80]);
        out.extend(cstr(name));
        let mut h = vec![0u8, flag, 0, 0, 0, 0, 0];
        h.extend_from_slice(&((id << 8) | 0xc0).to_le_bytes());
        h.extend_from_slice(&[0; 5]);
        h.extend_from_slice(&[0xff, 0xfe, 0xff]);
        // 13 more bytes up to offset 32, then the suppressed flag
        h.extend_from_slice(&[0; 13]);
        h.push(suppressed as u8);
        h.extend_from_slice(&[0; 8]);
        out.extend(h);
        for o in operands {
            out.extend_from_slice(&[0, 0, 0, 0]);
            out.extend(cstr(o));
        }
        out.extend_from_slice(&[0; 6]);
    }

    fn stream() -> Vec<u8> {
        let mut d = vec![0x58, 1, 0, 0, 3, 0];
        let asm = "TOP";
        let p = |s: &str| format!("{s}@{asm}");
        rec(&mut d, Ok("moMateCoincident"), "Coincident1", 0x11b, 1, false, &[&p("A-1"), &format!("Front Plane@{}", p("A-1")), &p("B-1"), &format!("Front Plane@{}", p("B-1"))]);
        rec(&mut d, Ok("moMateParallel"), "Parallel1", 0x11c, 1, false, &[&p("A-1"), &format!("Top Plane@{}", p("A-1")), &p("B-1"), &format!("Right Plane@{}", p("B-1"))]);
        rec(&mut d, Ok("moMateConcentric"), "Concentric1", 0x11d, 1, true, &["moWzdHoleSurfIdRep_c,169,2,0, \n", &p("A-1"), "moFromSktEntSurfIdRep_c,100,2, \n", "C-2@TOP/Sub-1"]);
        // a renamed mate of the Coincident class, by reference; and a distance mate whose class is declared
        rec(&mut d, Err(0x158), "Coincident2", 0x120, 1, false, &[&p("A-1")]);
        rec(&mut d, Err(0x158), "Origin lock", 0x121, 1, false, &[&p("A-1"), &format!("Front Plane@{}", p("A-1")), &p("B-1"), &format!("Front Plane@{}", p("B-1"))]);
        rec(&mut d, Ok("moMateDistanceDim_c"), "Distance1", 0x122, 1, false, &[&p("A-1"), &format!("Front Plane@{}", p("A-1")), &p("B-1"), &format!("Front Plane@{}", p("B-1"))]);
        rec(&mut d, Ok("moMisalignedMateFolder_c"), "Misaligned1", 0x123, 0, false, &[]);
        rec(&mut d, Err(0x159), "Misaligned1___EndTag___", 0x124, 0, false, &[]);
        d
    }

    #[test]
    fn decodes_records_types_operands_and_flags() {
        let l = parse_list(&stream(), 7);
        assert_eq!(l.config_id, 7);
        assert_eq!(l.folders, 2);
        let k: Vec<(&str, &str, u32, bool)> = l.mates.iter().map(|m| (m.name.as_str(), m.kind, m.id, m.suppressed)).collect();
        assert_eq!(
            k,
            vec![
                ("Coincident1", "coincident", 0x11b, false),
                ("Parallel1", "parallel", 0x11c, false),
                ("Concentric1", "concentric", 0x11d, true),
                ("Coincident2", "coincident", 0x120, false), // by its default name (no other record shares the reference)
                ("Origin lock", "coincident", 0x121, false), // renamed: the class reference it shares with Coincident2
                ("Distance1", "distance", 0x122, false),
            ]
        );
        let m = &l.mates[0];
        assert_eq!(m.operands.len(), 2);
        assert_eq!(m.operands[0], MateOperand { component: Some("A-1".into()), reference: Some("Front Plane".into()), kind: "plane" });
        let c = &l.mates[2].operands;
        assert_eq!(c[0], MateOperand { component: Some("A-1".into()), reference: Some("moWzdHoleSurfIdRep_c,169,2,0,".into()), kind: "face" });
        // the assembly part of the string is the path from the top: C-2 lives in Sub-1
        assert_eq!(c[1].component.as_deref(), Some("Sub-1/C-2"));
        assert_eq!(c[1].kind, "face");
        assert!(l.notes.is_empty());
    }

    #[test]
    fn a_class_reference_takes_the_type_of_the_records_that_share_it() {
        let mut d = stream();
        // two more records of the reference 0x158 with default names: it is the coincident class
        rec(&mut d, Err(0x158), "Coincident7", 0x130, 1, false, &[]);
        rec(&mut d, Err(0x158), "Coincident8", 0x131, 1, false, &[]);
        let l = parse_list(&d, 0);
        let by: HashMap<&str, &str> = l.mates.iter().map(|m| (m.name.as_str(), m.kind)).collect();
        assert_eq!(by["Coincident2"], "coincident");
        assert_eq!(by["Origin lock"], "coincident"); // renamed, same class
        assert_eq!(by["Coincident8"], "coincident");
    }

    fn comp(name: &str, tf: [f64; 16], kids: Vec<AsmComponent>) -> AsmComponent {
        AsmComponent { name: name.into(), model: name.into(), part: None, kind: None, configuration: String::new(), transform: tf, suppressed: false, hidden: false, virtual_component: false, exclude_from_bom: false, flexible: false, bbox: None, children: kids }
    }

    fn tree(comps: Vec<AsmComponent>) -> AsmTree {
        let count = comps.len();
        AsmTree { name: "TOP".into(), configurations: vec![], configuration: String::new(), components: comps, count, notes: vec![] }
    }

    const ID: [f64; 16] = [1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0];

    #[test]
    fn plane_mates_are_checked_against_the_transforms() {
        let l = parse_list(&stream(), 0);
        // B moved 5 mm along z: the Front planes (normal z) are 5 mm apart, A and B Top/Right differ
        let mut tb = ID;
        tb[11] = 5.0;
        let t = tree(vec![comp("A-1", ID, vec![]), comp("B-1", tb, vec![])]);
        let c = check_mates(&t, &l);
        assert_eq!(c[0].status, "violated");
        assert!((c[0].residual.unwrap() - 5.0).abs() < 1e-9);
        // Top (normal y) against Right (normal x): not parallel
        assert_eq!((c[1].status, c[1].residual.map(|r| r.round())), ("violated", Some(90.0)));
        assert_eq!((c[2].status, c[2].reason), ("unchecked", Some("suppressed")));
        // B back at the origin: the coincident mate holds
        let t = tree(vec![comp("A-1", ID, vec![]), comp("B-1", ID, vec![])]);
        let c = check_mates(&t, &l);
        assert_eq!(c[0].status, "holds");
        assert_eq!((c[5].status, c[5].reason), ("unchecked", Some("value-not-decoded")));
        // a flipped B (rotated 180 degrees about x): Front normals opposed, still coincident (planes, either alignment)
        let mut tf = ID;
        tf[5] = -1.0;
        tf[10] = -1.0;
        let t = tree(vec![comp("A-1", ID, vec![]), comp("B-1", tf, vec![])]);
        assert_eq!(check_mates(&t, &l)[0].status, "holds");
        // a component missing from the tree
        let t = tree(vec![comp("A-1", ID, vec![])]);
        assert_eq!(check_mates(&t, &l)[0].reason, Some("component-missing"));
    }

    #[test]
    fn nested_components_use_the_assembly_path_and_world_transform() {
        let mut d = vec![0u8; 6];
        rec(&mut d, Ok("moMateCoincident"), "Coincident1", 1, 1, false, &["P-1@TOP/Sub-1", "Right Plane@P-1@TOP/Sub-1", "Q-1@TOP", "Right Plane@Q-1@TOP"]);
        let l = parse_list(&d, 0);
        assert_eq!(l.mates[0].operands[0].component.as_deref(), Some("Sub-1/P-1"));
        let mut ts = ID;
        ts[3] = 10.0; // Sub-1 at x = 10
        let mut tq = ID;
        tq[3] = 10.0;
        let t = tree(vec![comp("Sub-1", ts, vec![comp("P-1", ID, vec![])]), comp("Q-1", tq, vec![])]);
        assert_eq!(check_mates(&t, &l)[0].status, "holds");
    }

    #[test]
    fn malformed_streams_give_nothing_not_panics() {
        let s = stream();
        for cut in 0..s.len() {
            let _ = parse_list(&s[..cut], 0);
        }
        let mut z = s.clone();
        for i in (0..z.len()).step_by(7) {
            z[i] ^= 0xa5;
            let _ = parse_list(&z, 0);
        }
        assert!(parse_list(&[], 0).mates.is_empty());
        assert!(parse_list(&[0xff; 64], 0).mates.is_empty());
        assert_eq!(config_of("Contents/Config-12-MatesList"), Some(12));
        assert_eq!(config_of("Config-3-MatesList__ZLB"), Some(3));
        assert_eq!(config_of("Config-3-Partition"), None);
    }
}
