//! Font reader for lettering, written from the OpenType specification (no third-party code).
//!
//! Reads: `head`, `maxp`, `hhea`, `hmtx`, `cmap` (formats 4 and 12), `name` (family only), `OS/2`
//! (cap height fallback) and the outlines either from `loca` + `glyf` (TrueType, simple and composite
//! glyphs) or from `CFF ` (Type 2 charstrings, cubic curves approximated by quadratics within
//! 0.05 % of the em). Collections (`ttcf`) read their first font; variable fonts read their default
//! instance. Pair kerning (`kern` format 0, GPOS PairPos formats 1 and 2 for the `kern` feature)
//! and standard ligatures (GSUB `liga`, LigatureSubst) are read and used only when the text entity
//! opts in (`kern: true` / `liga: true`); other lookups, hinting and colour tables are ignored.
//! `Font::subset` writes a plain TrueType font with just the glyphs of a string (layout tables are
//! dropped, so subsets kern and ligate as no-ops).

use std::sync::Arc;

/// One outline point of a glyph, in font units.
#[derive(Clone, Copy, Debug)]
pub struct Pt {
    pub x: f64,
    pub y: f64,
    pub on: bool,
}

/// The contours of a glyph (each a closed list of on/off-curve points).
pub type Contours = Vec<Vec<Pt>>;

/// Most points one glyph may expand to (composites of composites); hostile files stop here.
const MAX_GLYPH_POINTS: usize = 40_000;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum OutlineKind {
    Glyf,
    Cff,
}

/// What the file contains (reported by `loadFont`; kerning and ligatures are used when the
/// text entity opts in with `kern: true` / `liga: true`).
#[derive(Clone, Debug, Default)]
pub struct Notes {
    pub variable: bool,
    pub kerning: bool,
    pub substitution: bool,
    pub collection: bool,
    pub cap_height_estimated: bool,
}

/// GPOS/GSUB class definition (formats 1 and 2); glyphs not listed are class 0.
#[derive(Clone, Debug, Default)]
pub struct ClassDef {
    ranges: Vec<(u16, u16, u16)>,
}

impl ClassDef {
    fn parse(d: &[u8], o: usize) -> ClassDef {
        let mut ranges = vec![];
        match u16_at(d, o) {
            Some(1) => {
                if let (Some(start), Some(count)) = (u16_at(d, o + 2), u16_at(d, o + 4)) {
                    for i in 0..count {
                        if let Some(c) = u16_at(d, o + 6 + i as usize * 2) {
                            if c != 0 {
                                ranges.push((start + i, start + i, c));
                            }
                        }
                    }
                }
            }
            Some(2) => {
                if let Some(rc) = u16_at(d, o + 2) {
                    for i in 0..rc {
                        let r = o + 4 + i as usize * 6;
                        if let (Some(a), Some(b), Some(c)) = (u16_at(d, r), u16_at(d, r + 2), u16_at(d, r + 4)) {
                            if c != 0 && a <= b {
                                ranges.push((a, b, c));
                            }
                        }
                    }
                }
            }
            _ => {}
        }
        ranges.sort_by_key(|r| r.0);
        ClassDef { ranges }
    }
    fn class(&self, g: u16) -> u16 {
        // binary search of the sorted ranges
        let (mut lo, mut hi) = (0usize, self.ranges.len());
        while lo < hi {
            let mid = (lo + hi) / 2;
            let (a, b, c) = self.ranges[mid];
            if g < a {
                hi = mid;
            } else if g > b {
                lo = mid + 1;
            } else {
                return c;
            }
        }
        0
    }
}

/// One GPOS PairPos format 2 lookup: `class(g1) * n2 + class(g2)` indexes `values`.
#[derive(Clone, Debug)]
struct ClassPair {
    c1: ClassDef,
    c2: ClassDef,
    n2: usize,
    values: Vec<i16>,
}

/// Pair kerning: `kern` format-0 pairs and GPOS `kern`-feature PairPos lookups (format 1 pairs,
/// format 2 class pairs). Values are font units, summed across lookups like the OpenType
/// lookup order prescribes. `valueFormat2` (an adjustment to the *second* glyph's own advance)
/// is not read: every font in the set uses `valueFormat2 = 0`.
#[derive(Clone, Debug, Default)]
pub struct Kern {
    /// `(left << 16 | right, adjustment)`, sorted by key, duplicates summed.
    pairs: Vec<(u32, i16)>,
    class: Vec<ClassPair>,
}

impl Kern {
    pub(crate) fn value(&self, l: u16, r: u16) -> i16 {
        let key = (l as u32) << 16 | r as u32;
        let (mut lo, mut hi) = (0usize, self.pairs.len());
        let mut v = 0i16;
        while lo < hi {
            let mid = (lo + hi) / 2;
            match self.pairs[mid].0.cmp(&key) {
                std::cmp::Ordering::Less => lo = mid + 1,
                std::cmp::Ordering::Greater => hi = mid,
                std::cmp::Ordering::Equal => {
                    v += self.pairs[mid].1;
                    break;
                }
            }
        }
        for cp in &self.class {
            let (a, b) = (cp.c1.class(l) as usize, cp.c2.class(r) as usize);
            if a * cp.n2 + b < cp.values.len() {
                v = v.wrapping_add(cp.values[a * cp.n2 + b]);
            }
        }
        v
    }
    /// Record an exact pair (summing a repeated key).
    fn insert(&mut self, key: u32, val: i16) {
        if val == 0 {
            return;
        }
        let mut at = self.pairs.len();
        for (i, p) in self.pairs.iter().enumerate() {
            if p.0 >= key {
                at = i;
                break;
            }
        }
        if at < self.pairs.len() && self.pairs[at].0 == key {
            self.pairs[at].1 = self.pairs[at].1.wrapping_add(val);
        } else {
            self.pairs.insert(at, (key, val));
        }
    }
    /// `kern` table, format 0, horizontal non-cross-stream subtables.
    fn load_kern(&mut self, d: &[u8], at: Option<(usize, usize)>) {
        let Some((o, len)) = at else { return };
        let end = o + len;
        let Some(version) = u16_at(d, o) else { return };
        if version != 0 {
            return; // Apple extended format: not in any font of the set
        }
        let Some(n) = u16_at(d, o + 2) else { return };
        let mut p = o + 4;
        for _ in 0..n {
            let Some(sub_len) = u16_at(d, p + 2) else { return };
            let Some(cov) = u16_at(d, p + 4) else { return };
            let fmt = cov >> 8;
            let horizontal = cov & 1 != 0;
            let cross_stream = cov & 4 != 0;
            let minimum = cov & 2 != 0;
            if fmt == 0 && horizontal && !cross_stream && !minimum {
                if let Some(np) = u16_at(d, p + 6) {
                    for i in 0..np {
                        let r = p + 14 + i as usize * 6;
                        if r + 6 > end {
                            break;
                        }
                        if let (Some(l), Some(rt), Some(v)) = (u16_at(d, r), u16_at(d, r + 2), i16_at(d, r + 4)) {
                            self.insert((l as u32) << 16 | rt as u32, v);
                        }
                    }
                }
            }
            if sub_len < 6 {
                return;
            }
            p = p.saturating_add(sub_len as usize);
            if p > end {
                return;
            }
        }
    }
    /// GPOS `kern` feature: PairPos lookups (including extension lookups).
    fn load_gpos(&mut self, d: &[u8], at: Option<(usize, usize)>) {
        let Some((o, _)) = at else { return };
        let Some(fl) = u16_at(d, o + 6) else { return };
        let Some(ll) = u16_at(d, o + 8) else { return };
        let feature_list = o + fl as usize;
        let lookup_list = o + ll as usize;
        let Some(fc) = u16_at(d, feature_list) else { return };
        let mut wanted: Vec<u16> = vec![];
        for i in 0..fc {
            let r = feature_list + 2 + i as usize * 6;
            if d.get(r..r + 4) != Some(b"kern") {
                continue;
            }
            let Some(fo) = u16_at(d, r + 4) else { continue };
            let ft = feature_list + fo as usize;
            let Some(lic) = u16_at(d, ft + 2) else { continue };
            for j in 0..lic {
                if let Some(li) = u16_at(d, ft + 4 + j as usize * 2) {
                    if !wanted.contains(&li) {
                        wanted.push(li);
                    }
                }
            }
        }
        for li in wanted {
            let Some(lt_off) = u16_at(d, lookup_list + 2 + li as usize * 2) else { continue };
            let lt = lookup_list + lt_off as usize;
            let Some(ltype) = u16_at(d, lt) else { continue };
            let Some(stc) = u16_at(d, lt + 4) else { continue };
            for st in 0..stc {
                let Some(st_off) = u16_at(d, lt + 6 + st as usize * 2) else { continue };
                let mut so = lt + st_off as usize;
                let mut typ = ltype;
                if typ == 9 {
                    // ExtensionPos: uint16 posFormat, uint16 extensionLookupType, uint32 extensionOffset
                    typ = match u16_at(d, so + 2) {
                        Some(t) => t,
                        None => continue,
                    };
                    so += match u32_at(d, so + 4) {
                        Some(off) => off as usize,
                        None => continue,
                    };
                }
                if typ != 2 {
                    continue;
                }
                self.load_pair_pos(d, so);
            }
        }
    }
    fn load_pair_pos(&mut self, d: &[u8], so: usize) {
        let Some(fmt) = u16_at(d, so) else { return };
        let Some(cov_off) = u16_at(d, so + 2) else { return };
        let Some(vf1) = u16_at(d, so + 4) else { return };
        let vf2 = u16_at(d, so + 6).unwrap_or(0);
        let _ = vf2; // valueFormat2 not read (see struct doc)
        let cov = coverage_glyphs(d, so + cov_off as usize);
        match fmt {
            1 => {
                let Some(psc) = u16_at(d, so + 8) else { return };
                let rec = 2 + vf_size(vf1) + vf_size(vf2);
                for (i, &left) in cov.iter().enumerate().take(psc as usize) {
                    let Some(pso) = u16_at(d, so + 10 + i * 2) else { continue };
                    let ps = so + pso as usize;
                    let Some(pvc) = u16_at(d, ps) else { continue };
                    for k in 0..pvc {
                        let rp = ps + 2 + k as usize * rec;
                        let Some(right) = u16_at(d, rp) else { continue };
                        self.insert((left as u32) << 16 | right as u32, x_advance(d, rp + 2, vf1));
                    }
                }
            }
            2 => {
                let Some(cd1o) = u16_at(d, so + 8) else { return };
                let Some(cd2o) = u16_at(d, so + 10) else { return };
                let Some(n1) = u16_at(d, so + 12) else { return };
                let Some(n2) = u16_at(d, so + 14) else { return };
                let rec = vf_size(vf1) + vf_size(vf2);
                let mut values = Vec::with_capacity(n1 as usize * n2 as usize);
                for a in 0..n1 as usize {
                    for b in 0..n2 as usize {
                        values.push(x_advance(d, so + 16 + (a * n2 as usize + b) * rec, vf1));
                    }
                }
                self.class.push(ClassPair {
                    c1: ClassDef::parse(d, so + cd1o as usize),
                    c2: ClassDef::parse(d, so + cd2o as usize),
                    n2: n2 as usize,
                    values,
                });
            }
            _ => {}
        }
    }
}

/// Size in bytes of one value record for `vf`.
fn vf_size(vf: u16) -> usize {
    vf.count_ones() as usize * 2
}

/// The `XAdvance` field of a value record (0 when the format has none).
fn x_advance(d: &[u8], base: usize, vf: u16) -> i16 {
    if vf & 0x0004 == 0 {
        return 0;
    }
    let mut o = base;
    if vf & 0x0001 != 0 {
        o += 2;
    }
    if vf & 0x0002 != 0 {
        o += 2;
    }
    i16_at(d, o).unwrap_or(0)
}

/// Every glyph of a coverage table (formats 1 and 2), capped against runaway counts.
fn coverage_glyphs(d: &[u8], o: usize) -> Vec<u16> {
    let mut out = vec![];
    let Some(fmt) = u16_at(d, o) else { return out };
    let Some(n) = u16_at(d, o + 2) else { return out };
    match fmt {
        1 => {
            for i in 0..n {
                match u16_at(d, o + 4 + i as usize * 2) {
                    Some(g) => out.push(g),
                    None => break,
                }
            }
        }
        2 => {
            for i in 0..n {
                let r = o + 4 + i as usize * 6;
                match (u16_at(d, r), u16_at(d, r + 2)) {
                    (Some(a), Some(b)) if a <= b => {
                        for g in a..=b {
                            out.push(g);
                            if out.len() > 65_535 {
                                return out;
                            }
                        }
                    }
                    _ => {}
                }
            }
        }
        _ => {}
    }
    out
}

/// Standard ligatures (GSUB `liga` feature, LigatureSubst including extension lookups), keyed by
/// the first component glyph. Applied greedily, longest match first, only when the entity opts in.
#[derive(Clone, Debug, Default)]
pub struct Liga {
    rules: Vec<(u16, Vec<u16>, u16)>,
}

impl Liga {
    /// The ligature starting at `gids[i]`: total number of glyphs it consumes and the result glyph.
    pub(crate) fn ligature(&self, gids: &[u16], i: usize) -> Option<(usize, u16)> {
        let mut best: Option<(usize, u16)> = None;
        for (first, rest, lig) in &self.rules {
            if *first != gids[i] || i + 1 + rest.len() > gids.len() {
                continue;
            }
            if gids[i + 1..i + 1 + rest.len()] == rest[..] {
                let len = 1 + rest.len();
                if best.map_or(true, |(b, _)| len > b) {
                    best = Some((len, *lig));
                }
            }
        }
        best
    }
    fn load(&mut self, d: &[u8], at: Option<(usize, usize)>) {
        let Some((o, _)) = at else { return };
        let Some(fl) = u16_at(d, o + 6) else { return };
        let Some(ll) = u16_at(d, o + 8) else { return };
        let feature_list = o + fl as usize;
        let lookup_list = o + ll as usize;
        let Some(fc) = u16_at(d, feature_list) else { return };
        let mut wanted: Vec<u16> = vec![];
        for i in 0..fc {
            let r = feature_list + 2 + i as usize * 6;
            if d.get(r..r + 4) != Some(b"liga") {
                continue;
            }
            let Some(fo) = u16_at(d, r + 4) else { continue };
            let ft = feature_list + fo as usize;
            let Some(lic) = u16_at(d, ft + 2) else { continue };
            for j in 0..lic {
                if let Some(li) = u16_at(d, ft + 4 + j as usize * 2) {
                    if !wanted.contains(&li) {
                        wanted.push(li);
                    }
                }
            }
        }
        for li in wanted {
            let Some(lt_off) = u16_at(d, lookup_list + 2 + li as usize * 2) else { continue };
            let lt = lookup_list + lt_off as usize;
            let Some(ltype) = u16_at(d, lt) else { continue };
            let Some(stc) = u16_at(d, lt + 4) else { continue };
            for st in 0..stc {
                let Some(st_off) = u16_at(d, lt + 6 + st as usize * 2) else { continue };
                let mut so = lt + st_off as usize;
                let mut typ = ltype;
                if typ == 7 {
                    typ = match u16_at(d, so + 2) {
                        Some(t) => t,
                        None => continue,
                    };
                    so += match u32_at(d, so + 4) {
                        Some(off) => off as usize,
                        None => continue,
                    };
                }
                if typ != 4 {
                    continue;
                }
                self.load_ligature_subst(d, so);
            }
        }
    }
    fn load_ligature_subst(&mut self, d: &[u8], so: usize) {
        let Some(cov_off) = u16_at(d, so + 2) else { return };
        let Some(lsc) = u16_at(d, so + 4) else { return };
        let cov = coverage_glyphs(d, so + cov_off as usize);
        for (i, &first) in cov.iter().enumerate().take(lsc as usize) {
            let Some(lso) = u16_at(d, so + 6 + i * 2) else { continue };
            let ls = so + lso as usize;
            let Some(lc) = u16_at(d, ls) else { continue };
            for k in 0..lc {
                let lp = ls + 2 + k as usize * 2;
                let Some(loff) = u16_at(d, lp) else { continue };
                let lt = ls + loff as usize;
                let (Some(lig), Some(cc)) = (u16_at(d, lt), u16_at(d, lt + 2)) else { continue };
                if cc == 0 {
                    continue;
                }
                let mut rest = Vec::with_capacity(cc as usize - 1);
                for z in 0..cc - 1 {
                    match u16_at(d, lt + 4 + z as usize * 2) {
                        Some(g) => rest.push(g),
                        None => rest.clear(),
                    }
                }
                if rest.len() == cc as usize - 1 {
                    self.rules.push((first, rest, lig));
                }
            }
        }
    }
}

pub struct Font {
    data: Arc<[u8]>,
    pub units_per_em: f64,
    num_glyphs: usize,
    num_hmetrics: usize,
    hmtx: usize,
    cmap_off: usize,
    cmap_fmt: u16,
    cap_height: f64,
    kind: OutlineKind,
    loca_long: bool,
    loca: usize,
    glyf: usize,
    cff: Option<Cff>,
    tables: Vec<([u8; 4], usize, usize)>,
    pub notes: Notes,
    pub(crate) kern: Kern,
    pub(crate) liga: Liga,
}

fn u16_at(d: &[u8], o: usize) -> Option<u16> {
    Some(u16::from_be_bytes([*d.get(o)?, *d.get(o + 1)?]))
}
fn i16_at(d: &[u8], o: usize) -> Option<i16> {
    u16_at(d, o).map(|v| v as i16)
}
fn u32_at(d: &[u8], o: usize) -> Option<u32> {
    Some(u32::from_be_bytes([*d.get(o)?, *d.get(o + 1)?, *d.get(o + 2)?, *d.get(o + 3)?]))
}

impl Font {
    /// Parse a TrueType / OpenType font (or the first font of a collection).
    pub fn parse(data: Arc<[u8]>) -> Result<Font, String> {
        let d: &[u8] = &data;
        let tag = d.get(0..4).ok_or("file too short to be a font")?;
        let (dir, collection) = match tag {
            b"ttcf" => (u32_at(d, 12).ok_or("truncated collection header")? as usize, true),
            b"wOFF" | b"wOF2" => return Err("WOFF fonts are not supported; give the TTF or OTF file".into()),
            [0, 1, 0, 0] | b"true" | b"OTTO" => (0, false),
            _ => return Err("not a TrueType / OpenType font (unknown file signature)".into()),
        };
        let n = u16_at(d, dir + 4).ok_or("truncated table directory")? as usize;
        let mut tables = vec![];
        for i in 0..n {
            let r = dir + 12 + i * 16;
            let t = d.get(r..r + 4).ok_or("truncated table directory")?;
            let (off, len) = (u32_at(d, r + 8).unwrap_or(0) as usize, u32_at(d, r + 12).unwrap_or(0) as usize);
            if off > d.len() {
                return Err(format!("table {} lies outside the file", String::from_utf8_lossy(t)));
            }
            tables.push(([t[0], t[1], t[2], t[3]], off, len));
        }
        let table = |tag: &[u8; 4]| tables.iter().find(|t| &t.0 == tag).map(|t| t.1);
        let need = |tag: &[u8; 4]| table(tag).ok_or_else(|| format!("missing required table {}", String::from_utf8_lossy(tag)));
        let head = need(b"head")?;
        let maxp = need(b"maxp")?;
        let hhea = need(b"hhea")?;
        let hmtx = need(b"hmtx")?;
        let cmap = need(b"cmap")?;
        let units_per_em = u16_at(d, head + 18).ok_or("truncated head")? as f64;
        if units_per_em < 16.0 {
            return Err("unitsPerEm is out of range".into());
        }
        let num_glyphs = u16_at(d, maxp + 4).ok_or("truncated maxp")? as usize;
        let mut f = Font {
            data: data.clone(),
            units_per_em,
            num_glyphs,
            num_hmetrics: u16_at(d, hhea + 34).ok_or("truncated hhea")? as usize,
            hmtx,
            cmap_off: 0,
            cmap_fmt: 0,
            cap_height: 0.0,
            kind: OutlineKind::Glyf,
            loca_long: false,
            loca: 0,
            glyf: 0,
            cff: None,
            tables: tables.clone(),
            notes: Notes {
                variable: table(b"fvar").is_some(),
                kerning: table(b"kern").is_some() || table(b"GPOS").is_some(),
                substitution: table(b"GSUB").is_some(),
                collection,
                cap_height_estimated: false,
            },
            kern: Kern::default(),
            liga: Liga::default(),
        };
        if f.num_hmetrics == 0 {
            return Err("font has no horizontal metrics".into());
        }
        if let (Some(loca), Some(glyf)) = (table(b"loca"), table(b"glyf")) {
            f.loca = loca;
            f.glyf = glyf;
            f.loca_long = i16_at(d, head + 50).ok_or("truncated head")? != 0;
        } else if let Some(c) = table(b"CFF ") {
            f.kind = OutlineKind::Cff;
            f.cff = Some(Cff::parse(d, c, num_glyphs)?);
        } else if table(b"CFF2").is_some() {
            return Err("CFF2 outlines (variable OpenType) are not supported; use a static OTF or TTF".into());
        } else {
            return Err("font has no outlines (neither glyf nor CFF tables)".into());
        }
        f.pick_cmap(cmap)?;
        f.cap_height = match f.glyph_index('H').and_then(|g| f.contours(g)) {
            Some(c) => c.iter().flatten().map(|p| p.y).fold(0.0, f64::max),
            None => 0.0,
        };
        if f.cap_height <= 0.0 {
            // no H: the OS/2 sCapHeight (version 2 and later), else 0.7 em
            f.notes.cap_height_estimated = true;
            f.cap_height = table(b"OS/2")
                .filter(|&o| u16_at(d, o).unwrap_or(0) >= 2)
                .and_then(|o| i16_at(d, o + 88))
                .filter(|&c| c > 0)
                .map_or(0.7 * f.units_per_em, |c| c as f64);
        }
        let full = |tag: &[u8; 4]| tables.iter().find(|t| &t.0 == tag).map(|t| (t.1, t.2));
        f.kern.load_kern(d, full(b"kern"));
        f.kern.load_gpos(d, full(b"GPOS"));
        f.liga.load(d, full(b"GSUB"));
        Ok(f)
    }

    fn table(&self, tag: &[u8; 4]) -> Option<(usize, usize)> {
        self.tables.iter().find(|t| &t.0 == tag).map(|t| (t.1, t.2))
    }

    fn pick_cmap(&mut self, cmap: usize) -> Result<(), String> {
        let d = &self.data[..];
        let nt = u16_at(d, cmap + 2).ok_or("truncated cmap")? as usize;
        let mut best: Option<(u32, usize, u16)> = None;
        for i in 0..nt {
            let r = cmap + 4 + i * 8;
            let (Some(plat), Some(enc), Some(off)) = (u16_at(d, r), u16_at(d, r + 2), u32_at(d, r + 4)) else { break };
            let st = cmap + off as usize;
            let Some(fmt) = u16_at(d, st) else { continue };
            if fmt != 4 && fmt != 12 {
                continue;
            }
            let rank = match (plat, enc) {
                (3, 10) => 3,
                (0, 4) | (0, 6) => 3,
                (3, 1) => 2,
                (0, _) => 1,
                _ => 0,
            };
            if rank > 0 && best.map_or(true, |b| rank > b.0) {
                best = Some((rank, st, fmt));
            }
        }
        let (_, st, fmt) = best.ok_or("no Unicode cmap subtable of format 4 or 12")?;
        self.cmap_off = st;
        self.cmap_fmt = fmt;
        Ok(())
    }

    pub fn kind(&self) -> OutlineKind {
        self.kind
    }
    pub fn num_glyphs(&self) -> usize {
        self.num_glyphs
    }
    /// OS/2 `fsType` (embedding licence flags), if the font has an OS/2 table.
    pub fn fs_type(&self) -> Option<u16> {
        let (o, _) = self.table(b"OS/2")?;
        u16_at(&self.data, o + 8)
    }
    /// The raw file.
    pub fn bytes(&self) -> &[u8] {
        &self.data
    }

    /// Height of the capital letters (the 'H' glyph), in font units.
    pub fn cap_height(&self) -> f64 {
        self.cap_height
    }

    /// Glyph id of a character (`None`: the font has no glyph for it).
    pub fn glyph_index(&self, ch: char) -> Option<u16> {
        let d = &self.data[..];
        let cp = ch as u32;
        let st = self.cmap_off;
        let g = match self.cmap_fmt {
            4 => {
                if cp > 0xFFFF {
                    return None;
                }
                let sc = u16_at(d, st + 6)? as usize / 2;
                let ends = st + 14;
                let starts = ends + sc * 2 + 2;
                let deltas = starts + sc * 2;
                let ranges = deltas + sc * 2;
                // segments are sorted by end code: the first one that ends at or after cp
                let (mut lo, mut hi) = (0usize, sc);
                while lo < hi {
                    let mid = (lo + hi) / 2;
                    if u16_at(d, ends + mid * 2)? as u32 >= cp {
                        hi = mid;
                    } else {
                        lo = mid + 1;
                    }
                }
                let mut found = None;
                if lo < sc {
                    let s = lo;
                    let start = u16_at(d, starts + s * 2)? as u32;
                    if cp >= start {
                        let delta = u16_at(d, deltas + s * 2)?;
                        let ro = u16_at(d, ranges + s * 2)? as usize;
                        let g = if ro == 0 {
                            (cp as u16).wrapping_add(delta)
                        } else {
                            let a = ranges + s * 2 + ro + (cp - start) as usize * 2;
                            let g = u16_at(d, a)?;
                            if g == 0 {
                                0
                            } else {
                                g.wrapping_add(delta)
                            }
                        };
                        found = Some(g);
                    }
                }
                found?
            }
            12 => {
                let ng = u32_at(d, st + 12)? as usize;
                // groups are sorted by start code
                let (mut lo, mut hi) = (0usize, ng);
                let mut found = None;
                while lo < hi {
                    let mid = (lo + hi) / 2;
                    let r = st + 16 + mid * 12;
                    let (a, b, g0) = (u32_at(d, r)?, u32_at(d, r + 4)?, u32_at(d, r + 8)?);
                    if cp < a {
                        hi = mid;
                    } else if cp > b {
                        lo = mid + 1;
                    } else {
                        found = Some((g0 + cp - a) as u16);
                        break;
                    }
                }
                found?
            }
            _ => return None,
        };
        (g != 0 && (g as usize) < self.num_glyphs).then_some(g)
    }

    /// Every character the cmap maps to a glyph, ascending.
    pub fn mapped_chars(&self) -> Vec<char> {
        let d = &self.data[..];
        let st = self.cmap_off;
        let mut ranges: Vec<(u32, u32)> = vec![];
        match self.cmap_fmt {
            4 => {
                let sc = u16_at(d, st + 6).unwrap_or(0) as usize / 2;
                let ends = st + 14;
                let starts = ends + sc * 2 + 2;
                for s in 0..sc {
                    if let (Some(e), Some(b)) = (u16_at(d, ends + s * 2), u16_at(d, starts + s * 2)) {
                        ranges.push((b as u32, (e as u32).min(0xFFFE)));
                    }
                }
            }
            12 => {
                let ng = u32_at(d, st + 12).unwrap_or(0) as usize;
                for i in 0..ng.min(d.len() / 12) {
                    let r = st + 16 + i * 12;
                    if let (Some(a), Some(b)) = (u32_at(d, r), u32_at(d, r + 4)) {
                        ranges.push((a, b.min(0x10FFFF)));
                    }
                }
            }
            _ => {}
        }
        let mut out = vec![];
        for (a, b) in ranges {
            for cp in a..=b {
                if let Some(c) = char::from_u32(cp) {
                    // cmap segments are ascending; a damaged one that is not just loses the out-of-order characters
                    if out.last().map_or(true, |&l| c > l) && self.glyph_index(c).is_some() {
                        out.push(c);
                    }
                }
            }
            if out.len() > 0x11_0000 {
                break;
            }
        }
        out
    }

    /// Family and style name for display (name id 4, else 1), if the font has a name table.
    pub fn display_name(&self) -> Option<String> {
        let d = &self.data[..];
        let (n, _) = self.table(b"name")?;
        let count = u16_at(d, n + 2)? as usize;
        let so = n + u16_at(d, n + 4)? as usize;
        let mut best: Option<(u32, String)> = None;
        for i in 0..count {
            let r = n + 6 + i * 12;
            let (plat, id, len, off) = (u16_at(d, r)?, u16_at(d, r + 6)?, u16_at(d, r + 8)? as usize, u16_at(d, r + 10)? as usize);
            if id != 4 && id != 1 {
                continue;
            }
            let b = d.get(so + off..so + off + len)?;
            let s = match plat {
                3 | 0 => String::from_utf16_lossy(&b.chunks_exact(2).map(|c| u16::from_be_bytes([c[0], c[1]])).collect::<Vec<_>>()),
                1 => b.iter().map(|&c| c as char).collect(),
                _ => continue,
            };
            let rank = (if id == 4 { 2 } else { 0 }) + (if plat == 3 { 1 } else { 0 });
            if !s.is_empty() && best.as_ref().map_or(true, |b| rank > b.0) {
                best = Some((rank, s));
            }
        }
        best.map(|b| b.1)
    }

    /// Horizontal advance of a glyph, font units.
    pub fn advance(&self, g: u16) -> f64 {
        let i = (g as usize).min(self.num_hmetrics.saturating_sub(1));
        u16_at(&self.data, self.hmtx + i * 4).unwrap_or(0) as f64
    }

    fn glyph_range(&self, g: u16) -> Option<(usize, usize)> {
        let g = g as usize;
        if g >= self.num_glyphs {
            return None;
        }
        let d = &self.data[..];
        let (a, b) = if self.loca_long {
            (u32_at(d, self.loca + g * 4)? as usize, u32_at(d, self.loca + g * 4 + 4)? as usize)
        } else {
            (u16_at(d, self.loca + g * 2)? as usize * 2, u16_at(d, self.loca + g * 2 + 2)? as usize * 2)
        };
        Some((self.glyf + a, self.glyf + b))
    }

    /// Outline of a glyph (composites resolved), font units, y up. Empty for blank glyphs (space).
    pub fn contours(&self, g: u16) -> Option<Contours> {
        match self.kind {
            OutlineKind::Glyf => {
                let mut budget = MAX_GLYPH_POINTS;
                self.contours_depth(g, 0, &mut budget)
            }
            OutlineKind::Cff => self.cff.as_ref()?.glyph(&self.data, g, self.units_per_em),
        }
    }

    fn contours_depth(&self, g: u16, depth: u32, budget: &mut usize) -> Option<Contours> {
        let d = &self.data[..];
        let (a, b) = self.glyph_range(g)?;
        if b <= a {
            return Some(vec![]);
        }
        let nc = i16_at(d, a)?;
        if nc >= 0 {
            let nc = nc as usize;
            let mut end_pts = Vec::with_capacity(nc);
            for i in 0..nc {
                end_pts.push(u16_at(d, a + 10 + i * 2)? as usize);
            }
            let np = end_pts.last().map_or(0, |e| e + 1);
            if np > *budget {
                return None;
            }
            *budget -= np;
            let il = u16_at(d, a + 10 + nc * 2)? as usize;
            let mut p = a + 12 + nc * 2 + il;
            let mut flags = Vec::with_capacity(np);
            while flags.len() < np {
                let f = *d.get(p)?;
                p += 1;
                flags.push(f);
                if f & 8 != 0 {
                    let r = *d.get(p)?;
                    p += 1;
                    for _ in 0..r {
                        flags.push(f);
                    }
                }
            }
            flags.truncate(np);
            let mut xs = Vec::with_capacity(np);
            let mut v = 0i32;
            for &f in &flags {
                if f & 2 != 0 {
                    let dx = *d.get(p)? as i32;
                    p += 1;
                    v += if f & 16 != 0 { dx } else { -dx };
                } else if f & 16 == 0 {
                    v += i16_at(d, p)? as i32;
                    p += 2;
                }
                xs.push(v);
            }
            let mut ys = Vec::with_capacity(np);
            v = 0;
            for &f in &flags {
                if f & 4 != 0 {
                    let dy = *d.get(p)? as i32;
                    p += 1;
                    v += if f & 32 != 0 { dy } else { -dy };
                } else if f & 32 == 0 {
                    v += i16_at(d, p)? as i32;
                    p += 2;
                }
                ys.push(v);
            }
            let mut out = Vec::with_capacity(nc);
            let mut s = 0;
            for &e in &end_pts {
                if e < s || e >= np {
                    return None;
                }
                out.push((s..=e).map(|i| Pt { x: xs[i] as f64, y: ys[i] as f64, on: flags[i] & 1 != 0 }).collect());
                s = e + 1;
            }
            Some(out)
        } else {
            if depth > 4 {
                return None;
            }
            let mut out: Contours = vec![];
            let mut p = a + 10;
            loop {
                let fl = u16_at(d, p)?;
                let gi = u16_at(d, p + 2)?;
                p += 4;
                let (mut dx, mut dy) = (0.0, 0.0);
                if fl & 1 != 0 {
                    if fl & 2 != 0 {
                        dx = i16_at(d, p)? as f64;
                        dy = i16_at(d, p + 2)? as f64;
                    }
                    p += 4;
                } else {
                    if fl & 2 != 0 {
                        dx = *d.get(p)? as i8 as f64;
                        dy = *d.get(p + 1)? as i8 as f64;
                    }
                    p += 2;
                }
                let f2 = |o: usize| i16_at(d, o).map(|v| v as f64 / 16384.0);
                let (mut m00, mut m01, mut m10, mut m11) = (1.0, 0.0, 0.0, 1.0);
                if fl & 0x8 != 0 {
                    m00 = f2(p)?;
                    m11 = m00;
                    p += 2;
                } else if fl & 0x40 != 0 {
                    m00 = f2(p)?;
                    m11 = f2(p + 2)?;
                    p += 4;
                } else if fl & 0x80 != 0 {
                    m00 = f2(p)?;
                    m01 = f2(p + 2)?;
                    m10 = f2(p + 4)?;
                    m11 = f2(p + 6)?;
                    p += 8;
                }
                for c in self.contours_depth(gi, depth + 1, budget)? {
                    out.push(
                        c.into_iter()
                            .map(|q| Pt { x: m00 * q.x + m10 * q.y + dx, y: m01 * q.x + m11 * q.y + dy, on: q.on })
                            .collect(),
                    );
                }
                if fl & 0x20 == 0 {
                    break;
                }
            }
            Some(out)
        }
    }

    // -----------------------------------------------------------------------------------------
    // subsetting

    /// A plain TrueType font (glyf outlines, whatever the source had) holding just the glyphs of `chars`
    /// (plus `.notdef` and `H`, which carries the cap height), same unitsPerEm, same cmap entries for these
    /// characters, same advances. Composites are flattened; OpenType layout tables are dropped.
    pub fn subset(&self, chars: &[char]) -> Result<Vec<u8>, String> {
        let mut want: Vec<(char, u16)> = vec![];
        for &c in chars.iter().chain(&['H']) {
            if let Some(g) = self.glyph_index(c) {
                let at = want.partition_point(|w| w.0 < c);
                if want.get(at).map_or(true, |w| w.0 != c) {
                    want.insert(at, (c, g));
                }
            }
        }
        let mut gids: Vec<u16> = vec![0];
        for w in &want {
            let at = 1 + gids[1..].partition_point(|&g| g < w.1);
            if gids.get(at) != Some(&w.1) {
                gids.insert(at, w.1);
            }
        }
        let n = gids.len();
        let (mut glyf, mut loca, mut hmtx) = (vec![], vec![0u8; 0], vec![]);
        let (mut max_pts, mut max_cont) = (0usize, 0usize);
        let mut bbox = [i16::MAX, i16::MAX, i16::MIN, i16::MIN];
        for &g in &gids {
            loca.extend((glyf.len() as u32).to_be_bytes());
            let cs = self.contours(g).ok_or_else(|| format!("glyph {g} cannot be read"))?;
            let cs: Vec<Vec<(i16, i16, bool)>> = cs
                .iter()
                .map(|c| c.iter().map(|p| (p.x.round().clamp(-32768.0, 32767.0) as i16, p.y.round().clamp(-32768.0, 32767.0) as i16, p.on)).collect())
                .collect();
            let all: Vec<&(i16, i16, bool)> = cs.iter().flatten().collect();
            let mut x0 = 0i16;
            if !all.is_empty() {
                x0 = all.iter().map(|p| p.0).min().unwrap();
                let x1 = all.iter().map(|p| p.0).max().unwrap();
                let y0 = all.iter().map(|p| p.1).min().unwrap();
                let y1 = all.iter().map(|p| p.1).max().unwrap();
                bbox = [bbox[0].min(x0), bbox[1].min(y0), bbox[2].max(x1), bbox[3].max(y1)];
                max_pts = max_pts.max(all.len());
                max_cont = max_cont.max(cs.len());
                glyf.extend((cs.len() as i16).to_be_bytes());
                for v in [x0, y0, x1, y1] {
                    glyf.extend(v.to_be_bytes());
                }
                let mut e = 0u16;
                for c in &cs {
                    e += c.len() as u16;
                    glyf.extend((e - 1).to_be_bytes());
                }
                glyf.extend(0u16.to_be_bytes());
                glyf.extend(all.iter().map(|p| p.2 as u8));
                let mut prev = 0i16;
                for p in &all {
                    glyf.extend(p.0.wrapping_sub(prev).to_be_bytes());
                    prev = p.0;
                }
                prev = 0;
                for p in &all {
                    glyf.extend(p.1.wrapping_sub(prev).to_be_bytes());
                    prev = p.1;
                }
                while glyf.len() % 4 != 0 {
                    glyf.push(0);
                }
            }
            hmtx.extend((self.advance(g) as u16).to_be_bytes());
            hmtx.extend(x0.to_be_bytes());
        }
        loca.extend((glyf.len() as u32).to_be_bytes());
        if bbox[0] > bbox[2] {
            bbox = [0; 4];
        }
        let d = &self.data[..];
        let raw = |tag: &[u8; 4], _min: usize| self.table(tag).and_then(|(o, l)| d.get(o..o + l).map(|s| s.to_vec()));
        // head: original with the loca format, bbox and checksum adjustment replaced
        let mut head = raw(b"head", 54).filter(|h| h.len() >= 54).ok_or("head table is too short")?;
        head[8..12].copy_from_slice(&[0; 4]);
        for (i, v) in bbox.iter().enumerate() {
            head[36 + i * 2..38 + i * 2].copy_from_slice(&v.to_be_bytes());
        }
        head[50..52].copy_from_slice(&1i16.to_be_bytes());
        head.truncate(54);
        let mut hhea = raw(b"hhea", 36).filter(|h| h.len() >= 36).ok_or("hhea table is too short")?;
        hhea.truncate(36);
        hhea[34..36].copy_from_slice(&(n as u16).to_be_bytes());
        let mut maxp = vec![0, 1, 0, 0];
        maxp.extend((n as u16).to_be_bytes());
        for v in [max_pts as u16, max_cont as u16, 0, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0] {
            maxp.extend(v.to_be_bytes());
        }
        // cmap: format 4 for the BMP characters, format 12 for all
        let new_gid = |old: u16| gids.iter().position(|&g| g == old).unwrap() as u16;
        let map: Vec<(u32, u16)> = want.iter().map(|w| (w.0 as u32, new_gid(w.1))).collect();
        let mut cmap = vec![0, 0, 0, 2];
        let f4 = cmap_format4(&map.iter().filter(|m| m.0 < 0xFFFF).copied().collect::<Vec<_>>());
        let f12 = cmap_format12(&map);
        cmap.extend([0, 3, 0, 1]);
        cmap.extend(20u32.to_be_bytes());
        cmap.extend([0, 3, 0, 10]);
        cmap.extend((20 + f4.len() as u32).to_be_bytes());
        cmap.extend(f4);
        cmap.extend(f12);
        let mut post = raw(b"post", 32).filter(|p| p.len() >= 32).map(|mut p| {
            p.truncate(32);
            p[0..4].copy_from_slice(&[0, 3, 0, 0]);
            p
        });
        if post.is_none() {
            let mut p = vec![0u8; 32];
            p[0..4].copy_from_slice(&[0, 3, 0, 0]);
            post = Some(p);
        }
        let mut tabs: Vec<([u8; 4], Vec<u8>)> = vec![
            (*b"cmap", cmap),
            (*b"glyf", glyf),
            (*b"head", head),
            (*b"hhea", hhea),
            (*b"hmtx", hmtx),
            (*b"loca", loca),
            (*b"maxp", maxp),
            (*b"post", post.unwrap()),
        ];
        for t in [b"name", b"OS/2"] {
            if let Some(r) = raw(t, 0) {
                tabs.push((*t, r));
            }
        }
        // directory order: ascending tags (ten entries)
        for i in 1..tabs.len() {
            let mut j = i;
            while j > 0 && tabs[j].0 < tabs[j - 1].0 {
                tabs.swap(j, j - 1);
                j -= 1;
            }
        }
        Ok(write_sfnt(tabs))
    }
}

fn cmap_format4(map: &[(u32, u16)]) -> Vec<u8> {
    // one segment per run of consecutive characters with consecutive glyphs (idDelta), plus the 0xFFFF sentinel
    let mut segs: Vec<(u16, u16, u16)> = vec![];
    for &(c, g) in map {
        let c = c as u16;
        match segs.last_mut() {
            Some(s) if s.1.wrapping_add(1) == c && c != 0 && (g.wrapping_sub(c) == s.2) => s.1 = c,
            _ => segs.push((c, c, g.wrapping_sub(c))),
        }
    }
    segs.push((0xFFFF, 0xFFFF, 1));
    let sc = segs.len();
    let mut out = vec![];
    out.extend(4u16.to_be_bytes());
    out.extend((16 + sc as u16 * 8).to_be_bytes());
    out.extend(0u16.to_be_bytes());
    out.extend((sc as u16 * 2).to_be_bytes());
    let mut p = 1u16;
    let mut e = 0u16;
    while p * 2 <= sc as u16 {
        p *= 2;
        e += 1;
    }
    out.extend((p * 2).to_be_bytes());
    out.extend(e.to_be_bytes());
    out.extend((sc as u16 * 2 - p * 2).to_be_bytes());
    for s in &segs {
        out.extend(s.1.to_be_bytes());
    }
    out.extend(0u16.to_be_bytes());
    for s in &segs {
        out.extend(s.0.to_be_bytes());
    }
    for s in &segs {
        out.extend(s.2.to_be_bytes());
    }
    for _ in &segs {
        out.extend(0u16.to_be_bytes());
    }
    out
}

fn cmap_format12(map: &[(u32, u16)]) -> Vec<u8> {
    let mut groups: Vec<(u32, u32, u32)> = vec![];
    for &(c, g) in map {
        match groups.last_mut() {
            Some(s) if s.1 + 1 == c && s.2 + (s.1 - s.0) + 1 == g as u32 => s.1 = c,
            _ => groups.push((c, c, g as u32)),
        }
    }
    let mut out = vec![];
    out.extend(12u16.to_be_bytes());
    out.extend(0u16.to_be_bytes());
    out.extend((16 + groups.len() as u32 * 12).to_be_bytes());
    out.extend(0u32.to_be_bytes());
    out.extend((groups.len() as u32).to_be_bytes());
    for g in groups {
        for v in [g.0, g.1, g.2] {
            out.extend(v.to_be_bytes());
        }
    }
    out
}

fn checksum(b: &[u8]) -> u32 {
    let mut s = 0u32;
    for c in b.chunks(4) {
        let mut w = [0u8; 4];
        w[..c.len()].copy_from_slice(c);
        s = s.wrapping_add(u32::from_be_bytes(w));
    }
    s
}

fn write_sfnt(tabs: Vec<([u8; 4], Vec<u8>)>) -> Vec<u8> {
    let n = tabs.len() as u16;
    let mut out = vec![0, 1, 0, 0];
    out.extend(n.to_be_bytes());
    let mut p = 1u16;
    let mut e = 0u16;
    while p * 2 <= n {
        p *= 2;
        e += 1;
    }
    out.extend((p * 16).to_be_bytes());
    out.extend(e.to_be_bytes());
    out.extend((n * 16 - p * 16).to_be_bytes());
    let mut off = 12 + tabs.len() * 16;
    let mut body = vec![];
    let mut head_at = None;
    for (tag, data) in &tabs {
        out.extend(tag);
        out.extend(checksum(data).to_be_bytes());
        out.extend((off as u32).to_be_bytes());
        out.extend((data.len() as u32).to_be_bytes());
        if tag == b"head" {
            head_at = Some(off);
        }
        body.extend(data);
        while body.len() % 4 != 0 {
            body.push(0);
        }
        off = 12 + tabs.len() * 16 + body.len();
    }
    out.extend(body);
    if let Some(h) = head_at {
        let adj = 0xB1B0_AFBAu32.wrapping_sub(checksum(&out));
        out[h + 8..h + 12].copy_from_slice(&adj.to_be_bytes());
    }
    out
}

// ---------------------------------------------------------------------------------------------
// CFF (Compact Font Format, Type 2 charstrings)

#[derive(Clone, Copy)]
struct Index {
    count: usize,
    off_size: usize,
    offs: usize,
    /// position just before the first data byte (offsets are 1-based)
    data: usize,
    end: usize,
}

fn read_off(d: &[u8], p: usize, size: usize) -> Option<usize> {
    let mut v = 0usize;
    for i in 0..size {
        v = (v << 8) | *d.get(p + i)? as usize;
    }
    Some(v)
}

impl Index {
    fn read(d: &[u8], p: usize) -> Option<Index> {
        let count = u16_at(d, p)? as usize;
        if count == 0 {
            return Some(Index { count: 0, off_size: 1, offs: 0, data: 0, end: p + 2 });
        }
        let off_size = *d.get(p + 2)? as usize;
        if !(1..=4).contains(&off_size) {
            return None;
        }
        let offs = p + 3;
        let last = read_off(d, offs + count * off_size, off_size)?;
        let data = offs + (count + 1) * off_size - 1;
        Some(Index { count, off_size, offs, data, end: data + last })
    }
    fn get<'a>(&self, d: &'a [u8], i: usize) -> Option<&'a [u8]> {
        if i >= self.count {
            return None;
        }
        let a = read_off(d, self.offs + i * self.off_size, self.off_size)?;
        let b = read_off(d, self.offs + (i + 1) * self.off_size, self.off_size)?;
        if a == 0 || b < a {
            return None;
        }
        d.get(self.data + a..self.data + b)
    }
}

/// Operators and operands of a DICT.
fn parse_dict(b: &[u8]) -> Vec<(u16, Vec<f64>)> {
    let (mut out, mut args, mut i) = (vec![], vec![], 0usize);
    while i < b.len() {
        let b0 = b[i];
        i += 1;
        match b0 {
            0..=21 => {
                let op = if b0 == 12 {
                    i += 1;
                    0x0c00 | *b.get(i - 1).unwrap_or(&0) as u16
                } else {
                    b0 as u16
                };
                out.push((op, std::mem::take(&mut args)));
            }
            28 => {
                args.push(i16_at(b, i).unwrap_or(0) as f64);
                i += 2;
            }
            29 => {
                args.push(u32_at(b, i).unwrap_or(0) as i32 as f64);
                i += 4;
            }
            30 => {
                let mut s = String::new();
                'n: while i < b.len() {
                    let byte = b[i];
                    i += 1;
                    for nib in [byte >> 4, byte & 15] {
                        match nib {
                            0..=9 => s.push((b'0' + nib) as char),
                            10 => s.push('.'),
                            11 => s.push('E'),
                            12 => s.push_str("E-"),
                            14 => s.push('-'),
                            15 => break 'n,
                            _ => {}
                        }
                    }
                }
                args.push(s.parse().unwrap_or(0.0));
            }
            32..=246 => args.push(b0 as f64 - 139.0),
            247..=250 => {
                args.push((b0 as f64 - 247.0) * 256.0 + *b.get(i).unwrap_or(&0) as f64 + 108.0);
                i += 1;
            }
            251..=254 => {
                args.push(-(b0 as f64 - 251.0) * 256.0 - *b.get(i).unwrap_or(&0) as f64 - 108.0);
                i += 1;
            }
            _ => {}
        }
        if args.len() > 64 {
            break;
        }
    }
    out
}

struct Cff {
    charstrings: Index,
    gsubrs: Index,
    /// FDSelect table position (CID fonts)
    fd_select: Option<usize>,
    /// local subroutines per font DICT (one entry for a name-keyed font)
    local: Vec<Option<Index>>,
}

impl Cff {
    fn parse(d: &[u8], base: usize, num_glyphs: usize) -> Result<Cff, String> {
        let bad = || "damaged CFF table".to_string();
        let hdr = *d.get(base + 2).ok_or_else(bad)? as usize;
        let names = Index::read(d, base + hdr).ok_or_else(bad)?;
        let tops = Index::read(d, names.end).ok_or_else(bad)?;
        let strings = Index::read(d, tops.end).ok_or_else(bad)?;
        let gsubrs = Index::read(d, strings.end).ok_or_else(bad)?;
        let top = parse_dict(tops.get(d, 0).ok_or_else(bad)?);
        let find = |op: u16| top.iter().find(|t| t.0 == op).map(|t| t.1.clone());
        if find(0x0c06).map_or(false, |v| v.first().map_or(false, |&t| t != 2.0)) {
            return Err("CFF font uses Type 1 charstrings, not supported".into());
        }
        let cs_off = find(17).and_then(|v| v.first().copied()).ok_or_else(bad)? as usize;
        let charstrings = Index::read(d, base + cs_off).ok_or_else(bad)?;
        if charstrings.count < num_glyphs.min(1) {
            return Err(bad());
        }
        let private_subrs = |dict: &[(u16, Vec<f64>)]| -> Option<Index> {
            let p = dict.iter().find(|t| t.0 == 18)?;
            let (size, off) = (*p.1.first()? as usize, *p.1.get(1)? as usize);
            let pd = parse_dict(d.get(base + off..base + off + size)?);
            let sub = pd.iter().find(|t| t.0 == 19)?.1.first().copied()? as usize;
            Index::read(d, base + off + sub)
        };
        let mut local = vec![];
        let mut fd_select = None;
        if let Some(fda) = find(0x0c24).and_then(|v| v.first().copied()) {
            let fdi = Index::read(d, base + fda as usize).ok_or_else(bad)?;
            for i in 0..fdi.count.min(256) {
                let fd = parse_dict(fdi.get(d, i).ok_or_else(bad)?);
                local.push(private_subrs(&fd));
            }
            fd_select = find(0x0c25).and_then(|v| v.first().map(|&o| base + o as usize));
        } else {
            local.push(private_subrs(&top));
        }
        Ok(Cff { charstrings, gsubrs, fd_select, local })
    }

    fn fd_of(&self, d: &[u8], g: usize) -> usize {
        let Some(p) = self.fd_select else { return 0 };
        match d.get(p) {
            Some(0) => d.get(p + 1 + g).map_or(0, |&v| v as usize),
            Some(3) => {
                let n = u16_at(d, p + 1).unwrap_or(0) as usize;
                for i in 0..n {
                    let r = p + 3 + i * 3;
                    let (first, fd) = (u16_at(d, r).unwrap_or(0) as usize, d.get(r + 2).copied().unwrap_or(0) as usize);
                    let next = u16_at(d, r + 3).unwrap_or(0xFFFF) as usize;
                    if g >= first && g < next {
                        return fd;
                    }
                }
                0
            }
            _ => 0,
        }
    }

    fn glyph(&self, d: &[u8], g: u16, upm: f64) -> Option<Contours> {
        let code = self.charstrings.get(d, g as usize)?;
        let local = self.local.get(self.fd_of(d, g as usize)).copied().flatten();
        let mut st = St { stack: vec![], x: 0.0, y: 0.0, cur: vec![], out: vec![], nstems: 0, width_done: false, ops: 0, points: 0, tol: upm * 0.0005 };
        let mut ctx = Ctx { d, gsubrs: self.gsubrs, local };
        ctx.run(code, 0, &mut st)?;
        st.close();
        Some(st.out)
    }
}

struct Ctx<'a> {
    d: &'a [u8],
    gsubrs: Index,
    local: Option<Index>,
}

struct St {
    stack: Vec<f64>,
    x: f64,
    y: f64,
    cur: Vec<Pt>,
    out: Contours,
    nstems: usize,
    width_done: bool,
    ops: usize,
    points: usize,
    tol: f64,
}

fn bias(n: usize) -> i64 {
    if n < 1240 {
        107
    } else if n < 33900 {
        1131
    } else {
        32768
    }
}

impl St {
    fn close(&mut self) {
        let mut c = std::mem::take(&mut self.cur);
        if c.len() > 1 && c[0].on && c.last().map_or(false, |l| l.on && l.x == c[0].x && l.y == c[0].y) {
            c.pop();
        }
        if c.len() >= 3 {
            self.out.push(c);
        }
    }
    fn move_to(&mut self, dx: f64, dy: f64) {
        self.close();
        self.x += dx;
        self.y += dy;
        self.cur.push(Pt { x: self.x, y: self.y, on: true });
    }
    fn ensure_start(&mut self) {
        if self.cur.is_empty() {
            self.cur.push(Pt { x: self.x, y: self.y, on: true });
        }
    }
    fn line(&mut self, dx: f64, dy: f64) {
        self.ensure_start();
        self.x += dx;
        self.y += dy;
        self.cur.push(Pt { x: self.x, y: self.y, on: true });
        self.points += 1;
    }
    fn curve(&mut self, d1: (f64, f64), d2: (f64, f64), d3: (f64, f64)) {
        self.ensure_start();
        let p0 = (self.x, self.y);
        let p1 = (p0.0 + d1.0, p0.1 + d1.1);
        let p2 = (p1.0 + d2.0, p1.1 + d2.1);
        let p3 = (p2.0 + d3.0, p2.1 + d3.1);
        cubic_to_quads(p0, p1, p2, p3, self.tol, 0, &mut self.cur);
        self.x = p3.0;
        self.y = p3.1;
        self.points += 8;
    }
}

#[cfg(test)]
pub fn cubic_to_quads_for_test(p0: (f64, f64), p1: (f64, f64), p2: (f64, f64), p3: (f64, f64), tol: f64, out: &mut Vec<Pt>) {
    cubic_to_quads(p0, p1, p2, p3, tol, 0, out)
}

/// Approximate a cubic Bezier by quadratics (off-curve control, on-curve end points appended to `out`).
fn cubic_to_quads(p0: (f64, f64), p1: (f64, f64), p2: (f64, f64), p3: (f64, f64), tol: f64, depth: u32, out: &mut Vec<Pt>) {
    let q = ((3.0 * (p1.0 + p2.0) - p0.0 - p3.0) / 4.0, (3.0 * (p1.1 + p2.1) - p0.1 - p3.1) / 4.0);
    let mut err = 0.0f64;
    for i in 1..8 {
        let t = i as f64 / 8.0;
        let u = 1.0 - t;
        let c = (
            u * u * u * p0.0 + 3.0 * u * u * t * p1.0 + 3.0 * u * t * t * p2.0 + t * t * t * p3.0,
            u * u * u * p0.1 + 3.0 * u * u * t * p1.1 + 3.0 * u * t * t * p2.1 + t * t * t * p3.1,
        );
        let s = (u * u * p0.0 + 2.0 * u * t * q.0 + t * t * p3.0, u * u * p0.1 + 2.0 * u * t * q.1 + t * t * p3.1);
        err = err.max((c.0 - s.0).hypot(c.1 - s.1));
    }
    if err <= tol || depth >= 5 {
        out.push(Pt { x: q.0, y: q.1, on: false });
        out.push(Pt { x: p3.0, y: p3.1, on: true });
        return;
    }
    let m = |a: (f64, f64), b: (f64, f64)| ((a.0 + b.0) * 0.5, (a.1 + b.1) * 0.5);
    let (a, b, c) = (m(p0, p1), m(p1, p2), m(p2, p3));
    let (ab, bc) = (m(a, b), m(b, c));
    let mid = m(ab, bc);
    cubic_to_quads(p0, a, ab, mid, tol, depth + 1, out);
    cubic_to_quads(mid, bc, c, p3, tol, depth + 1, out);
}

impl Ctx<'_> {
    /// Run a charstring; `Some(true)` when `endchar` was reached.
    fn run(&mut self, code: &[u8], depth: u32, st: &mut St) -> Option<bool> {
        if depth > 10 {
            return None;
        }
        let mut i = 0usize;
        while i < code.len() {
            st.ops += 1;
            if st.ops > 200_000 || st.points > MAX_GLYPH_POINTS || st.stack.len() > 96 {
                return None;
            }
            let b0 = code[i];
            i += 1;
            match b0 {
                28 => {
                    st.stack.push(i16_at(code, i)? as f64);
                    i += 2;
                }
                32..=246 => st.stack.push(b0 as f64 - 139.0),
                247..=250 => {
                    st.stack.push((b0 as f64 - 247.0) * 256.0 + *code.get(i)? as f64 + 108.0);
                    i += 1;
                }
                251..=254 => {
                    st.stack.push(-(b0 as f64 - 251.0) * 256.0 - *code.get(i)? as f64 - 108.0);
                    i += 1;
                }
                255 => {
                    st.stack.push(u32_at(code, i)? as i32 as f64 / 65536.0);
                    i += 4;
                }
                1 | 3 | 18 | 23 => {
                    if !st.width_done && st.stack.len() % 2 == 1 {
                        st.stack.remove(0);
                    }
                    st.width_done = true;
                    st.nstems += st.stack.len() / 2;
                    st.stack.clear();
                }
                19 | 20 => {
                    if !st.width_done && st.stack.len() % 2 == 1 {
                        st.stack.remove(0);
                    }
                    st.width_done = true;
                    st.nstems += st.stack.len() / 2;
                    st.stack.clear();
                    i += (st.nstems + 7) / 8;
                }
                21 => {
                    if !st.width_done && st.stack.len() > 2 {
                        st.stack.remove(0);
                    }
                    st.width_done = true;
                    let n = st.stack.len();
                    if n < 2 {
                        return None;
                    }
                    st.move_to(st.stack[n - 2], st.stack[n - 1]);
                    st.stack.clear();
                }
                22 | 4 => {
                    if !st.width_done && st.stack.len() > 1 {
                        st.stack.remove(0);
                    }
                    st.width_done = true;
                    let v = *st.stack.last()?;
                    if b0 == 22 {
                        st.move_to(v, 0.0);
                    } else {
                        st.move_to(0.0, v);
                    }
                    st.stack.clear();
                }
                5 => {
                    let a = std::mem::take(&mut st.stack);
                    for c in a.chunks_exact(2) {
                        st.line(c[0], c[1]);
                    }
                }
                6 | 7 => {
                    let a = std::mem::take(&mut st.stack);
                    let mut horiz = b0 == 6;
                    for &v in &a {
                        if horiz {
                            st.line(v, 0.0);
                        } else {
                            st.line(0.0, v);
                        }
                        horiz = !horiz;
                    }
                }
                8 => {
                    let a = std::mem::take(&mut st.stack);
                    for c in a.chunks_exact(6) {
                        st.curve((c[0], c[1]), (c[2], c[3]), (c[4], c[5]));
                    }
                }
                24 => {
                    let a = std::mem::take(&mut st.stack);
                    if a.len() < 2 {
                        return None;
                    }
                    let nc = (a.len() - 2) / 6 * 6;
                    for c in a[..nc].chunks_exact(6) {
                        st.curve((c[0], c[1]), (c[2], c[3]), (c[4], c[5]));
                    }
                    st.line(a[nc], *a.get(nc + 1)?);
                }
                25 => {
                    let a = std::mem::take(&mut st.stack);
                    if a.len() < 6 {
                        return None;
                    }
                    let nl = (a.len() - 6) / 2 * 2;
                    for c in a[..nl].chunks_exact(2) {
                        st.line(c[0], c[1]);
                    }
                    let c = &a[nl..];
                    st.curve((c[0], c[1]), (c[2], c[3]), (c[4], c[5]));
                }
                26 => {
                    let a = std::mem::take(&mut st.stack);
                    let (mut dx1, mut k) = (0.0, 0);
                    if a.len() % 4 == 1 {
                        dx1 = a[0];
                        k = 1;
                    }
                    for (n, c) in a[k..].chunks_exact(4).enumerate() {
                        st.curve((if n == 0 { dx1 } else { 0.0 }, c[0]), (c[1], c[2]), (0.0, c[3]));
                    }
                }
                27 => {
                    let a = std::mem::take(&mut st.stack);
                    let (mut dy1, mut k) = (0.0, 0);
                    if a.len() % 4 == 1 {
                        dy1 = a[0];
                        k = 1;
                    }
                    for (n, c) in a[k..].chunks_exact(4).enumerate() {
                        st.curve((c[0], if n == 0 { dy1 } else { 0.0 }), (c[1], c[2]), (c[3], 0.0));
                    }
                }
                30 | 31 => {
                    let a = std::mem::take(&mut st.stack);
                    let mut horiz = b0 == 31;
                    let mut k = 0;
                    while a.len() - k >= 4 {
                        let last = a.len() - k == 5;
                        let extra = if last { a[k + 4] } else { 0.0 };
                        if horiz {
                            st.curve((a[k], 0.0), (a[k + 1], a[k + 2]), (extra, a[k + 3]));
                        } else {
                            st.curve((0.0, a[k]), (a[k + 1], a[k + 2]), (a[k + 3], extra));
                        }
                        horiz = !horiz;
                        k += 4;
                    }
                }
                10 | 29 => {
                    let n = st.stack.pop()? as i64;
                    let idx = if b0 == 10 { self.local } else { Some(self.gsubrs) }?;
                    let k = n + bias(idx.count);
                    let sub = idx.get(self.d, usize::try_from(k).ok()?)?;
                    if self.run(sub, depth + 1, st)? {
                        return Some(true);
                    }
                }
                11 => return Some(false),
                14 => {
                    if !st.width_done && (st.stack.len() == 1 || st.stack.len() == 5) {
                        st.stack.remove(0);
                    }
                    // 4 operands left: the old accented-character form (seac) is not composed
                    st.stack.clear();
                    return Some(true);
                }
                12 => {
                    let b1 = *code.get(i)?;
                    i += 1;
                    let a = std::mem::take(&mut st.stack);
                    match b1 {
                        35 if a.len() >= 12 => {
                            st.curve((a[0], a[1]), (a[2], a[3]), (a[4], a[5]));
                            st.curve((a[6], a[7]), (a[8], a[9]), (a[10], a[11]));
                        }
                        34 if a.len() >= 7 => {
                            st.curve((a[0], 0.0), (a[1], a[2]), (a[3], 0.0));
                            st.curve((a[4], 0.0), (a[5], -a[2]), (a[6], 0.0));
                        }
                        36 if a.len() >= 9 => {
                            st.curve((a[0], a[1]), (a[2], a[3]), (a[4], 0.0));
                            st.curve((a[5], 0.0), (a[6], a[7]), (a[8], -(a[1] + a[3] + a[7])));
                        }
                        37 if a.len() >= 11 => {
                            let dx: f64 = a[0] + a[2] + a[4] + a[6] + a[8];
                            let dy: f64 = a[1] + a[3] + a[5] + a[7] + a[9];
                            let last = if dx.abs() > dy.abs() { (a[10], -dy) } else { (-dx, a[10]) };
                            st.curve((a[0], a[1]), (a[2], a[3]), (a[4], a[5]));
                            st.curve((a[6], a[7]), (a[8], a[9]), last);
                        }
                        // arithmetic operators: the few that appear in real fonts
                        9 | 10 | 11 | 12 | 14 | 24 | 18 | 27 | 28 => {
                            // restore the stack and apply
                            st.stack = a;
                            let s = &mut st.stack;
                            match b1 {
                                9 => {
                                    let v = s.pop()?;
                                    s.push(v.abs());
                                }
                                10 | 11 | 24 | 12 => {
                                    let (y, x) = (s.pop()?, s.pop()?);
                                    s.push(match b1 {
                                        10 => x + y,
                                        11 => x - y,
                                        24 => x * y,
                                        _ => {
                                            if y == 0.0 {
                                                return None;
                                            }
                                            x / y
                                        }
                                    });
                                }
                                14 => {
                                    let v = s.pop()?;
                                    s.push(-v);
                                }
                                18 => {
                                    s.pop()?;
                                }
                                27 => {
                                    let v = *s.last()?;
                                    s.push(v);
                                }
                                _ => {
                                    let (y, x) = (s.pop()?, s.pop()?);
                                    s.push(y);
                                    s.push(x);
                                }
                            }
                        }
                        _ => return None,
                    }
                }
                _ => return None,
            }
        }
        Some(false)
    }
}
