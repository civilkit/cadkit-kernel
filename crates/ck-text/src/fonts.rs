//! Fonts by name: the two built-in subsets and the fonts loaded at run time (`loadFont`).
//!
//! The registry is process-wide (one kernel instance = one session): regen has no handle on the request
//! that loaded a font. A loaded font is identified by its name AND the SHA-256 of its file; the hash is
//! part of the regen cache key (`fingerprint`), so replacing the bytes under a name invalidates every
//! cached state that used the font. A text entity may pin the hash (`fontHash`).

use crate::sha256::sha256;
use crate::ttf::{Font, OutlineKind};
use std::sync::{Arc, Mutex, OnceLock};

static GAUGE: &[u8] = include_bytes!("../fonts/gauge.ttf");
static ENGRAVE: &[u8] = include_bytes!("../fonts/engrave.ttf");

pub const FONT_NAMES: [&str; 2] = ["gauge", "engrave"];
/// Names that cannot be loaded over (the built-ins and their aliases).
const RESERVED: [&str; 6] = ["gauge", "engrave", "sans", "serif", "default", ""];
/// Largest accepted font file, bytes.
pub const MAX_FONT_BYTES: usize = 24 << 20;
/// Most fonts loaded at once.
pub const MAX_FONTS: usize = 64;

#[derive(Clone)]
pub struct Entry {
    pub name: String,
    pub hash: [u8; 32],
    pub font: Arc<Font>,
    pub builtin: bool,
}

fn builtins() -> &'static [Entry; 2] {
    static B: OnceLock<[Entry; 2]> = OnceLock::new();
    B.get_or_init(|| {
        let mk = |name: &str, bytes: &'static [u8]| Entry {
            name: name.into(),
            hash: sha256(&[bytes]),
            font: Arc::new(Font::parse(Arc::from(bytes)).expect("built-in font")),
            builtin: true,
        };
        [mk("gauge", GAUGE), mk("engrave", ENGRAVE)]
    })
}

static REG: Mutex<Vec<Entry>> = Mutex::new(Vec::new());

fn reg() -> std::sync::MutexGuard<'static, Vec<Entry>> {
    REG.lock().unwrap_or_else(|e| e.into_inner())
}

pub fn hex(h: &[u8]) -> String {
    h.iter().map(|b| format!("{b:02x}")).collect()
}

fn builtin_named(name: &str) -> Option<&'static Entry> {
    let b = builtins();
    match name {
        "" | "gauge" | "sans" | "default" => Some(&b[0]),
        "engrave" | "serif" => Some(&b[1]),
        _ => None,
    }
}

/// The font behind a name, built-in or loaded.
pub fn entry(name: &str) -> Option<Entry> {
    builtin_named(name).cloned().or_else(|| reg().iter().find(|e| e.name == name).cloned())
}

/// The font behind a name (`gauge`, `engrave`, their aliases, or a loaded font).
pub fn font(name: &str) -> Option<Arc<Font>> {
    entry(name).map(|e| e.font)
}

/// What the regen cache key must contain for a font reference: the content hash, or `missing`.
pub fn fingerprint(name: &str, pin: Option<&str>) -> String {
    let h = entry(name).map_or("missing".to_string(), |e| hex(&e.hash));
    format!("{name}={h}{}", pin.map(|p| format!("^{p}")).unwrap_or_default())
}

/// Resolve a font reference of a text entity.
pub fn resolve(name: &str, pin: Option<&str>) -> Result<Arc<Font>, crate::Error> {
    let Some(e) = entry(name) else { return Err(crate::Error::FontNotLoaded(name.to_string())) };
    if let Some(p) = pin.filter(|p| !p.is_empty()) {
        let h = hex(&e.hash);
        if !h.starts_with(&p.to_ascii_lowercase()) {
            return Err(crate::Error::FontMismatch { name: name.to_string(), pinned: p.to_string(), loaded: h });
        }
    }
    Ok(e.font)
}

#[derive(Clone, Debug)]
pub struct Warn {
    pub code: &'static str,
    pub detail: String,
}

#[derive(Clone, Debug)]
pub struct Info {
    pub name: String,
    pub hash: String,
    pub builtin: bool,
    pub bytes: usize,
    pub glyphs: usize,
    pub chars: usize,
    pub units_per_em: f64,
    pub cap_height: f64,
    pub outlines: &'static str,
    pub family: Option<String>,
    /// Unicode blocks with at least one mapped character and the count (`coverage`), in code order.
    pub coverage: Vec<(&'static str, usize, usize)>,
    pub warnings: Vec<Warn>,
    /// The same name was already loaded with the same file; nothing changed.
    pub already_loaded: bool,
    /// The name held another file, which was replaced (`replace: true`).
    pub replaced: bool,
}

/// A failed `loadFont` / `unloadFont` / `subsetFont`: `code` is the wire error code.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct FontError {
    pub code: &'static str,
    pub message: String,
}

fn ferr<T>(code: &'static str, message: String) -> Result<T, FontError> {
    Err(FontError { code, message })
}

const BLOCKS: [(&str, u32, u32); 15] = [
    ("Basic Latin", 0x20, 0x7E),
    ("Latin-1 Supplement", 0xA0, 0xFF),
    ("Latin Extended-A", 0x100, 0x17F),
    ("Latin Extended-B", 0x180, 0x24F),
    ("Greek", 0x370, 0x3FF),
    ("Cyrillic", 0x400, 0x4FF),
    ("General Punctuation", 0x2000, 0x206F),
    ("Currency Symbols", 0x20A0, 0x20CF),
    ("Letterlike Symbols", 0x2100, 0x214F),
    ("Arrows", 0x2190, 0x21FF),
    ("Mathematical Operators", 0x2200, 0x22FF),
    ("Hiragana", 0x3040, 0x309F),
    ("Katakana", 0x30A0, 0x30FF),
    ("CJK Unified Ideographs", 0x4E00, 0x9FFF),
    ("Hangul Syllables", 0xAC00, 0xD7AF),
];

fn info_of(e: &Entry) -> Info {
    let f = &e.font;
    let chars = f.mapped_chars();
    let coverage: Vec<(&'static str, usize, usize)> = BLOCKS
        .iter()
        .filter_map(|&(n, lo, hi)| {
            let m = chars.iter().filter(|&&c| (lo..=hi).contains(&(c as u32))).count();
            (m > 0).then_some((n, m, (hi - lo + 1) as usize))
        })
        .collect();
    let mut warnings = vec![];
    let nt = &f.notes;
    if nt.variable {
        warnings.push(Warn { code: "fontVariableDefaultOnly", detail: "variable font: only the default instance is read (no weight / width axes)".into() });
    }
    if nt.kerning {
        warnings.push(Warn { code: "fontKerningIgnored", detail: "the font carries pair kerning (kern / GPOS): applied only when the text sets kern: true; plain advance widths otherwise".into() });
    }
    if nt.substitution {
        warnings.push(Warn { code: "fontLigaturesIgnored", detail: "the font carries GSUB: standard ligatures apply only when the text sets liga: true; other substitutions are ignored".into() });
    }
    if nt.collection {
        warnings.push(Warn { code: "fontCollectionFirstFace", detail: "font collection: only the first face is used".into() });
    }
    if nt.cap_height_estimated {
        warnings.push(Warn { code: "fontNoCapHeight", detail: "the font has no letter H; the cap height comes from OS/2 or is estimated (0.7 em)".into() });
    }
    if f.kind() == OutlineKind::Cff {
        warnings.push(Warn { code: "fontCffApproximated", detail: "CFF cubic curves are converted to quadratic B-splines within 0.05 % of the em; the loops have more edges than a TrueType font".into() });
    }
    if let Some(t) = f.fs_type() {
        if t & 0x0002 != 0 {
            warnings.push(Warn { code: "fontEmbeddingRestricted", detail: "the font's licence flag (OS/2 fsType) restricts embedding; do not embed it in documents".into() });
        } else if t & 0x0100 != 0 {
            warnings.push(Warn { code: "fontNoSubsetting", detail: "the font's licence flag (OS/2 fsType) forbids subsetting".into() });
        }
    }
    let ov = crate::overlapping_glyphs(f);
    if !ov.is_empty() {
        warnings.push(Warn {
            code: "fontOverlappingContours",
            detail: format!(
                "the outlines of some letters cross themselves ({}); set merge: true on the text to union them, or expect seams or boolean failures on engraved or embossed letters",
                ov.iter().collect::<String>()
            ),
        });
    }
    if !chars.iter().any(|c| c.is_ascii_alphabetic()) {
        warnings.push(Warn { code: "fontNoLatin", detail: "the font maps no Latin letters; most strings will give textMissingGlyph".into() });
    }
    Info {
        name: e.name.clone(),
        hash: hex(&e.hash),
        builtin: e.builtin,
        bytes: f.bytes().len(),
        glyphs: f.num_glyphs(),
        chars: chars.len(),
        units_per_em: f.units_per_em,
        cap_height: f.cap_height(),
        outlines: if f.kind() == OutlineKind::Cff { "cff" } else { "glyf" },
        family: f.display_name(),
        coverage,
        warnings,
        already_loaded: false,
        replaced: false,
    }
}

fn check_name(name: &str) -> Result<(), FontError> {
    if name.is_empty() || name.chars().count() > 64 || name.chars().any(|c| c.is_control()) || name != name.trim() {
        return ferr("fontBadName", format!("font name {name:?} must be 1 to 64 characters, no control characters, no leading or trailing blanks"));
    }
    if RESERVED.contains(&name) {
        return ferr("fontReserved", format!("{name:?} is a built-in font name; choose another name for the loaded font"));
    }
    Ok(())
}

/// Register a font under `name` for the session. Same name and same file: no change. Same name, other
/// file: `fontNameConflict` unless `replace`.
pub fn load(name: &str, bytes: Vec<u8>, replace: bool) -> Result<Info, FontError> {
    check_name(name)?;
    if bytes.is_empty() {
        return ferr("fontBadData", "the font data is empty".into());
    }
    if bytes.len() > MAX_FONT_BYTES {
        return ferr("fontTooLarge", format!("the font file is {} bytes; the limit is {MAX_FONT_BYTES}", bytes.len()));
    }
    let hash = sha256(&[&bytes]);
    let mut r = reg();
    let mut replaced = false;
    if let Some(i) = r.iter().position(|e| e.name == name) {
        if r[i].hash == hash {
            let mut info = info_of(&r[i]);
            info.already_loaded = true;
            return Ok(info);
        }
        if !replace {
            return ferr(
                "fontNameConflict",
                format!("a different font is already loaded as {name:?} (sha256 {}, new file {}); unload it first or pass replace:true", &hex(&r[i].hash)[..16], &hex(&hash)[..16]),
            );
        }
        r.remove(i);
        replaced = true;
    } else if r.len() >= MAX_FONTS {
        return ferr("fontTooMany", format!("{MAX_FONTS} fonts are loaded; unload some first"));
    }
    let font = Font::parse(Arc::from(bytes)).map_err(|m| FontError { code: "fontBadData", message: format!("{name}: {m}") })?;
    let e = Entry { name: name.to_string(), hash, font: Arc::new(font), builtin: false };
    let mut info = info_of(&e);
    info.replaced = replaced;
    r.push(e);
    Ok(info)
}

/// Remove a loaded font; `Ok(false)` when nothing was loaded under the name.
pub fn unload(name: &str) -> Result<bool, FontError> {
    if builtin_named(name).is_some() {
        return ferr("fontReserved", format!("{name:?} is a built-in font and cannot be unloaded"));
    }
    let mut r = reg();
    let n = r.len();
    r.retain(|e| e.name != name);
    Ok(r.len() != n)
}

/// The built-in fonts, then the loaded ones in load order.
pub fn list() -> Vec<Info> {
    let mut v: Vec<Info> = builtins().iter().map(info_of).collect();
    v.extend(reg().iter().map(info_of));
    v
}

/// TrueType bytes of a loaded (or built-in) font reduced to the glyphs of `text`.
pub fn subset(name: &str, text: &str) -> Result<(Vec<u8>, Vec<Warn>), FontError> {
    let Some(e) = entry(name) else {
        return ferr("textFontNotLoaded", format!("font {name:?} is not loaded; load it with {{type:\"loadFont\", name:{name:?}, data:<base64 TTF/OTF>}}"));
    };
    let chars: Vec<char> = text.chars().collect();
    let missing: String = {
        let mut m = String::new();
        for &c in &chars {
            if !c.is_whitespace() && e.font.glyph_index(c).is_none() && !m.contains(c) {
                m.push(c);
            }
        }
        m
    };
    let bytes = e.font.subset(&chars).map_err(|m| FontError { code: "fontSubsetFailed", message: format!("{name}: {m}") })?;
    let mut w = vec![];
    if !missing.is_empty() {
        w.push(Warn { code: "textMissingGlyph", detail: format!("no glyph for {missing:?} in font {name}; not in the subset") });
    }
    if let Some(t) = e.font.fs_type() {
        if t & 0x0002 != 0 || t & 0x0100 != 0 {
            w.push(Warn { code: "fontEmbeddingRestricted", detail: "the font's licence flag (OS/2 fsType) restricts embedding or subsetting".into() });
        }
    }
    Ok((bytes, w))
}

// ---------------------------------------------------------------------------------------------
// base64

const B64: &[u8; 64] = b"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

pub fn base64_encode(b: &[u8]) -> String {
    let mut s = String::with_capacity(b.len() / 3 * 4 + 4);
    for c in b.chunks(3) {
        let n = (c[0] as u32) << 16 | (*c.get(1).unwrap_or(&0) as u32) << 8 | *c.get(2).unwrap_or(&0) as u32;
        for i in 0..4 {
            if i <= c.len() {
                s.push(B64[(n >> (18 - 6 * i) & 63) as usize] as char);
            } else {
                s.push('=');
            }
        }
    }
    s
}

/// Standard base64; also url-safe letters, blanks, and a `data:...;base64,` prefix.
pub fn base64_decode(s: &str) -> Result<Vec<u8>, String> {
    let s = s.find("base64,").map_or(s, |i| &s[i + 7..]);
    let mut out = Vec::with_capacity(s.len() / 4 * 3);
    let (mut acc, mut nb) = (0u32, 0u32);
    for c in s.bytes() {
        let v = match c {
            b'A'..=b'Z' => c - b'A',
            b'a'..=b'z' => c - b'a' + 26,
            b'0'..=b'9' => c - b'0' + 52,
            b'+' | b'-' => 62,
            b'/' | b'_' => 63,
            b'=' | b' ' | b'\n' | b'\r' | b'\t' => continue,
            _ => return Err(format!("data is not base64 (byte 0x{c:02x})")),
        };
        acc = (acc << 6) | v as u32;
        nb += 6;
        if nb >= 8 {
            nb -= 8;
            out.push((acc >> nb) as u8);
            acc &= (1 << nb) - 1;
        }
    }
    Ok(out)
}
