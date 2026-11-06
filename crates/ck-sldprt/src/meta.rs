//! What a SolidWorks part file holds beside its body, read from plain streams: the feature list (`swXmlContents/KeyWords`),
//! the configurations, the material and its density (`swXmlContents/MATERIALTREE`), custom properties, units and the stored
//! mass properties (`docProps/*.xml` in the modern container, OLE property sets in OLE2 files), the SolidWorks version
//! (`_MO_VERSION_<n>` streams) and the PNG preview (`PreviewPNG`). Every stream is optional: a missing or damaged one leaves its
//! field empty, it never fails the import.
//!
//! Privacy: these files also record Windows user names (last saved by, author) and absolute paths (the folder the part was
//! saved in, the save history). Those streams and fields are not read into [`SwMeta`] at all, and every string that is kept
//! is checked: a value that looks like an absolute path, or that contains one of the user names the file records, is left
//! out and counted in [`SwMeta::omitted`].

use crate::container::Container;
use crate::error::Limits;

/// The mass properties SolidWorks stored in the file (property `SW-MassProp-Config-0`), in the SI units as stored: centre of
/// mass (m), volume (m^3, six decimals), surface area (m^2, six decimals, i.e. 1 mm^2 resolution) and mass (kg). An
/// independent check of the imported body.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SwMassProps {
    pub com_m: [f64; 3],
    pub volume_m3: f64,
    pub area_m2: f64,
    pub mass_kg: f64,
}

/// A feature dimension as SolidWorks displays it (`text`, display units, e.g. `R1`, `360°`, `Ø20`) and its number.
#[derive(Debug, Clone, PartialEq)]
pub struct SwDim {
    pub name: String,
    pub text: String,
    pub value: Option<f64>,
}

/// One entry of the feature tree.
#[derive(Debug, Clone, PartialEq)]
pub struct SwFeature {
    /// SolidWorks feature id; the faces a feature built carry it (`FACE_ID_2001`).
    pub id: i64,
    pub name: String,
    /// Feature type (`Boss-Extrude`, `Fillet`, `HoleWizard`, `Sketch`, `Plane`, ...).
    pub kind: String,
    pub dims: Vec<SwDim>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct SwConfig {
    pub name: String,
    pub material: Option<String>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct SwMaterial {
    pub name: String,
    /// Mass density, kg/m^3 (`DENS` of the material's physical properties).
    pub density_kg_m3: Option<f64>,
}

/// Document units (SolidWorks API enum codes, named).
#[derive(Debug, Clone, PartialEq)]
pub struct SwUnits {
    /// `MMGS`, `CGS`, `MKS`, `IPS` or `custom`.
    pub system: Option<String>,
    /// Display length unit: `mm`, `cm`, `m`, `in`, `ft`, ...
    pub length: Option<String>,
    /// Display angle unit: `deg`, `rad`, ...
    pub angle: Option<String>,
    pub length_decimals: Option<i64>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct SwThumbnail {
    pub png: Vec<u8>,
    pub width: u32,
    pub height: u32,
}

#[derive(Debug, Clone, Default, PartialEq)]
pub struct SwMeta {
    /// The feature tree in id order, without the fixed folders (annotations, lights, bodies, equations, ...).
    pub features: Vec<SwFeature>,
    pub configurations: Vec<SwConfig>,
    pub material: Option<SwMaterial>,
    /// User-defined file properties (name, value).
    pub custom_properties: Vec<(String, String)>,
    /// User-defined properties of the first configuration (name, value).
    pub config_properties: Vec<(String, String)>,
    pub units: Option<SwUnits>,
    pub mass: Option<SwMassProps>,
    /// The `_MO_VERSION_<n>` number of the file (4700 = SolidWorks 2011, 9000 = 2016, 15000 = 2022).
    pub sw_version: Option<u32>,
    pub thumbnail: Option<SwThumbnail>,
    /// Strings left out because they looked like a path or held a user name.
    pub omitted: usize,
}

impl SwMeta {
    /// SolidWorks release year from [`SwMeta::sw_version`] (2007 + n / 1000, checked on 2011 to 2022 files).
    pub fn sw_release(&self) -> Option<u32> {
        self.sw_version.filter(|&v| v >= 4000).map(|v| 2007 + v / 1000)
    }

    /// The feature with SolidWorks id `id`.
    pub fn feature(&self, id: i64) -> Option<&SwFeature> {
        self.features.iter().find(|f| f.id == id)
    }
}

/// Read the metadata of an opened container.
pub fn read(c: &Container, limits: &Limits) -> SwMeta {
    let text = |name: &str| c.data(name, limits).map(|d| String::from_utf8_lossy(&d).into_owned());
    // file properties: XML in the modern container, OLE property sets in OLE2
    let (info, custom, config, users) = if c.stream("docProps/ISolidWorksInformation.xml").is_some() || c.stream("docProps/custom.xml").is_some() {
        let info = text("docProps/ISolidWorksInformation.xml").map(|t| xml_props(&t, "")).unwrap_or_default();
        let custom = text("docProps/custom.xml").map(|t| xml_props(&t, "UserDefinedProperties")).unwrap_or_default();
        let config = text("docProps/Config-0-Properties.xml").map(|t| xml_props(&t, "UserDefinedProperties")).unwrap_or_default();
        let mut users: Vec<String> = vec![];
        if let Some(core) = text("docProps/core.xml") {
            users.extend(["dc:creator", "cp:lastModifiedBy"].iter().filter_map(|t| xml_element(&core, t)));
        }
        (info, custom, config, users)
    } else {
        let ps = |name: &str, fmtid: &[u8; 16]| c.stream(name).map(|s| ole_props(&s.data, fmtid)).unwrap_or_default();
        let named = |v: Vec<(u32, Option<String>, String)>| v.into_iter().filter_map(|(_, n, v)| Some((n?, v))).collect::<Vec<_>>();
        let summary = ps("SummaryInformation", &FMTID_SUMMARY);
        // author (4) and last saved by (8)
        let users: Vec<String> = summary.into_iter().filter(|p| p.0 == 4 || p.0 == 8).map(|p| p.2).collect();
        (named(ps("ISolidWorksInformation", &FMTID_USER)), named(ps("DocumentSummaryInformation", &FMTID_USER)), named(ps("Config-0-Properties", &FMTID_USER)), users)
    };
    let prop = |name: &str| info.iter().find(|p| p.0 == name).map(|p| p.1.as_str());
    let mut scrub = Scrub::new(users.into_iter().chain(["SW-Author", "SW-Last Saved By"].iter().filter_map(|n| prop(n).map(str::to_string))));
    let mut m = SwMeta::default();

    // feature tree, configurations and their material
    if let Some(kw) = c.data("swXmlContents/KeyWords", limits) {
        let t = String::from_utf8_lossy(&kw);
        let (mut features, configs) = keywords(&t);
        crate::classes::canonicalize(c, limits, &mut features);
        for mut f in features {
            f.name = scrub.keep(f.name).unwrap_or_default();
            f.dims.retain_mut(|d| scrub.ok(&d.name) && scrub.ok(&d.text) || scrub.drop());
            m.features.push(f);
        }
        for cf in configs {
            let Some(name) = scrub.keep(cf.name) else { continue };
            let material = cf.material.filter(|x| !x.contains("not specified")).and_then(|x| scrub.keep(x));
            m.configurations.push(SwConfig { name, material });
        }
    }
    let tree = c.data("swXmlContents/MATERIALTREE", limits).map(|d| utf16_or_8(&d));
    let wanted = m.configurations.first().and_then(|c| c.material.clone());
    m.material = match tree.as_deref().and_then(|t| material_tree(t, wanted.as_deref())) {
        Some(mut mt) => scrub.keep(mt.name.clone()).map(|n| {
            mt.name = n;
            mt
        }),
        None => wanted.map(|name| SwMaterial { name, density_kg_m3: None }),
    };

    let props = |v: Vec<(String, String)>, scrub: &mut Scrub| -> Vec<(String, String)> {
        v.into_iter()
            .filter(|(n, _)| !n.is_empty() && !n.starts_with("SW-"))
            .filter_map(|(n, v)| if scrub.ok(&n) && scrub.ok(&v) { Some((n.trim().to_string(), v.trim().to_string())) } else { scrub.drop_none() })
            .collect()
    };
    m.custom_properties = props(custom, &mut scrub);
    m.config_properties = props(config, &mut scrub);

    let int = |n: &str| prop(n).and_then(|v| v.trim().parse::<i64>().ok());
    let pick = |n: &str, names: &[&str]| int(n).map(|k| names.get(k as usize).filter(|s| !s.is_empty()).map_or_else(|| format!("code {k}"), |s| s.to_string()));
    let units = SwUnits {
        system: pick("SW_UnitSystem", &["", "CGS", "MKS", "IPS", "custom", "MMGS"]),
        length: pick("SW_UnitsLinear", &["mm", "cm", "m", "in", "ft", "ft-in", "angstrom", "nm", "um", "mil", "uin"]),
        angle: pick("SW_UnitsAngular", &["deg", "deg-min", "deg-min-sec", "rad"]),
        length_decimals: int("SW_UnitsLinearDecimalPlaces"),
    };
    if units.system.is_some() || units.length.is_some() || units.angle.is_some() {
        m.units = Some(units);
    }
    m.mass = prop("SW-MassProp-Config-0").and_then(mass_props);

    m.sw_version = c.streams.iter().filter_map(|s| s.name.strip_prefix("_MO_VERSION_")).filter_map(|r| r.split('/').next()?.parse::<u32>().ok()).max();
    m.thumbnail = c.stream("PreviewPNG").and_then(|s| png(&s.data));
    m.omitted = scrub.omitted;
    m
}

fn mass_props(v: &str) -> Option<SwMassProps> {
    let v: Vec<f64> = v.split(',').filter_map(|x| x.trim().parse().ok()).collect();
    if v.len() < 6 || v.iter().any(|x| !x.is_finite()) {
        return None;
    }
    Some(SwMassProps { com_m: [v[0], v[1], v[2]], volume_m3: v[3], area_m2: v[4], mass_kg: v[5] })
}

/// The preview PNG with its text and time chunks (`tEXt`, `zTXt`, `iTXt`, `eXIf`, `tIME`) removed, so no metadata rides
/// along with the image.
fn png(d: &[u8]) -> Option<SwThumbnail> {
    if d.len() < 24 || d[..8] != [0x89, b'P', b'N', b'G', 0x0D, 0x0A, 0x1A, 0x0A] || &d[12..16] != b"IHDR" {
        return None;
    }
    let be = |p: usize| u32::from_be_bytes([d[p], d[p + 1], d[p + 2], d[p + 3]]);
    let mut out = d[..8].to_vec();
    let mut p = 8usize;
    while p + 12 <= d.len() {
        let end = p.checked_add(12 + be(p) as usize).filter(|&e| e <= d.len())?;
        let ty = &d[p + 4..p + 8];
        if ![&b"tEXt"[..], b"zTXt", b"iTXt", b"eXIf", b"tIME"].contains(&ty) {
            out.extend_from_slice(&d[p..end]);
        }
        p = end;
        if ty == b"IEND" {
            break;
        }
    }
    Some(SwThumbnail { png: out, width: be(16), height: be(20) })
}

// ------------------------------------------------------------------ privacy

struct Scrub {
    /// User names recorded in the file, lower case.
    users: Vec<String>,
    omitted: usize,
}

impl Scrub {
    fn new(users: impl Iterator<Item = String>) -> Scrub {
        let mut u: Vec<String> = users.map(|s| s.trim().to_lowercase()).filter(|s| !s.is_empty()).collect();
        u.sort();
        u.dedup();
        Scrub { users: u, omitted: 0 }
    }

    /// `s` may be output: it is not an absolute path and holds no recorded user name.
    fn ok(&self, s: &str) -> bool {
        let l = s.to_lowercase();
        !looks_like_path(&l) && !self.users.iter().any(|u| if u.chars().count() >= 3 { l.contains(u.as_str()) } else { l.trim() == u })
    }

    fn keep(&mut self, s: String) -> Option<String> {
        if self.ok(&s) {
            Some(s)
        } else {
            self.omitted += 1;
            None
        }
    }

    fn drop(&mut self) -> bool {
        self.omitted += 1;
        false
    }

    fn drop_none<T>(&mut self) -> Option<T> {
        self.omitted += 1;
        None
    }
}

/// An absolute path (lower-cased input): a drive letter (`c:\`, `d:/`), a UNC prefix or a home directory.
fn looks_like_path(l: &str) -> bool {
    let b = l.as_bytes();
    let drive = (0..b.len().saturating_sub(2)).any(|i| b[i].is_ascii_alphabetic() && b[i + 1] == b':' && (b[i + 2] == b'\\' || b[i + 2] == b'/') && (i == 0 || !b[i - 1].is_ascii_alphanumeric()));
    drive || l.contains("\\\\") || ["\\users\\", "/users/", "/home/", "~/", "\\documents and settings\\"].iter().any(|p| l.contains(p))
}

// ------------------------------------------------------------------ KeyWords

/// Folders and fixed entries of every feature tree, not listed as features.
const NOT_FEATURES: &[&str] = &[
    "Annotations", "Surface Bodies", "Solid Bodies", "Ambient", "Directional", "Spot", "Point", "Equations", "Notes", "Exploded Views",
    "Markups", "Sensors", "Favorites", "History", "Selection Sets", "Lights, Cameras and Scene", "Design Binder", "Comments",
    "Detail Folder", "SOLIDWORKS Materials", "SolidWorks Materials", "Attribute", "Attribute-Definition", "Model Break Views",
    "Sheet-Metal Master", "Folder",
];

/// The features and configurations of a `KeyWords` stream: a flat list of elements `<Tag id=".." Name=".." Type="..">` with
/// `<Dimension Name="D1">10</Dimension>` children.
fn keywords(t: &str) -> (Vec<SwFeature>, Vec<SwConfig>) {
    let mut feats: Vec<SwFeature> = vec![];
    let mut configs = vec![];
    // the open feature element: (tag, index in feats)
    let mut open: Option<(String, usize)> = None;
    let mut i = 0usize;
    while let Some(k) = t[i..].find('<') {
        let at = i + k;
        let Some((tag, attrs, self_closing, end)) = element(t, at) else { break };
        i = end;
        if let Some(name) = tag.strip_prefix('/') {
            if open.as_ref().map_or(false, |o| o.0 == name) {
                open = None;
            }
            continue;
        }
        if tag.starts_with('?') || tag.starts_with('!') {
            continue;
        }
        let get = |k: &str| attrs.iter().find(|a| a.0 == k).map(|a| a.1.clone());
        if tag == "Dimension" {
            let (Some((_, fi)), Some(name)) = (&open, get("Name")) else { continue };
            let close = t[i..].find('<').map_or(t.len(), |p| i + p);
            let text = unescape(&t[i..close]).replace("<MOD-DIAM>", "Ø").trim().to_string();
            let value = first_number(&text);
            feats[*fi].dims.push(SwDim { name, text, value });
            continue;
        }
        if tag == "Configuration" {
            if let Some(name) = get("Name") {
                configs.push(SwConfig { name, material: get("Material") });
            }
            continue;
        }
        let Some(id) = get("id").and_then(|v| v.parse::<i64>().ok()) else { continue };
        if tag == "Keywords" {
            continue; // the document itself (its name is the file name)
        }
        let ty = get("Type").filter(|t| !t.is_empty() && !t.contains('<'));
        let kind = ty.unwrap_or_else(|| tag.clone());
        if NOT_FEATURES.contains(&kind.as_str()) {
            continue;
        }
        feats.push(SwFeature { id, name: get("Name").unwrap_or_default(), kind, dims: vec![] });
        if !self_closing {
            open = Some((tag, feats.len() - 1));
        }
    }
    feats.sort_by_key(|f| f.id);
    (feats, configs)
}

/// The element starting at `at` (`<`): (tag, attributes, self-closing, index after `>`).
fn element(t: &str, at: usize) -> Option<(String, Vec<(String, String)>, bool, usize)> {
    let b = t.as_bytes();
    let mut j = at + 1;
    let mut quote: Option<u8> = None;
    while j < b.len() {
        match (quote, b[j]) {
            (None, b'"') | (None, b'\'') => quote = Some(b[j]),
            (Some(q), c) if c == q => quote = None,
            (None, b'>') => break,
            _ => {}
        }
        j += 1;
    }
    if j >= b.len() {
        return None;
    }
    let inner = &t[at + 1..j];
    let self_closing = inner.ends_with('/');
    let inner = inner.trim_end_matches('/');
    let tag_end = inner.find(|c: char| c.is_whitespace()).unwrap_or(inner.len());
    let tag = inner[..tag_end].to_string();
    let mut attrs = vec![];
    let mut rest = &inner[tag_end..];
    while let Some(eq) = rest.find('=') {
        let key = rest[..eq].trim().to_string();
        let after = rest[eq + 1..].trim_start();
        let Some(q) = after.chars().next().filter(|c| *c == '"' || *c == '\'') else { break };
        let Some(close) = after[1..].find(q) else { break };
        attrs.push((key, unescape(&after[1..1 + close])));
        rest = &after[close + 2..];
    }
    Some((tag, attrs, self_closing, j + 1))
}

fn unescape(s: &str) -> String {
    if !s.contains('&') {
        return s.to_string();
    }
    let mut out = String::with_capacity(s.len());
    let mut rest = s;
    while let Some(p) = rest.find('&') {
        out.push_str(&rest[..p]);
        rest = &rest[p..];
        let Some(semi) = rest.as_bytes()[..rest.len().min(12)].iter().position(|&c| c == b';') else {
            out.push('&');
            rest = &rest[1..];
            continue;
        };
        let ent = &rest[1..semi];
        let ch = match ent {
            "lt" => Some('<'),
            "gt" => Some('>'),
            "amp" => Some('&'),
            "quot" => Some('"'),
            "apos" => Some('\''),
            _ => ent
                .strip_prefix("#x")
                .or_else(|| ent.strip_prefix("#X"))
                .and_then(|h| u32::from_str_radix(h, 16).ok())
                .or_else(|| ent.strip_prefix('#').and_then(|d| d.parse().ok()))
                .and_then(char::from_u32),
        };
        match ch {
            Some(c) => {
                out.push(c);
                rest = &rest[semi + 1..];
            }
            None => {
                out.push('&');
                rest = &rest[1..];
            }
        }
    }
    out.push_str(rest);
    out
}

/// The first decimal number in `s` (`R1` -> 1, `360°` -> 360, `Ø20` -> 20, `-2.5e-3` -> -0.0025).
fn first_number(s: &str) -> Option<f64> {
    let b = s.as_bytes();
    let start = (0..b.len()).find(|&i| b[i].is_ascii_digit() || (b[i] == b'.' && b.get(i + 1).map_or(false, u8::is_ascii_digit)))?;
    let start = if start > 0 && b[start - 1] == b'-' { start - 1 } else { start };
    let mut end = start + 1;
    while end < b.len() && (b[end].is_ascii_digit() || b[end] == b'.') {
        end += 1;
    }
    if end < b.len() && (b[end] == b'e' || b[end] == b'E') {
        let mut e = end + 1;
        if e < b.len() && (b[e] == b'-' || b[e] == b'+') {
            e += 1;
        }
        if e < b.len() && b[e].is_ascii_digit() {
            while e < b.len() && b[e].is_ascii_digit() {
                e += 1;
            }
            end = e;
        }
    }
    s[start..end].parse().ok().filter(|v: &f64| v.is_finite())
}

// ------------------------------------------------------------------ material

fn utf16_or_8(d: &[u8]) -> String {
    if d.len() >= 2 && d[..2] == [0xFF, 0xFE] {
        let u: Vec<u16> = d[2..].chunks_exact(2).map(|c| u16::from_le_bytes([c[0], c[1]])).collect();
        String::from_utf16_lossy(&u)
    } else {
        String::from_utf8_lossy(d).into_owned()
    }
}

/// The material of a `MATERIALTREE` stream: the one named `wanted` (library prefix `Library|` ignored), else the first.
fn material_tree(t: &str, wanted: Option<&str>) -> Option<SwMaterial> {
    let mut found: Vec<SwMaterial> = vec![];
    let mut i = 0usize;
    while let Some(k) = t[i..].find("<material ") {
        let at = i + k;
        let (_, attrs, _, end) = element(t, at)?;
        i = end;
        let Some(full) = attrs.iter().find(|a| a.0 == "name").map(|a| a.1.clone()) else { continue };
        let name = full.rsplit('|').next().unwrap_or(&full).trim().to_string();
        let close = t[i..].find("</material>").map_or(t.len(), |p| i + p);
        let body = &t[i..close];
        let density = body.find("<DENS ").and_then(|p| element(body, p)).and_then(|(_, a, _, _)| a.iter().find(|x| x.0 == "value").and_then(|x| x.1.trim().parse::<f64>().ok())).filter(|d| d.is_finite() && *d > 0.0);
        found.push(SwMaterial { name, density_kg_m3: density });
    }
    let w = wanted.map(str::trim);
    let k = found.iter().position(|m| Some(m.name.as_str()) == w).unwrap_or(0);
    (k < found.len()).then(|| found.swap_remove(k))
}

// ------------------------------------------------------------------ properties

/// `<property name=".." pid=".."><vt:TYPE>value</vt:TYPE></property>` entries of the `<propertySection name="SECTION">`
/// sections of a docProps XML stream (`section` empty: every section).
fn xml_props(t: &str, section: &str) -> Vec<(String, String)> {
    let mut out = vec![];
    let mut in_section = section.is_empty();
    let mut i = 0usize;
    while let Some(k) = t[i..].find('<') {
        let Some((tag, attrs, self_closing, end)) = element(t, i + k) else { break };
        i = end;
        match tag.as_str() {
            "propertySection" if !section.is_empty() => in_section = attrs.iter().any(|a| a.0 == "name" && a.1 == section),
            "property" if in_section && !self_closing => {
                let Some(name) = attrs.iter().find(|a| a.0 == "name").map(|a| a.1.clone()) else { continue };
                // the value element
                let Some(v) = t[i..].find('<').map(|p| i + p) else { break };
                let Some((vt, _, vclosed, vend)) = element(t, v) else { break };
                if !vt.starts_with("vt:") || vclosed {
                    continue;
                }
                let close = t[vend..].find('<').map_or(t.len(), |p| vend + p);
                out.push((name, unescape(&t[vend..close])));
                i = close;
            }
            _ => {}
        }
    }
    out
}

/// The text of the first element `<tag ...>text</tag>`.
fn xml_element(t: &str, tag: &str) -> Option<String> {
    let at = t.find(&format!("<{tag}>")).or_else(|| t.find(&format!("<{tag} ")))?;
    let (_, _, closed, end) = element(t, at)?;
    if closed {
        return None;
    }
    let close = t[end..].find('<').map_or(t.len(), |p| end + p);
    Some(unescape(&t[end..close]))
}

/// Section FMTIDs as stored (little-endian GUIDs): SummaryInformation and the user-defined properties section.
const FMTID_SUMMARY: [u8; 16] = [0xE0, 0x85, 0x9F, 0xF2, 0xF9, 0x4F, 0x68, 0x10, 0xAB, 0x91, 0x08, 0x00, 0x2B, 0x27, 0xB3, 0xD9];
const FMTID_USER: [u8; 16] = [0x05, 0xD5, 0xCD, 0xD5, 0x9C, 0x2E, 0x1B, 0x10, 0x93, 0x97, 0x08, 0x00, 0x2B, 0x2C, 0xF9, 0xAE];

/// The properties of section `fmtid` of an OLE property set stream: (property id, dictionary name, value as text). Strings,
/// integers, reals and booleans are read; other types are skipped. Names have SolidWorks' `(Display Name)` suffix removed.
fn ole_props(d: &[u8], fmtid: &[u8; 16]) -> Vec<(u32, Option<String>, String)> {
    let u16at = |p: usize| d.get(p..p + 2).map(|b| u16::from_le_bytes([b[0], b[1]]));
    let u32at = |p: usize| d.get(p..p + 4).map(|b| u32::from_le_bytes([b[0], b[1], b[2], b[3]]));
    let mut out = vec![];
    if u16at(0) != Some(0xFFFE) {
        return out;
    }
    let nsets = u32at(24).unwrap_or(0).min(16) as usize;
    let Some(base) = (0..nsets).find(|&s| d.get(28 + 20 * s..44 + 20 * s) == Some(&fmtid[..])).and_then(|s| u32at(44 + 20 * s)).map(|o| o as usize) else { return out };
    let n = u32at(base + 4).unwrap_or(0).min(10_000) as usize;
    let mut entries: Vec<(u32, usize)> = (0..n).filter_map(|k| Some((u32at(base + 8 + 8 * k)?, base + u32at(base + 12 + 8 * k)? as usize))).collect();
    let cp = entries.iter().find(|e| e.0 == 1).and_then(|e| u16at(e.1 + 4)).unwrap_or(1252);
    let utf8 = cp == 0xFDE9; // 65001
    let text = |b: &[u8]| -> String {
        let b = &b[..b.iter().position(|&c| c == 0).unwrap_or(b.len())];
        if utf8 { String::from_utf8_lossy(b).into_owned() } else { b.iter().map(|&c| c as char).collect() }
    };
    let mut names: Vec<(u32, String)> = vec![];
    if let Some(pos) = entries.iter().position(|e| e.0 == 0) {
        let p = entries.remove(pos).1;
        let mut q = p + 4;
        for _ in 0..u32at(p).unwrap_or(0).min(10_000) {
            let (Some(pid), Some(len)) = (u32at(q), u32at(q + 4)) else { break };
            let len = len as usize;
            let raw = if cp == 1200 { len * 2 } else { len };
            let Some(b) = d.get(q + 8..q + 8 + raw) else { break };
            let name = if cp == 1200 {
                let u: Vec<u16> = b.chunks_exact(2).map(|c| u16::from_le_bytes([c[0], c[1]])).take_while(|&c| c != 0).collect();
                String::from_utf16_lossy(&u)
            } else {
                text(b)
            };
            names.push((pid, name));
            q += 8 + if cp == 1200 { (raw + 3) & !3 } else { raw };
        }
    }
    for (pid, p) in entries {
        if pid == 1 {
            continue; // code page
        }
        let Some(ty) = u16at(p) else { continue };
        let v = match ty {
            2 => u16at(p + 4).map(|v| (v as i16).to_string()),
            3 => u32at(p + 4).map(|v| (v as i32).to_string()),
            0x13 => u32at(p + 4).map(|v| v.to_string()),
            5 => d.get(p + 4..p + 12).map(|b| f64::from_le_bytes([b[0], b[1], b[2], b[3], b[4], b[5], b[6], b[7]]).to_string()),
            0x0B => u16at(p + 4).map(|v| if v != 0 { "Yes".to_string() } else { "No".to_string() }),
            0x1E => u32at(p + 4).and_then(|l| d.get(p + 8..p + 8 + l as usize)).map(|b| text(b)),
            0x1F => u32at(p + 4).and_then(|l| d.get(p + 8..p + 8 + 2 * l as usize)).map(|b| {
                let u: Vec<u16> = b.chunks_exact(2).map(|c| u16::from_le_bytes([c[0], c[1]])).take_while(|&c| c != 0).collect();
                String::from_utf16_lossy(&u)
            }),
            _ => None,
        };
        let Some(v) = v else { continue };
        let name = names.iter().find(|n| n.0 == pid).map(|n| match n.1.find('(') {
            Some(k) if n.1.ends_with(')') && k > 0 => n.1[..k].to_string(),
            _ => n.1.clone(),
        });
        out.push((pid, name, v));
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn keywords_features_and_dimensions() {
        let t = "\u{86}<?xml version=\"1.0\"?>\r\n<Keywords id=\"9\" Name=\"doc\"><Configuration id=\"0\" Name=\"Default\" Type=\"ConfigurationManager\" Material=\"1060 Alloy\"/>\
            <Extrusion id=\"32\" Name=\"Boss-Extrude1\" Type=\"Boss-Extrude\"><Dimension Name=\"D1\">10</Dimension></Extrusion>\
            <Extrusion id=\"46\" Name=\"Cut-Extrude1\" Dissectable=\"true\"/><Feature id=\"1\" Name=\"Annotations\" Type=\"Annotations\"/>\
            <Feature id=\"34\" Name=\"Fillet1\" Type=\"Fillet\"><Dimension Name=\"D1\">R1</Dimension></Feature>\
            <HoleWizard id=\"50\" Name=\"&#216;5 Hole1\"><Dimension Name=\"Hole Dia.\">&lt;MOD-DIAM&gt;5</Dimension></HoleWizard>\
            <Sketch id=\"60\" Name=\"Sketch2&lt;3&gt;\" Type=\"Sketch2&lt;3&gt;\"/><Feature id=\"61\" Name=\"Revolve1\" Type=\"Revolve\"><Dimension Name=\"D1\">360°</Dimension></Feature></Keywords>";
        let (f, c) = keywords(t);
        assert_eq!(c, vec![SwConfig { name: "Default".into(), material: Some("1060 Alloy".into()) }]);
        let ids: Vec<(i64, &str, &str)> = f.iter().map(|x| (x.id, x.name.as_str(), x.kind.as_str())).collect();
        assert_eq!(ids, vec![(32, "Boss-Extrude1", "Boss-Extrude"), (34, "Fillet1", "Fillet"), (46, "Cut-Extrude1", "Extrusion"), (50, "Ø5 Hole1", "HoleWizard"), (60, "Sketch2<3>", "Sketch"), (61, "Revolve1", "Revolve")]);
        assert_eq!(f[0].dims, vec![SwDim { name: "D1".into(), text: "10".into(), value: Some(10.0) }]);
        assert_eq!(f[1].dims[0].value, Some(1.0));
        assert_eq!((f[3].dims[0].text.as_str(), f[3].dims[0].value), ("Ø5", Some(5.0)));
        assert_eq!(f[5].dims[0].value, Some(360.0));
        assert!(f[2].dims.is_empty());
    }

    #[test]
    fn paths_and_user_names_are_left_out() {
        let s = Scrub::new(["Example-User".to_string(), "ab".to_string()].into_iter());
        for bad in ["C:\\Users\\someone\\part.SLDPRT", "d:/work/part", "\\\\server\\share", "saved by example-user", "/home/x/y", "AB"] {
            assert!(!s.ok(bad), "{bad}");
        }
        for good in ["Boss-Extrude1", "PART 01 REV A", "http://example.com/x", "R1", "Ø20", "1:2 scale", "about"] {
            assert!(s.ok(good), "{good}");
        }
    }

    #[test]
    fn png_text_chunks_are_dropped() {
        let chunk = |ty: &[u8], data: &[u8]| [(data.len() as u32).to_be_bytes().to_vec(), ty.to_vec(), data.to_vec(), vec![0; 4]].concat();
        let ihdr = [0, 0, 2, 128, 0, 0, 1, 224, 8, 2, 0, 0, 0];
        let d = [vec![0x89, b'P', b'N', b'G', 0x0D, 0x0A, 0x1A, 0x0A], chunk(b"IHDR", &ihdr), chunk(b"tEXt", b"Author\0someone"), chunk(b"IDAT", &[1, 2, 3]), chunk(b"IEND", &[])].concat();
        let t = png(&d).unwrap();
        assert_eq!((t.width, t.height), (640, 480));
        assert_eq!(t.png.len(), d.len() - (12 + 14));
        assert!(!t.png.windows(4).any(|w| w == b"tEXt"));
        assert!(png(&d[..30]).is_none());
    }

    #[test]
    fn ole_property_set() {
        // one user-defined section: code page 65001, a dictionary {2: "Revision", 3: "SW-Last Saved By(Last Saved By)"}, an
        // i4 and two strings
        let mut sec: Vec<u8> = vec![];
        let props: Vec<(u32, Vec<u8>)> = vec![
            (1, [2u32.to_le_bytes().to_vec(), (0xFDE9u16 as i16).to_le_bytes().to_vec(), vec![0, 0]].concat()),
            (0, {
                let mut v = 2u32.to_le_bytes().to_vec();
                for (pid, n) in [(2u32, "Revision\0"), (3, "SW-Last Saved By(Last Saved By)\0")] {
                    v.extend(pid.to_le_bytes());
                    v.extend((n.len() as u32).to_le_bytes());
                    v.extend(n.as_bytes());
                }
                v
            }),
            (2, [0x1Eu32.to_le_bytes().to_vec(), 2u32.to_le_bytes().to_vec(), b"D\0".to_vec()].concat()),
            (3, [0x1Eu32.to_le_bytes().to_vec(), 4u32.to_le_bytes().to_vec(), b"abc\0".to_vec()].concat()),
            (4, [3u32.to_le_bytes().to_vec(), 7i32.to_le_bytes().to_vec()].concat()),
        ];
        let head = 8 + 8 * props.len();
        let mut off = head;
        let mut table = vec![];
        for (pid, v) in &props {
            table.extend(pid.to_le_bytes());
            table.extend((off as u32).to_le_bytes());
            off += v.len();
        }
        sec.extend((off as u32).to_le_bytes());
        sec.extend((props.len() as u32).to_le_bytes());
        sec.extend(table);
        for (_, v) in &props {
            sec.extend(v);
        }
        let mut d = vec![0xFE, 0xFF, 0, 0, 0, 0, 0, 0];
        d.extend([0u8; 16]);
        d.extend(1u32.to_le_bytes());
        d.extend(FMTID_USER);
        d.extend(48u32.to_le_bytes());
        d.extend(sec);
        let p = ole_props(&d, &FMTID_USER);
        assert_eq!(p, vec![(2, Some("Revision".into()), "D".into()), (3, Some("SW-Last Saved By".into()), "abc".into()), (4, None, "7".into())]);
        assert!(ole_props(&d, &FMTID_SUMMARY).is_empty());
        assert!(ole_props(&d[..40], &FMTID_USER).is_empty());
    }
}
