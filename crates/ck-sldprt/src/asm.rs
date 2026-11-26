//! `.SLDASM` (SolidWorks assembly): the component tree, read from the plain XML stream `swXmlContents/COMPINSTANCETREE`.
//!
//! An assembly file uses the same container as a part (see [`crate::container`]). Its tree stream is XML in the
//! `sw2003` schema (the layout below was read from the sample assemblies, not from another project's code):
//!
//! * `swHeader/swFile` (`id`, `swDocType` = `PART` | `ASSEMBLY`, `swPath`): every document the tree refers to. Only the
//!   file name part of `swPath` is kept (the folders of the author's machine are never returned).
//! * `swModelList/swModel` (`id`, `swName`, `swConfigurationName`, `swConfigurationId`, `swFileRef`, `swBoundingBox`):
//!   one entry per (document, configuration) in use. A part model is a leaf; an assembly model holds the components of
//!   that configuration as `swReference` children.
//! * `swReference` (`swComponentName`, `swReferenceNumber`, `swModelRef`, `swConfigurationName`, `swSuppressed`,
//!   `swHidden`, `swIsVirtualComponent`, `swExcludeFromBOM`, `swFlexible`, `swTransform`): one component instance. The
//!   instance name is `<swComponentName>-<swReferenceNumber>`. A reference to an assembly model is a sub-assembly: its
//!   components are the references of that model, expanded in place.
//! * `swTransform`: 16 numbers, a 4x4 matrix for row vectors (`p' = p * M`): rows 0 to 2 are the images of the part's
//!   x, y and z axes in the parent assembly, row 3 the origin (metres), and the last column is `0 0 0 1`. The
//!   transform is relative to the parent assembly. (Checked against the assembly's stored bounding box: the bounding
//!   boxes of the placed parts reproduce the assembly's exactly with this reading and not with the transpose.)
//! * `swBoundingBox`: `xmin ymin zmin xmax ymax zmax` of a model in its own coordinates, metres.
//! * `swConfigurationList/swConfiguration` (`swName`, `swMostRecentConfiguration`, `swModelRef`): the configurations of
//!   the top assembly; `swModelRef` is the assembly model of that configuration, whose references are its components.
//!
//! The root of the tree is the assembly model no reference points to. Mates are read by [`crate::mates`] (the `swConfigurationId`
//! of a configuration's model is the `N` of its `Config-N-MatesList` stream); in-context references, exploded views and the
//! geometry are not read; the referenced parts are not opened (the tree names them only).
//!
//! Never panics on malformed input: failures are [`SldprtError`] values (`asm.*`, `limit.*`, `container.*`).

use crate::container::{self, Container};
use crate::error::{Limits, SldprtError, R};
use crate::meta::{element, utf16_or_8};
use std::collections::HashMap;

/// Deepest nesting of sub-assemblies followed.
pub const MAX_DEPTH: usize = 64;

/// One configuration of the top assembly.
#[derive(Debug, Clone, PartialEq)]
pub struct AsmConfig {
    pub name: String,
    /// The configuration the file was last saved in.
    pub active: bool,
    /// `swConfigurationId` of the configuration's assembly model: the `N` of the stream `Contents/Config-N-MatesList` that holds
    /// its mates (see [`crate::mates`]).
    pub id: Option<u32>,
}

/// One component instance.
#[derive(Debug, Clone, PartialEq)]
pub struct AsmComponent {
    /// Instance name (`<component>-<n>`), as shown in the SolidWorks feature tree.
    pub name: String,
    /// Name of the model (document) it is an instance of, without extension.
    pub model: String,
    /// File name of the referenced document (`name.SLDPRT` or `name.SLDASM`, no folders), when the tree names one.
    pub part: Option<String>,
    /// `part` or `assembly` (from the document type of the referenced file), `None` when the tree does not say.
    pub kind: Option<&'static str>,
    /// Configuration of the referenced document this instance uses.
    pub configuration: String,
    /// Placement relative to the parent assembly: 4x4, row-major, for column vectors (`p' = M p`, translation in the last
    /// column), millimetres. Identity when the tree has none (see [`AsmTree::notes`]).
    pub transform: [f64; 16],
    pub suppressed: bool,
    pub hidden: bool,
    pub virtual_component: bool,
    pub exclude_from_bom: bool,
    pub flexible: bool,
    /// Bounding box of the referenced model in its own coordinates (`[xmin, ymin, zmin, xmax, ymax, zmax]`, mm).
    pub bbox: Option<[f64; 6]>,
    /// The components of a sub-assembly (empty for a part).
    pub children: Vec<AsmComponent>,
}

impl AsmComponent {
    /// Rotation (row-major 3x3, column-vector convention) and translation (mm) of [`Self::transform`].
    pub fn rotation_translation(&self) -> ([f64; 9], [f64; 3]) {
        let m = &self.transform;
        ([m[0], m[1], m[2], m[4], m[5], m[6], m[8], m[9], m[10]], [m[3], m[7], m[11]])
    }
}

/// The assembly tree of a `.SLDASM` file for one configuration.
#[derive(Debug, Clone, PartialEq)]
pub struct AsmTree {
    /// Model name of the top assembly.
    pub name: String,
    /// The configurations of the top assembly (the tree below is the one named in [`Self::configuration`]).
    pub configurations: Vec<AsmConfig>,
    pub configuration: String,
    pub components: Vec<AsmComponent>,
    /// Components counted in the whole tree (sub-assemblies included, suppressed ones too).
    pub count: usize,
    pub notes: Vec<String>,
}

impl AsmTree {
    /// Every component of the tree, depth first, with its parent's index path omitted.
    pub fn walk(&self) -> Vec<&AsmComponent> {
        fn go<'a>(c: &'a [AsmComponent], out: &mut Vec<&'a AsmComponent>) {
            for x in c {
                out.push(x);
                go(&x.children, out);
            }
        }
        let mut out = vec![];
        go(&self.components, &mut out);
        out
    }
}

/// Read the tree of the active configuration with the default limits.
pub fn read_asm(bytes: &[u8]) -> R<AsmTree> {
    read_asm_with(bytes, None, &Limits::default())
}

/// Read the tree of configuration `configuration` (the active one when `None`).
pub fn read_asm_with(bytes: &[u8], configuration: Option<&str>, limits: &Limits) -> R<AsmTree> {
    let c = container::open(bytes, limits)?;
    read_container(&c, configuration, limits)
}

pub fn read_container(c: &Container, configuration: Option<&str>, limits: &Limits) -> R<AsmTree> {
    let Some(d) = c.data("swXmlContents/COMPINSTANCETREE", limits) else {
        let why = if c.damaged.iter().any(|d| d.name.contains("COMPINSTANCETREE")) { "its block is damaged" } else { "the file has no assembly tree (it is not an assembly)" };
        return Err(SldprtError::new("asm.no_tree", format!("no `COMPINSTANCETREE` stream: {why}")));
    };
    parse_tree(&utf16_or_8(&d), configuration, limits)
}

#[derive(Default)]
struct File {
    kind: Option<&'static str>,
    name: Option<String>,
}

#[derive(Default)]
struct Model {
    id: String,
    name: String,
    file_ref: String,
    config_id: Option<u32>,
    bbox: Option<[f64; 6]>,
    refs: Vec<Reference>,
}

struct Reference {
    name: String,
    number: String,
    model_ref: String,
    configuration: String,
    transform: Option<[f64; 16]>,
    suppressed: bool,
    hidden: bool,
    virtual_component: bool,
    exclude_from_bom: bool,
    flexible: bool,
}

/// The file name of a Windows or POSIX path (the folders are dropped).
fn file_name(path: &str) -> Option<String> {
    let n = path.rsplit(['\\', '/']).next().unwrap_or("").trim();
    if n.is_empty() {
        None
    } else {
        Some(n.to_string())
    }
}

fn numbers<const N: usize>(s: &str) -> Option<[f64; N]> {
    let mut out = [0.0; N];
    let mut it = s.split_whitespace();
    for o in out.iter_mut() {
        let v: f64 = it.next()?.parse().ok()?;
        if !v.is_finite() {
            return None;
        }
        *o = v;
    }
    if it.next().is_some() {
        return None;
    }
    Some(out)
}

/// Row-vector file matrix (metres) to the column-vector matrix in millimetres.
fn to_mm(f: &[f64; 16]) -> [f64; 16] {
    let mut m = [0.0; 16];
    for r in 0..3 {
        for c in 0..3 {
            m[r * 4 + c] = f[c * 4 + r];
        }
        m[r * 4 + 3] = f[12 + r] * 1000.0;
    }
    m[15] = 1.0;
    m
}

const IDENTITY: [f64; 16] = [1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0];

fn yes(v: Option<&String>) -> bool {
    v.map_or(false, |s| s.eq_ignore_ascii_case("YES") || s == "1" || s.eq_ignore_ascii_case("true"))
}

pub fn parse_tree(t: &str, wanted: Option<&str>, limits: &Limits) -> R<AsmTree> {
    let mut files: HashMap<String, File> = HashMap::new();
    let mut models: Vec<Model> = vec![];
    let mut by_id: HashMap<String, usize> = HashMap::new();
    // (name, active, model id)
    let mut configs: Vec<(String, bool, String)> = vec![];
    let mut in_model: Option<usize> = None;
    let mut seen_root = false;
    let mut notes = vec![];
    let mut refs_seen = 0usize;
    let mut i = 0usize;
    while let Some(k) = t[i..].find('<') {
        let at = i + k;
        let Some((tag, attrs, self_closing, end)) = element(t, at) else { break };
        i = end;
        if tag == "swSolidWorks" {
            seen_root = true;
        }
        if tag == "/swModel" {
            in_model = None;
            continue;
        }
        let get = |k: &str| attrs.iter().find(|a| a.0 == k).map(|a| &a.1);
        match tag.as_str() {
            "swFile" => {
                if let Some(id) = get("id") {
                    let kind = match get("swDocType").map(|s| s.to_ascii_uppercase()).as_deref() {
                        Some("PART") => Some("part"),
                        Some("ASSEMBLY") => Some("assembly"),
                        _ => None,
                    };
                    files.insert(id.clone(), File { kind, name: get("swPath").and_then(|p| file_name(p)) });
                }
            }
            "swModel" => {
                let m = Model {
                    id: get("id").cloned().unwrap_or_default(),
                    name: get("swName").cloned().unwrap_or_default(),
                    file_ref: get("swFileRef").cloned().unwrap_or_default(),
                    config_id: get("swConfigurationId").and_then(|v| v.trim().parse().ok()),
                    bbox: get("swBoundingBox").and_then(|b| numbers::<6>(b)).map(|b| b.map(|v| v * 1000.0)),
                    refs: vec![],
                };
                // the configuration name is kept on the reference side; the model's own is only needed for the roots
                if let Some(id) = get("id") {
                    by_id.insert(id.clone(), models.len());
                }
                models.push(m);
                in_model = if self_closing { None } else { Some(models.len() - 1) };
            }
            "swReference" => {
                let Some(mi) = in_model else { continue };
                refs_seen += 1;
                if refs_seen > limits.max_components.saturating_mul(4) {
                    return Err(SldprtError::new("limit.components", format!("the assembly tree lists more than {} references", limits.max_components.saturating_mul(4))));
                }
                let tf = get("swTransform").and_then(|s| numbers::<16>(s));
                let r = Reference {
                    name: get("swComponentName").or_else(|| get("swName")).cloned().unwrap_or_default(),
                    number: get("swReferenceNumber").cloned().unwrap_or_default(),
                    model_ref: get("swModelRef").cloned().unwrap_or_default(),
                    configuration: get("swConfigurationName").cloned().unwrap_or_default(),
                    transform: tf,
                    suppressed: yes(get("swSuppressed")),
                    hidden: yes(get("swHidden")),
                    virtual_component: yes(get("swIsVirtualComponent")),
                    exclude_from_bom: yes(get("swExcludeFromBOM")),
                    flexible: yes(get("swFlexible")),
                };
                models[mi].refs.push(r);
            }
            "swConfiguration" => {
                if let Some(name) = get("swName") {
                    configs.push((name.clone(), yes(get("swMostRecentConfiguration")), get("swModelRef").cloned().unwrap_or_default()));
                }
            }
            _ => {}
        }
    }
    if !seen_root || models.is_empty() {
        return Err(SldprtError::new("asm.format", "the assembly tree stream holds no `swSolidWorks` model list"));
    }
    // roots: models no reference points to
    let referenced: std::collections::HashSet<&str> = models.iter().flat_map(|m| m.refs.iter().map(|r| r.model_ref.as_str())).collect();
    let is_root = |id: &str| !referenced.contains(id);
    let first_root = models.iter().position(|m| is_root(&m.id));
    if first_root.is_none() {
        return Err(SldprtError::new("asm.cycle", "every model of the assembly tree is referenced by another: no top assembly"));
    }
    // the top assembly's configurations: the listed ones whose model is a root (any order of the file kept)
    let mut cfgs: Vec<AsmConfig> = vec![];
    let mut cfg_model: Vec<&str> = vec![];
    for (name, active, mref) in &configs {
        if by_id.contains_key(mref) && is_root(mref) {
            cfgs.push(AsmConfig { name: name.clone(), active: *active, id: models[by_id[mref]].config_id });
            cfg_model.push(mref);
        }
    }
    let pick = match wanted {
        Some(w) => cfgs.iter().position(|c| c.name == w).ok_or_else(|| SldprtError::new("asm.configuration", format!("the assembly has no configuration `{w}` (it has: {})", cfgs.iter().map(|c| c.name.as_str()).collect::<Vec<_>>().join(", "))))?,
        None => cfgs.iter().position(|c| c.active).or(if cfgs.is_empty() { None } else { Some(0) }).unwrap_or(usize::MAX),
    };
    let (root_idx, configuration) = if pick != usize::MAX {
        (by_id[cfg_model[pick]], cfgs[pick].name.clone())
    } else {
        // no configuration list: the first root in file order
        (first_root.unwrap_or(0), String::new())
    };
    if let Some(f) = files.get(&models[root_idx].file_ref) {
        if f.kind == Some("part") {
            return Err(SldprtError::new("asm.not_assembly", "the file's top document is a part, not an assembly"));
        }
    }
    let mut count = 0usize;
    let components = expand(&models, &by_id, &files, root_idx, 0, &mut count, limits, &mut notes)?;
    notes.sort();
    notes.dedup();
    Ok(AsmTree { name: models[root_idx].name.clone(), configurations: cfgs, configuration, components, count, notes })
}

#[allow(clippy::too_many_arguments)]
fn expand(models: &[Model], by_id: &HashMap<String, usize>, files: &HashMap<String, File>, mi: usize, depth: usize, count: &mut usize, limits: &Limits, notes: &mut Vec<String>) -> R<Vec<AsmComponent>> {
    if depth >= MAX_DEPTH {
        return Err(SldprtError::new("limit.depth", format!("sub-assemblies nest deeper than {MAX_DEPTH} levels (a reference cycle?)")));
    }
    let mut out = Vec::with_capacity(models[mi].refs.len());
    for r in &models[mi].refs {
        *count += 1;
        if *count > limits.max_components {
            return Err(SldprtError::new("limit.components", format!("the assembly tree has more than {} components", limits.max_components)));
        }
        let target = by_id.get(&r.model_ref).copied();
        let m = target.map(|t| &models[t]);
        let file = m.and_then(|m| files.get(&m.file_ref));
        let transform = match r.transform {
            Some(f) => to_mm(&f),
            None => {
                notes.push("a component has no usable `swTransform`; it is placed at the origin".to_string());
                IDENTITY
            }
        };
        let name = if r.number.is_empty() { r.name.clone() } else { format!("{}-{}", r.name, r.number) };
        let children = match target {
            Some(t) if !models[t].refs.is_empty() => expand(models, by_id, files, t, depth + 1, count, limits, notes)?,
            _ => vec![],
        };
        if target.is_none() {
            notes.push(format!("component `{name}` refers to a model that is not in the tree"));
        }
        let kind = file.and_then(|f| f.kind).or(if children.is_empty() { None } else { Some("assembly") });
        out.push(AsmComponent {
            name,
            model: m.map(|m| m.name.clone()).unwrap_or_else(|| r.name.clone()),
            part: file.and_then(|f| f.name.clone()),
            kind,
            configuration: r.configuration.clone(),
            transform,
            suppressed: r.suppressed,
            hidden: r.hidden,
            virtual_component: r.virtual_component,
            exclude_from_bom: r.exclude_from_bom,
            flexible: r.flexible,
            bbox: m.and_then(|m| m.bbox),
            children,
        });
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;

    const HEAD: &str = r#"<?xml version="1.0" encoding="UTF-8"?><swSolidWorks xmlns="http://www.solidworks.com/sw2003/schema" swVersion="15000">"#;

    /// Top assembly "TOP" (two configurations A, B) holding a part twice and a sub-assembly "SUB" that holds the part once.
    fn synthetic() -> String {
        let r = |id: &str, name: &str, n: u32, mref: &str, cfg: &str, supp: &str, hid: &str, tf: &str| {
            format!(r#"<swReference id="{id}" swName="{name}" swReferenceNumber="{n}" swComponentName="{name}" swIsVirtualComponent="NO" swConfigurationName="{cfg}" swSuppressed="{supp}" swHidden="{hid}" swModelRef="{mref}" swTransform="{tf}"/>"#)
        };
        let id = "1 0 0 0 0 1 0 0 0 0 1 0 0 0 0 1";
        // rotation of 90 degrees about z for row vectors (x -> y, y -> -x), origin (0.1, 0.2, 0.3) m
        let rz = "0 1 0 0 -1 0 0 0 0 0 1 0 0.1 0.2 0.3 1";
        format!(
            r#"{HEAD}<swHeader><swFile id="3" swDocType="ASSEMBLY" swPath="C:\Some Folder\TOP.SLDASM"/><swFile id="6" swDocType="PART" swPath="C:\Some Folder\bolt &amp; nut.SLDPRT"/><swFile id="9" swDocType="ASSEMBLY" swPath="/x/SUB.SLDASM"/></swHeader><swModelList>
<swModel id="5" swName="bolt &amp; nut" swConfigurationName="Default" swFileRef="6" swBoundingBox="0 0 0 0.01 0.02 0.03"/>
<swModel id="8" swName="SUB" swConfigurationName="Default" swFileRef="9">{sub}</swModel>
<swModel id="2" swName="TOP" swConfigurationName="A" swFileRef="3">{a}{b}{c}<swPropertyList id="30"/></swModel>
<swModel id="12" swName="TOP" swConfigurationName="B" swFileRef="3">{d}</swModel>
</swModelList><swConfigurationList><swConfiguration id="1" swName="A" swMostRecentConfiguration="NO" swModelRef="2"/><swConfiguration id="11" swName="B" swMostRecentConfiguration="YES" swModelRef="12"/></swConfigurationList></swSolidWorks>"#,
            sub = r("20", "bolt &amp; nut", 1, "5", "Default", "NO", "NO", rz),
            a = r("21", "bolt &amp; nut", 1, "5", "Default", "NO", "NO", id),
            b = r("22", "bolt &amp; nut", 2, "5", "Default", "YES", "YES", rz),
            c = r("23", "SUB", 1, "8", "Default", "NO", "NO", rz),
            d = r("24", "bolt &amp; nut", 1, "5", "Default", "NO", "NO", id),
        )
    }

    #[test]
    fn reads_nested_components_flags_and_transforms() {
        let t = parse_tree(&synthetic(), Some("A"), &Limits::default()).unwrap();
        assert_eq!(t.name, "TOP");
        assert_eq!(t.configurations.iter().map(|c| (c.name.as_str(), c.active)).collect::<Vec<_>>(), vec![("A", false), ("B", true)]);
        assert_eq!(t.configuration, "A");
        assert_eq!(t.components.len(), 3);
        assert_eq!(t.count, 4);
        let c = &t.components;
        assert_eq!(c[0].name, "bolt & nut-1");
        assert_eq!(c[0].part.as_deref(), Some("bolt & nut.SLDPRT"));
        assert_eq!(c[0].kind, Some("part"));
        assert_eq!(c[0].bbox, Some([0.0, 0.0, 0.0, 10.0, 20.0, 30.0]));
        assert_eq!(c[0].transform, IDENTITY);
        assert!(!c[0].suppressed && !c[0].hidden);
        assert!(c[1].suppressed && c[1].hidden);
        assert_eq!(c[2].kind, Some("assembly"));
        assert_eq!(c[2].part.as_deref(), Some("SUB.SLDASM"));
        assert_eq!(c[2].children.len(), 1);
        // x axis of the part goes to +y in the assembly (column 0 of the column-vector matrix is (0,1,0)); translation in mm
        let (r, tr) = c[2].rotation_translation();
        assert_eq!(r, [0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0]);
        assert_eq!(tr, [100.0, 200.0, 300.0]);
        assert_eq!(t.walk().len(), 4);
    }

    #[test]
    fn default_is_the_active_configuration_and_an_unknown_one_is_refused() {
        let t = parse_tree(&synthetic(), None, &Limits::default()).unwrap();
        assert_eq!(t.configuration, "B");
        assert_eq!(t.components.len(), 1);
        let e = parse_tree(&synthetic(), Some("nope"), &Limits::default()).unwrap_err();
        assert_eq!(e.code, "asm.configuration");
    }

    #[test]
    fn malformed_trees_give_codes_not_panics() {
        let l = Limits::default();
        assert_eq!(parse_tree("", None, &l).unwrap_err().code, "asm.format");
        assert_eq!(parse_tree("<swSolidWorks><swModelList></swModelList></swSolidWorks>", None, &l).unwrap_err().code, "asm.format");
        // a reference cycle: A holds B, B holds A (no root at all)
        let cyc = format!(r#"{HEAD}<swModelList><swModel id="1" swName="A"><swReference swModelRef="2"/></swModel><swModel id="2" swName="B"><swReference swModelRef="1"/></swModel></swModelList></swSolidWorks>"#);
        assert_eq!(parse_tree(&cyc, None, &l).unwrap_err().code, "asm.cycle");
        // a cycle below a root: depth limit
        let deep = format!(r#"{HEAD}<swModelList><swModel id="0" swName="R"><swReference swModelRef="1"/></swModel><swModel id="1" swName="A"><swReference swModelRef="2"/></swModel><swModel id="2" swName="B"><swReference swModelRef="1"/></swModel></swModelList></swSolidWorks>"#);
        assert_eq!(parse_tree(&deep, None, &l).unwrap_err().code, "limit.depth");
        // bad transform, dangling model reference, truncated text
        let bad = format!(r#"{HEAD}<swModelList><swModel id="0" swName="R"><swReference swComponentName="x" swReferenceNumber="1" swModelRef="99" swTransform="1 2 3"/></swModel></swModelList></swSolidWorks>"#);
        let t = parse_tree(&bad, None, &l).unwrap();
        assert_eq!(t.components[0].transform, IDENTITY);
        assert_eq!(t.notes.len(), 2);
        let s = synthetic();
        for cut in (0..s.len()).step_by(37) {
            if s.is_char_boundary(cut) {
                let _ = parse_tree(&s[..cut], None, &l);
            }
        }
    }

    #[test]
    fn component_limit() {
        let mut s = format!(r#"{HEAD}<swModelList><swModel id="0" swName="R">"#);
        for n in 0..50 {
            s.push_str(&format!(r#"<swReference swComponentName="x" swReferenceNumber="{n}" swModelRef="1"/>"#));
        }
        s.push_str(r#"</swModel><swModel id="1" swName="P"/></swModelList></swSolidWorks>"#);
        let l = Limits { max_components: 10, ..Limits::default() };
        assert_eq!(parse_tree(&s, None, &l).unwrap_err().code, "limit.components");
        assert_eq!(parse_tree(&s, None, &Limits::default()).unwrap().components.len(), 50);
    }

    #[test]
    fn a_non_assembly_file_and_noise_are_refused() {
        let e = read_asm(&[1, 2, 3, 4]).unwrap_err();
        assert!(e.code.starts_with("container."), "{e}");
    }
}
