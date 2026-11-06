//! Language-independent feature kinds. The `Type` attribute of a feature in the `KeyWords` tree is a display token: it is
//! translated to the language SolidWorks ran in for the first instance of some feature types (`Бобышка-Вытянуть`,
//! `凸台-拉伸`, `Boss.-Extru.`), and spelled with an internal English token for others (`Extrusion`, `Sketch`), in the same
//! file. The stable identity of a feature is the native class of its object in `Config-N-ResolvedFeatures`
//! (`moExtrusion_c`, `moRevolution_c`, `Fillet_c`, ...), written as a class declaration before the first object of the
//! class and as a map tag (a u16 with the high bit set) before the others.
//!
//! This reads the feature-name records of that stream (`<class declaration or tag> <name-record tag> ff fe ff <len>
//! <UTF-16 name>`) and gives every feature a family: from its own class declaration, else from another feature with the
//! same `Type` token or the same class tag whose family is known (an English token is known by definition). Features
//! whose token is not a recognised English one get the English token of their family; extrusions also get their Boolean
//! operation from the family word that follows the name record (`0x0140` joins, `0x01ca` subtracts) and so `Cut-Extrude`
//! or `Boss-Extrude`, whatever the name says.

use crate::container::Container;
use crate::error::Limits;
use crate::meta::SwFeature;
use std::collections::HashMap;

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum Fam {
    Extrude,
    Revolve,
    Fillet,
    Chamfer,
    Hole,
    LPattern,
    CirPattern,
    Mirror,
    Shell,
    Loft,
    Sweep,
    Dome,
    /// An inserted part (`moStock_c`): a feature the user names after the part it brings in, so its display token is that
    /// name; it brings its body in as a foreign one.
    Stock,
}

/// The family of an English feature type token (the tokens the editable rebuild knows).
fn token_family(k: &str) -> Option<Fam> {
    Some(match k {
        "Boss-Extrude" | "Extrusion" | "Cut-Extrude" | "ICE" | "Extrude" => Fam::Extrude,
        "Revolve" | "RevCut" | "Revolution" | "Revolve-Cut" | "Cut-Revolve" => Fam::Revolve,
        "Fillet" => Fam::Fillet,
        "Chamfer" => Fam::Chamfer,
        "HoleWizard" | "Hole" => Fam::Hole,
        "LPattern" => Fam::LPattern,
        "CirPattern" => Fam::CirPattern,
        "Mirror" | "MirrorPattern" => Fam::Mirror,
        "Shell" => Fam::Shell,
        "Loft" | "Blend" | "LoftBoss" | "LoftCut" | "BlendCut" | "Cut-Loft" | "CutLoft" => Fam::Loft,
        "Sweep" | "SweepCut" => Fam::Sweep,
        "Dome" => Fam::Dome,
        _ => return None,
    })
}

/// The family of a native class name; `Some(true)` when the class is a cut of its family.
fn class_family(c: &str) -> Option<(Fam, Option<bool>)> {
    Some(match c {
        "moExtrusion_c" | "moICE_c" | "moBoss_c" => (Fam::Extrude, None),
        "moCut_c" => (Fam::Extrude, Some(true)),
        "moRevolution_c" => (Fam::Revolve, Some(false)),
        "moRevCut_c" => (Fam::Revolve, Some(true)),
        "Fillet_c" => (Fam::Fillet, None),
        "Chamfer_c" => (Fam::Chamfer, None),
        "moHoleWzd_c" => (Fam::Hole, None),
        "moLPattern_c" => (Fam::LPattern, None),
        "moCirPattern_c" => (Fam::CirPattern, None),
        "moMirrorPattern_c" | "moMirrorSolid_c" => (Fam::Mirror, None),
        "moShell_c" => (Fam::Shell, None),
        "moBlend_c" => (Fam::Loft, None),
        "moSweep_c" => (Fam::Sweep, Some(false)),
        "moSweepCut_c" => (Fam::Sweep, Some(true)),
        "moDome_c" => (Fam::Dome, None),
        "moStock_c" => (Fam::Stock, None),
        _ => return None,
    })
}

/// Last resort, for a file whose native classes cannot be read: the display tokens met in files saved by localized
/// SolidWorks releases (Russian, Chinese, Polish, French) for the families above.
fn localized_family(k: &str) -> Option<(Fam, Option<bool>)> {
    Some(match k {
        "Бобышка-Вытянуть" | "凸台-拉伸" | "Dodanie-wyciągnięcie" | "Boss.-Extru." => (Fam::Extrude, Some(false)),
        "Вырез-Вытянуть" | "Wycięcie-wyciągnięcie" | "Enlèv. mat.-Extru." => (Fam::Extrude, Some(true)),
        "Повернуть" | "Obrót" => (Fam::Revolve, Some(false)),
        "Wycięcie-obrót" => (Fam::Revolve, Some(true)),
        "Skorupa" => (Fam::Shell, None),
        _ => return None,
    })
}

/// One feature-name record of the stream.
struct Rec {
    name: String,
    /// The class declared right before the record (first object of its class).
    decl: Option<String>,
    /// Otherwise the class tag before it.
    tag: Option<u16>,
    /// Boolean operation of an extrusion record: true = subtract.
    cut: Option<bool>,
}

fn records(d: &[u8]) -> Vec<Rec> {
    let mut out = vec![];
    let mut last_decl: Option<(usize, String)> = None;
    let mut i = 0usize;
    while i + 6 < d.len() {
        if d[i..i + 4] == [0xff, 0xff, 1, 0] {
            let l = u16::from_le_bytes([d[i + 4], d[i + 5]]) as usize;
            if l > 2 && l < 40 && i + 6 + l <= d.len() && d[i + 6..i + 6 + l].iter().all(|x| x.is_ascii_graphic()) {
                last_decl = Some((i + 6 + l, String::from_utf8_lossy(&d[i + 6..i + 6 + l]).into_owned()));
                i += 6 + l;
                continue;
            }
        }
        if d[i..i + 3] == [0xff, 0xfe, 0xff] && i >= 4 {
            let l = d[i + 3] as usize;
            if l > 0 && l < 80 && i + 4 + 2 * l <= d.len() {
                let t: Vec<u16> = (0..l).map(|k| u16::from_le_bytes([d[i + 4 + 2 * k], d[i + 5 + 2 * k]])).collect();
                if t.iter().all(|&x| x >= 32) {
                    let end = i + 4 + 2 * l;
                    if u16::from_le_bytes([d[i - 2], d[i - 1]]) & 0x8000 != 0 {
                        let decl = last_decl.as_ref().filter(|(e, _)| *e == i - 2).map(|x| x.1.clone());
                        let tag = if decl.is_none() { Some(u16::from_le_bytes([d[i - 4], d[i - 3]])).filter(|t| t & 0x8000 != 0) } else { None };
                        // extrusion trailer: four zero bytes, u16 family word, operation byte
                        let cut = if d.len() >= end + 7 && d[end..end + 4] == [0, 0, 0, 0] {
                            match u16::from_le_bytes([d[end + 4], d[end + 5]]) {
                                0x01ca => Some(true),
                                0x0140 => Some(false),
                                _ => None,
                            }
                        } else {
                            None
                        };
                        out.push(Rec { name: String::from_utf16_lossy(&t), decl, tag, cut });
                    }
                    i = end;
                    continue;
                }
            }
        }
        i += 1;
    }
    out
}

/// Rewrites the `kind` of every feature whose `Type` token is not one the editable rebuild knows, to the English token of
/// its family found from the native classes, and fixes the Boolean operation of extrusions. A file without the
/// resolved-features stream is left alone.
pub fn canonicalize(c: &Container, limits: &Limits, feats: &mut [SwFeature]) {
    let base = c.streams.iter().map(|s| s.name.as_str()).filter(|n| n.contains("Config-") && n.contains("ResolvedFeatures")).min_by_key(|n| (!n.contains("Config-0-"), n.len()));
    let d = base.and_then(|b| c.data(b.strip_suffix("__ZLB").unwrap_or(b), limits)).unwrap_or_default();
    canonicalize_stream(&d, feats);
}

fn canonicalize_stream(d: &[u8], feats: &mut [SwFeature]) {
    let recs = records(d);
    // per feature: its record (the first one that carries a declaration or a tag, preferring a declaration of a known class)
    let rec_of = |f: &SwFeature| -> Option<&Rec> {
        let mut it = recs.iter().filter(|r| r.name == f.name);
        let first = it.next()?;
        if first.decl.as_deref().and_then(class_family).is_some() {
            return Some(first);
        }
        recs.iter().filter(|r| r.name == f.name).find(|r| r.decl.as_deref().and_then(class_family).is_some()).or(Some(first))
    };
    let n = feats.len();
    let mut fam: Vec<Option<Fam>> = vec![None; n];
    let mut cut: Vec<Option<bool>> = vec![None; n];
    let mut tag: Vec<Option<u16>> = vec![None; n];
    for (i, f) in feats.iter().enumerate() {
        if let Some(r) = rec_of(f) {
            tag[i] = r.tag;
            cut[i] = r.cut;
            if let Some((fm, ct)) = r.decl.as_deref().and_then(class_family) {
                fam[i] = Some(fm);
                cut[i] = cut[i].or(ct);
            }
        }
        if fam[i].is_none() {
            fam[i] = token_family(&f.kind);
        }
    }
    // propagate: the same `Type` token, or the same class tag, means the same family (a conflict removes the entry)
    for _ in 0..4 {
        let mut by_tok: HashMap<&str, Option<Fam>> = HashMap::new();
        let mut by_tag: HashMap<u16, Option<Fam>> = HashMap::new();
        for (i, f) in feats.iter().enumerate() {
            let Some(fm) = fam[i] else { continue };
            let e = by_tok.entry(f.kind.as_str()).or_insert(Some(fm));
            if *e != Some(fm) {
                *e = None;
            }
            if let Some(t) = tag[i] {
                let e = by_tag.entry(t).or_insert(Some(fm));
                if *e != Some(fm) {
                    *e = None;
                }
            }
        }
        let mut changed = false;
        for (i, f) in feats.iter().enumerate() {
            if fam[i].is_some() {
                continue;
            }
            let got = by_tok.get(f.kind.as_str()).copied().flatten().or_else(|| tag[i].and_then(|t| by_tag.get(&t).copied().flatten()));
            if got.is_some() {
                fam[i] = got;
                changed = true;
            }
        }
        if !changed {
            break;
        }
    }
    for (i, f) in feats.iter().enumerate() {
        if fam[i].is_none() {
            if let Some((fm, ct)) = localized_family(&f.kind) {
                fam[i] = Some(fm);
                cut[i] = cut[i].or(ct);
            }
        }
    }
    for (i, f) in feats.iter_mut().enumerate() {
        let Some(fm) = fam[i] else { continue };
        let known = token_family(&f.kind).is_some();
        match fm {
            Fam::Extrude => {
                // an extrusion's operation: the trailer, else the class, else the English name prefix
                let is_cut = cut[i].unwrap_or_else(|| f.name.starts_with("Cut") || f.kind.starts_with("Cut"));
                if !known || (f.kind == "Extrusion" && cut[i].is_some() && is_cut != f.name.starts_with("Cut")) {
                    if f.kind.contains("Thin") || f.name.contains("Thin") {
                        continue;
                    }
                    f.kind = if is_cut { "Cut-Extrude" } else { "Boss-Extrude" }.into();
                }
            }
            Fam::Revolve if !known => f.kind = if cut[i] == Some(true) { "RevCut" } else { "Revolve" }.into(),
            Fam::Fillet if !known => f.kind = "Fillet".into(),
            Fam::Chamfer if !known => f.kind = "Chamfer".into(),
            Fam::Hole if !known => f.kind = "HoleWizard".into(),
            Fam::LPattern if !known => f.kind = "LPattern".into(),
            Fam::CirPattern if !known => f.kind = "CirPattern".into(),
            Fam::Mirror if !known => f.kind = "Mirror".into(),
            Fam::Shell if !known => f.kind = "Shell".into(),
            Fam::Loft if !known => f.kind = if f.name.starts_with("Cut") { "LoftCut" } else { "Loft" }.into(),
            Fam::Sweep if !known => f.kind = if cut[i] == Some(true) { "SweepCut" } else { "Sweep" }.into(),
            Fam::Dome if !known => f.kind = "Dome".into(),
            Fam::Stock if !known => f.kind = "Imported".into(),
            _ => {}
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn name_rec(out: &mut Vec<u8>, name: &str) {
        out.extend([0xb0, 0x80, 0xff, 0xfe, 0xff, name.chars().count() as u8]);
        for u in name.encode_utf16() {
            out.extend(u.to_le_bytes());
        }
    }
    fn decl(out: &mut Vec<u8>, class: &str) {
        out.extend([0xff, 0xff, 1, 0]);
        out.extend((class.len() as u16).to_le_bytes());
        out.extend(class.as_bytes());
    }
    fn feat(id: i64, name: &str, kind: &str) -> SwFeature {
        SwFeature { id, name: name.into(), kind: kind.into(), dims: vec![] }
    }

    /// A localized token resolves through the native class (declared or tagged) and the extrusion trailer gives the operation.
    #[test]
    fn localized_tokens_resolve_to_the_family_and_operation() {
        let mut d = vec![0u8; 8];
        // first boss: class declared; trailer family 0x0140 = joins
        decl(&mut d, "moExtrusion_c");
        name_rec(&mut d, "Бобышка1");
        d.extend([0, 0, 0, 0, 0x40, 0x01, 0]);
        d.extend([0u8; 12]);
        // a cut of class moICE_c, declared; trailer family 0x01ca = subtracts, English token `Extrusion`, localized name
        decl(&mut d, "moICE_c");
        name_rec(&mut d, "Вырез1");
        d.extend([0, 0, 0, 0, 0xca, 0x01, 0]);
        d.extend([0u8; 12]);
        // second boss: only a class tag; its token is the localized one of the first
        d.extend([0x93, 0x83]);
        name_rec(&mut d, "Бобышка2");
        d.extend([0, 0, 0, 0, 0x40, 0x01, 0]);
        d.extend([0u8; 12]);
        // a revolve: class declared
        decl(&mut d, "moRevCut_c");
        name_rec(&mut d, "Вырез-вращение1");
        d.extend([0u8; 12]);
        let mut fs = vec![
            feat(1, "Бобышка1", "Бобышка-Вытянуть"),
            feat(2, "Вырез1", "Extrusion"),
            feat(3, "Бобышка2", "Бобышка-Вытянуть"),
            feat(4, "Вырез-вращение1", "Вырез-Повернуть"),
            feat(5, "Эскиз1", "Sketch"),
        ];
        canonicalize_stream(&d, &mut fs);
        let kinds: Vec<&str> = fs.iter().map(|f| f.kind.as_str()).collect();
        assert_eq!(kinds, ["Boss-Extrude", "Cut-Extrude", "Boss-Extrude", "RevCut", "Sketch"]);
    }

    /// Without the stream the few display tokens met in localized files still classify.
    #[test]
    fn display_token_table_is_the_fallback() {
        let mut fs = vec![feat(1, "Obrót1", "Obrót"), feat(2, "凸台-拉伸1", "凸台-拉伸")];
        canonicalize_stream(&[0u8; 16], &mut fs);
        assert_eq!(fs[0].kind, "Revolve");
        assert_eq!(fs[1].kind, "Boss-Extrude");
    }
    /// A feature the user named after an inserted part (its `Type` token is its own name) is an inserted part by its class.
    #[test]
    fn a_feature_named_after_an_inserted_part_is_found_by_its_class() {
        let mut d = vec![0u8; 8];
        decl(&mut d, "moStock_c");
        name_rec(&mut d, "plate_pinhole_profile");
        d.extend([0u8; 12]);
        decl(&mut d, "moExtrusion_c");
        name_rec(&mut d, "Boss-Extrude1");
        d.extend([0u8; 12]);
        let mut fs = vec![feat(1, "plate_pinhole_profile", "plate_pinhole_profile"), feat(2, "Boss-Extrude1", "Extrusion")];
        canonicalize_stream(&d, &mut fs);
        assert_eq!(fs[0].kind, "Imported");
        assert_eq!(fs[1].kind, "Extrusion");
    }

}
