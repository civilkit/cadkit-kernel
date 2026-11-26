//! `.SLDASM` mate lists: a synthetic assembly container with a mate list stream (always runs), and a corpus-gated read of every
//! assembly under `SLDASM_CORPUS` (default `<repo>/corpus-local/samples`; skipped when there is none) that checks the decoded
//! mates against the component tree.

use ck_sldprt::{asm, container, mates, Limits};
use std::collections::BTreeMap;
use std::path::{Path, PathBuf};

fn container_with(streams: &[(&str, &[u8])]) -> Vec<u8> {
    let mut out = vec![0, 0, 0, 0, 0, 0, 0, 4];
    for (name, data) in streams {
        out.extend_from_slice(&container::MARKER);
        out.extend_from_slice(&0u32.to_le_bytes());
        out.extend_from_slice(&ck_sldprt::inflate::crc32(data).to_le_bytes());
        out.extend_from_slice(&((5 + data.len()) as u32).to_le_bytes());
        out.extend_from_slice(&(data.len() as u32).to_le_bytes());
        out.extend_from_slice(&(name.len() as u32).to_le_bytes());
        out.extend(name.bytes().map(|b| (b << 4) | (b >> 4)));
        out.push(1);
        out.extend_from_slice(&(data.len() as u16).to_le_bytes());
        out.extend_from_slice(&(!(data.len() as u16)).to_le_bytes());
        out.extend_from_slice(data);
        out.extend_from_slice(&[0; 16]);
    }
    out
}

fn cstr(s: &str) -> Vec<u8> {
    let mut v = vec![0xff, 0xfe, 0xff, s.chars().count() as u8];
    for u in s.encode_utf16() {
        v.extend_from_slice(&u.to_le_bytes());
    }
    v
}

fn rec(out: &mut Vec<u8>, class: &str, name: &str, id: u32, suppressed: bool, operands: &[String]) {
    out.extend_from_slice(&[0xff, 0xff, 1, 0]);
    out.extend_from_slice(&(class.len() as u16).to_le_bytes());
    out.extend_from_slice(class.as_bytes());
    out.extend_from_slice(&[0x20, 0x80]);
    out.extend(cstr(name));
    let mut h = vec![0u8, 1, 0, 0, 0, 0, 0];
    h.extend_from_slice(&((id << 8) | 0xc0).to_le_bytes());
    h.extend_from_slice(&[0; 5]);
    h.extend_from_slice(&[0xff, 0xfe, 0xff]);
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

/// Two configurations of TOP: ids 0 (Default) and 3 (Moved, part B shifted 5 mm along z). Mates exist for Default only.
const TREE: &str = r#"<?xml version="1.0" encoding="UTF-8"?><swSolidWorks xmlns="http://www.solidworks.com/sw2003/schema"><swHeader><swFile id="3" swDocType="ASSEMBLY" swPath="C:\a\TOP.SLDASM"/><swFile id="6" swDocType="PART" swPath="C:\a\P.SLDPRT"/></swHeader><swModelList><swModel id="5" swName="P" swConfigurationName="Default" swFileRef="6"/>
<swModel id="2" swName="TOP" swConfigurationName="Default" swConfigurationId="0" swFileRef="3"><swReference id="7" swComponentName="A" swReferenceNumber="1" swConfigurationName="Default" swSuppressed="NO" swHidden="NO" swModelRef="5" swTransform="1 0 0 0 0 1 0 0 0 0 1 0 0 0 0 1"/><swReference id="8" swComponentName="B" swReferenceNumber="1" swConfigurationName="Default" swSuppressed="NO" swHidden="NO" swModelRef="5" swTransform="1 0 0 0 0 1 0 0 0 0 1 0 0 0 0 1"/></swModel>
<swModel id="12" swName="TOP" swConfigurationName="Moved" swConfigurationId="3" swFileRef="3"><swReference id="17" swComponentName="A" swReferenceNumber="1" swConfigurationName="Default" swSuppressed="NO" swHidden="NO" swModelRef="5" swTransform="1 0 0 0 0 1 0 0 0 0 1 0 0 0 0 1"/><swReference id="18" swComponentName="B" swReferenceNumber="1" swConfigurationName="Default" swSuppressed="NO" swHidden="NO" swModelRef="5" swTransform="1 0 0 0 0 1 0 0 0 0 1 0 0 0 0.005 1"/></swModel>
</swModelList><swConfigurationList><swConfiguration id="1" swName="Default" swMostRecentConfiguration="YES" swModelRef="2"/><swConfiguration id="9" swName="Moved" swMostRecentConfiguration="NO" swModelRef="12"/></swConfigurationList></swSolidWorks>"#;

fn mates_stream(moved: bool) -> Vec<u8> {
    let mut d = vec![0x58, 1, 0, 0, 3, 0];
    let ops = |p: &str| vec!["A-1@TOP".to_string(), format!("{p}@A-1@TOP"), "B-1@TOP".to_string(), format!("{p}@B-1@TOP")];
    rec(&mut d, "moMateCoincident", "Coincident1", 0x11b, moved, &ops("Front Plane"));
    rec(&mut d, "moMateParallel", "Parallel1", 0x11c, false, &ops("Top Plane"));
    d
}

#[test]
fn a_synthetic_assembly_gives_mates_per_configuration_and_checks_them() {
    let f = container_with(&[("swXmlContents/COMPINSTANCETREE", TREE.as_bytes()), ("Contents/Config-0-MatesList", &mates_stream(false)), ("Contents/Config-3-MatesList", &mates_stream(true))]);
    let c = container::open(&f, &Limits::default()).unwrap();
    let lists = mates::read_container(&c, &Limits::default());
    assert_eq!(lists.iter().map(|l| l.config_id).collect::<Vec<_>>(), vec![0, 3]);
    // the configuration's id selects its list
    let t = asm::read_container(&c, Some("Default"), &Limits::default()).unwrap();
    assert_eq!(t.configurations.iter().map(|c| c.id).collect::<Vec<_>>(), vec![Some(0), Some(3)]);
    let id = t.configurations.iter().find(|c| c.name == t.configuration).and_then(|c| c.id);
    let l = mates::list_for(&lists, id).unwrap();
    assert_eq!(l.mates.len(), 2);
    let checks = mates::check_mates(&t, l);
    assert_eq!((checks[0].status, checks[1].status), ("holds", "holds"));
    // configuration Moved: B is 5 mm up; its list flags the coincident mate suppressed (unchecked), the parallel one still holds
    let t = asm::read_container(&c, Some("Moved"), &Limits::default()).unwrap();
    let id = t.configurations.iter().find(|c| c.name == t.configuration).and_then(|c| c.id);
    let l = mates::list_for(&lists, id).unwrap();
    assert!(l.mates[0].suppressed && !l.mates[1].suppressed);
    let checks = mates::check_mates(&t, l);
    assert_eq!((checks[0].status, checks[0].reason), ("unchecked", Some("suppressed")));
    assert_eq!(checks[1].status, "holds");
    // the same list against the wrong configuration's transforms: the coincident mate is violated by 5 mm
    let mut l2 = l.clone();
    l2.mates[0].suppressed = false;
    let checks = mates::check_mates(&t, &l2);
    assert_eq!(checks[0].status, "violated");
    assert!((checks[0].residual.unwrap() - 5.0).abs() < 1e-9);
}

#[test]
fn damaged_mate_streams_never_panic() {
    let s = mates_stream(false);
    let f = container_with(&[("swXmlContents/COMPINSTANCETREE", TREE.as_bytes()), ("Contents/Config-0-MatesList", &s)]);
    for cut in (0..f.len()).step_by(13) {
        if let Ok(c) = container::open(&f[..cut], &Limits::default()) {
            let _ = mates::read_container(&c, &Limits::default());
        }
    }
    let mut g = f.clone();
    for i in (0..g.len()).step_by(5) {
        g[i] ^= 0x33;
        if let Ok(c) = container::open(&g, &Limits::default()) {
            let _ = mates::read_container(&c, &Limits::default());
        }
        g[i] ^= 0x33;
    }
}

fn find(dir: &Path, out: &mut Vec<PathBuf>) {
    let Ok(rd) = std::fs::read_dir(dir) else { return };
    for e in rd.flatten() {
        let p = e.path();
        if p.is_dir() {
            find(&p, out);
        } else if p.extension().map_or(false, |x| x.eq_ignore_ascii_case("sldasm")) {
            out.push(p);
        }
    }
}

/// Every assembly under `SLDASM_CORPUS`: each mate list belongs to a configuration of the tree (`N` is a configuration id), every
/// mate has a name and a known type, no checked mate is violated by the tree's transforms, and the counts are printed.
#[test]
fn corpus_mates_read_and_hold() {
    let dir = std::env::var("SLDASM_CORPUS").map(PathBuf::from).unwrap_or_else(|_| PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../corpus-local/samples"));
    let mut files = vec![];
    find(&dir, &mut files);
    files.sort();
    if files.is_empty() {
        eprintln!("no .SLDASM files under {}: skipped", dir.display());
        return;
    }
    let lim = Limits::default();
    let (mut by_type, mut by_check, mut by_reason) = (BTreeMap::<&str, usize>::new(), BTreeMap::<&str, usize>::new(), BTreeMap::<&str, usize>::new());
    let (mut lists_n, mut mates_n, mut suppressed, mut resolved, mut operands_n) = (0, 0, 0, 0, 0);
    for f in &files {
        let bytes = std::fs::read(f).unwrap();
        let c = container::open(&bytes, &lim).unwrap();
        let lists = mates::read_container(&c, &lim);
        let base = asm::read_container(&c, None, &lim).unwrap();
        for l in &lists {
            // N is the id of a configuration of the top assembly
            let cfg = base.configurations.iter().find(|c| c.id == Some(l.config_id)).unwrap_or_else(|| panic!("{}: list {} matches no configuration id", f.display(), l.config_id));
            let t = asm::read_container(&c, Some(&cfg.name), &lim).unwrap();
            let worlds = mates::component_worlds(&t);
            let checks = mates::check_mates(&t, l);
            lists_n += 1;
            for (m, ck) in l.mates.iter().zip(&checks) {
                assert!(!m.name.is_empty());
                mates_n += 1;
                suppressed += m.suppressed as usize;
                *by_type.entry(m.kind).or_default() += 1;
                *by_check.entry(ck.status).or_default() += 1;
                if let Some(r) = ck.reason {
                    *by_reason.entry(r).or_default() += 1;
                }
                assert_ne!(ck.status, "violated", "{}: mate {} of list {} is violated by the tree's transforms (residual {:?})", f.display(), m.name, l.config_id, ck.residual);
                for o in &m.operands {
                    operands_n += 1;
                    resolved += o.component.as_ref().map_or(false, |p| worlds.contains_key(p)) as usize;
                }
            }
        }
    }
    eprintln!("{} assemblies, {lists_n} mate lists, {mates_n} mates ({suppressed} suppressed), operands {operands_n} ({resolved} name a component of the tree)", files.len());
    eprintln!("by type: {by_type:?}\nchecks: {by_check:?}\nunchecked reasons: {by_reason:?}");
}
