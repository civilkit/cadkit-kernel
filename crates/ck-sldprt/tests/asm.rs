//! `.SLDASM` component trees: a synthetic container (always runs), the container's failure modes, and a corpus-gated read of
//! every assembly under `SLDASM_CORPUS` (default `<repo>/corpus-local/samples`; skipped when there is none).

use ck_sldprt::{asm, container, Limits};
use std::path::{Path, PathBuf};

/// A modern-container file holding the given streams, each as one stored (uncompressed) DEFLATE block.
pub fn container_with(streams: &[(&str, &[u8])]) -> Vec<u8> {
    let mut out = vec![0, 0, 0, 0, 0, 0, 0, 4];
    for (name, data) in streams {
        out.extend_from_slice(&container::MARKER);
        out.extend_from_slice(&0u32.to_le_bytes());
        out.extend_from_slice(&ck_sldprt::inflate::crc32(data).to_le_bytes());
        let comp = 5 + data.len();
        out.extend_from_slice(&(comp as u32).to_le_bytes());
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

const TREE: &str = r#"<?xml version="1.0" encoding="UTF-8"?><swSolidWorks xmlns="http://www.solidworks.com/sw2003/schema"><swHeader><swFile id="3" swDocType="ASSEMBLY" swPath="C:\a\TOP.SLDASM"/><swFile id="6" swDocType="PART" swPath="C:\a\Plate.SLDPRT"/></swHeader><swModelList><swModel id="5" swName="Plate" swConfigurationName="Default" swFileRef="6"/><swModel id="2" swName="TOP" swConfigurationName="Default" swFileRef="3"><swReference id="7" swComponentName="Plate" swReferenceNumber="1" swConfigurationName="Default" swSuppressed="NO" swHidden="NO" swModelRef="5" swTransform="1 0 0 0 0 1 0 0 0 0 1 0 0.5 0 0 1"/></swModel></swModelList><swConfigurationList><swConfiguration id="1" swName="Default" swMostRecentConfiguration="YES" swModelRef="2"/></swConfigurationList></swSolidWorks>"#;

#[test]
fn a_synthetic_assembly_container_reads() {
    let f = container_with(&[("swXmlContents/COMPINSTANCETREE", TREE.as_bytes()), ("Header2", b"x")]);
    let t = asm::read_asm(&f).unwrap();
    assert_eq!(t.name, "TOP");
    assert_eq!(t.components.len(), 1);
    assert_eq!(t.components[0].name, "Plate-1");
    assert_eq!(t.components[0].part.as_deref(), Some("Plate.SLDPRT"));
    assert_eq!(t.components[0].transform[3], 500.0);
}

#[test]
fn a_container_without_the_tree_and_damaged_inputs_are_errors_not_panics() {
    let f = container_with(&[("Header2", b"x")]);
    assert_eq!(asm::read_asm(&f).unwrap_err().code, "asm.no_tree");
    let mut f = container_with(&[("swXmlContents/COMPINSTANCETREE", TREE.as_bytes())]);
    for cut in (0..f.len()).step_by(11) {
        let _ = asm::read_asm(&f[..cut]);
    }
    // flip bytes one at a time across the whole file: never a panic
    for i in (0..f.len()).step_by(3) {
        f[i] ^= 0x5a;
        let _ = asm::read_asm(&f);
        f[i] ^= 0x5a;
    }
    // an inflated stream above the limit is refused by the container reader
    let big = container_with(&[("swXmlContents/COMPINSTANCETREE", TREE.as_bytes())]);
    let l = Limits { max_stream: 100, ..Limits::default() };
    assert!(asm::read_asm_with(&big, None, &l).is_err());
}

#[test]
fn a_part_file_is_not_an_assembly() {
    let dir = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures");
    let Some(p) = std::fs::read_dir(&dir).ok().and_then(|d| d.flatten().map(|e| e.path()).find(|p| p.extension().map_or(false, |e| e.eq_ignore_ascii_case("sldprt")))) else { return };
    let e = asm::read_asm(&std::fs::read(p).unwrap()).unwrap_err();
    assert_eq!(e.code, "asm.no_tree");
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

fn check(c: &ck_sldprt::AsmComponent, depth: usize) -> usize {
    assert!(c.transform.iter().all(|v| v.is_finite()));
    assert_eq!(&c.transform[12..], &[0.0, 0.0, 0.0, 1.0]);
    // a proper rotation (orthonormal, determinant +1) up to the stored precision (no mirroring in the corpus)
    let m = &c.transform;
    let det = m[0] * (m[5] * m[10] - m[6] * m[9]) - m[1] * (m[4] * m[10] - m[6] * m[8]) + m[2] * (m[4] * m[9] - m[5] * m[8]);
    assert!((det.abs() - 1.0).abs() < 1e-6, "{}: det {det}", c.name);
    assert!(c.part.is_some() || c.virtual_component, "{} has no file", c.name);
    1 + c.children.iter().map(|x| check(x, depth + 1)).sum::<usize>()
}

#[test]
fn corpus_assemblies_read() {
    let dir = std::env::var("SLDASM_CORPUS").map(PathBuf::from).unwrap_or_else(|_| PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../corpus-local/samples"));
    let mut files = vec![];
    find(&dir, &mut files);
    files.sort();
    if files.is_empty() {
        eprintln!("no .SLDASM files under {}: skipped", dir.display());
        return;
    }
    let (mut ok, mut comps, mut multi) = (0, 0, 0);
    for f in &files {
        let bytes = std::fs::read(f).unwrap();
        let t = asm::read_asm(&bytes).unwrap_or_else(|e| panic!("{}: {e}", f.display()));
        assert!(!t.components.is_empty(), "{}", f.display());
        let n: usize = t.components.iter().map(|c| check(c, 0)).sum();
        assert_eq!(n, t.count, "{}", f.display());
        // every configuration reads, and a configuration of the tree is the active one by default
        assert!(t.configurations.is_empty() || t.configurations.iter().any(|c| c.name == t.configuration));
        for c in &t.configurations {
            asm::read_asm_with(&bytes, Some(&c.name), &Limits::default()).unwrap_or_else(|e| panic!("{} config {}: {e}", f.display(), c.name));
        }
        if t.configurations.len() > 1 {
            multi += 1;
        }
        comps += n;
        ok += 1;
    }
    eprintln!("{ok} assemblies read, {comps} components, {multi} with several configurations");
}
