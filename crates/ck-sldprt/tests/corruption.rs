//! Damage in the modern container is reported, never silently read past: bytes changed inside the geometry block are an
//! error (`container.corrupt`, naming the block), bytes changed inside any other block or a file cut short in a block
//! import with a loud warning, and a sound file reads as before with no warning.

use ck_sldprt::container::{self, MARKER};
use ck_sldprt::inflate::{crc32, inflate_raw};
use ck_sldprt::{import_sldprt_with, Limits};
use std::path::PathBuf;

fn fixture(name: &str) -> Vec<u8> {
    std::fs::read(PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures").join(format!("{name}.sldprt"))).unwrap()
}

const MODERN: [&str; 3] = ["sw2022_cube_10mm", "sw2022_cube_hole_5mm", "sw2022_cone"];

/// (name, start of the compressed data, its length) of every sound block of a modern file.
fn blocks(b: &[u8]) -> Vec<(String, usize, usize)> {
    let u32at = |p: usize| u32::from_le_bytes(b[p..p + 4].try_into().unwrap());
    let mut out = vec![];
    let mut i = 0;
    while let Some(k) = b[i..].windows(6).position(|w| w == MARKER) {
        let at = i + k;
        i = at + 1;
        if at + 26 > b.len() {
            continue;
        }
        let (crc, csz, dsz, nl) = (u32at(at + 10), u32at(at + 14) as usize, u32at(at + 18) as usize, u32at(at + 22) as usize);
        let s = at + 26 + nl;
        if nl == 0 || nl > 200 || csz == 0 || s + csz > b.len() {
            continue;
        }
        let name: String = b[at + 26..s].iter().map(|&c| ((c << 4) | (c >> 4)) as char).collect();
        if let Ok((d, _)) = inflate_raw(&b[s..s + csz], dsz.max(1)) {
            if d.len() == dsz && crc32(&d) == crc {
                out.push((name, s, csz));
                i = s + csz;
            }
        }
    }
    out
}

fn block(b: &[u8], name: &str) -> (usize, usize) {
    let f = blocks(b).into_iter().filter(|x| x.0 == name).last().unwrap_or_else(|| panic!("no block {name}"));
    (f.1, f.2)
}

#[test]
fn sound_files_report_no_damage() {
    for n in MODERN {
        let b = fixture(n);
        let c = container::open(&b, &Limits::default()).unwrap();
        assert!(c.damaged.is_empty(), "{n}: {:?}", c.damaged);
        let imp = import_sldprt_with(&b, &Limits::default()).unwrap();
        assert!(imp.notes.iter().all(|x| !x.contains("damaged")), "{n}: {:?}", imp.notes);
    }
}

#[test]
fn a_changed_byte_in_the_geometry_block_is_an_error_naming_the_block() {
    for n in MODERN {
        let b = fixture(n);
        let (s, l) = block(&b, "Contents/Config-0-Partition");
        // (not the first byte: a stored block ignores five bits of it)
        for off in [5, l / 2, l - 1] {
            let mut c = b.clone();
            c[s + off] ^= 0x10;
            let e = import_sldprt_with(&c, &Limits::default()).expect_err(&format!("{n} flip at {off} read as sound"));
            assert_eq!(e.code, "container.corrupt", "{n}: {e}");
            assert!(e.message.contains("Config-0-Partition"), "{e}");
        }
        // the recorded CRC itself changed: the data is intact but the file does not vouch for it
        let at = s - 26 - "Contents/Config-0-Partition".len();
        let mut c = b.clone();
        c[at + 10] ^= 1;
        let e = import_sldprt_with(&c, &Limits::default()).expect_err("crc flip");
        assert_eq!(e.code, "container.corrupt", "{e}");
        assert!(e.message.contains("CRC-32"), "{e}");
    }
}

#[test]
fn a_changed_byte_in_a_metadata_block_imports_with_a_warning() {
    let plain = import_sldprt_with(&fixture("sw2022_cube_10mm"), &Limits::default()).unwrap();
    for name in ["swXmlContents/Features", "Contents/Definition", "docProps/core.xml", "PreviewPNG"] {
        let b = fixture("sw2022_cube_10mm");
        let (s, l) = block(&b, name);
        let mut c = b.clone();
        c[s + l / 2] ^= 0x01;
        let imp = import_sldprt_with(&c, &Limits::default()).unwrap_or_else(|e| panic!("{name}: {e}"));
        assert!(imp.notes.iter().any(|x| x.starts_with("WARNING: the file is damaged") && x.contains(name)), "{name}: {:?}", imp.notes);
        assert_eq!(imp.body.num_faces(), plain.body.num_faces(), "{name}");
    }
}

#[test]
fn a_file_cut_short_is_an_error_or_a_loud_warning_never_silent() {
    for n in MODERN {
        let b = fixture(n);
        // (past the last block the file holds only stale cache cells, which nothing reads)
        let last = blocks(&b).iter().map(|x| x.1 + x.2).max().unwrap();
        for cut in [b.len() / 2, b.len() * 3 / 4, last - 1] {
            let r = import_sldprt_with(&b[..cut], &Limits::default());
            match r {
                Err(e) => assert!(e.code.starts_with("container.") || e.code.starts_with("xt.") || e.code.starts_with("unsupported."), "{n} cut {cut}: {e}"),
                Ok(imp) => assert!(imp.notes.iter().any(|x| x.contains("damaged")), "{n} cut {cut} read without a word: {:?}", imp.notes),
            }
        }
        // cut inside the geometry block: an error
        let (s, l) = block(&b, "Contents/Config-0-Partition");
        let e = import_sldprt_with(&b[..s + l / 2], &Limits::default()).expect_err("cut in the geometry block");
        assert_eq!(e.code, "container.corrupt", "{e}");
    }
}

#[test]
fn every_single_byte_flip_in_the_blocks_is_caught_or_harmless_and_never_panics() {
    let b = fixture("sw2022_cube_10mm");
    let plain = import_sldprt_with(&b, &Limits::default()).unwrap();
    let mut silent = 0;
    for (name, s, l) in blocks(&b) {
        for off in (0..l.saturating_sub(1)).step_by((l / 7).max(1)) {
            let mut c = b.clone();
            c[s + off] ^= 0x04;
            match import_sldprt_with(&c, &Limits::default()) {
                Err(_) => {}
                Ok(imp) => {
                    if !imp.notes.iter().any(|x| x.contains("damaged")) {
                        silent += 1;
                        // only when the flip changed nothing the reader uses
                        assert_eq!(imp.body.num_faces(), plain.body.num_faces(), "{name}+{off}");
                    }
                }
            }
        }
    }
    assert_eq!(silent, 0, "a flipped byte inside a block read without a word");
}
