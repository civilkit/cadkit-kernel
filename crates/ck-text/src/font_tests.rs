//! Loaded fonts: reader coverage (glyf, composites, CFF, variable), the registry (name + content hash),
//! subsetting. Fixtures in `tests/fixtures/` are tiny subsets of the curated fonts (licences next to them);
//! `tests/fonts-full/` (not in the repository) holds the whole fonts for the exploratory `full_fonts_report`.

use super::tests::cross_free;
use super::*;
use std::sync::Arc;

const FIXTURES: [(&str, &[u8]); 11] = [
    ("sans-barlow", include_bytes!("../tests/fixtures/fx-sans-barlow.ttf")),
    ("sans-source-otf", include_bytes!("../tests/fixtures/fx-sans-source-otf.otf")),
    ("condensed-barlow", include_bytes!("../tests/fixtures/fx-condensed-barlow.ttf")),
    ("condensed-bebas", include_bytes!("../tests/fixtures/fx-condensed-bebas.ttf")),
    ("serif-ptserif", include_bytes!("../tests/fixtures/fx-serif-ptserif.ttf")),
    ("slab-specialelite", include_bytes!("../tests/fixtures/fx-slab-specialelite.ttf")),
    ("mono-plex", include_bytes!("../tests/fixtures/fx-mono-plex.ttf")),
    ("script-greatvibes", include_bytes!("../tests/fixtures/fx-script-greatvibes.ttf")),
    ("script-satisfy", include_bytes!("../tests/fixtures/fx-script-satisfy.ttf")),
    ("display-anton", include_bytes!("../tests/fixtures/fx-display-anton.ttf")),
    ("variable-inter", include_bytes!("../tests/fixtures/fx-variable-inter.ttf")),
];
const GAUGE: &[u8] = include_bytes!("../fonts/gauge.ttf");

fn fixture(name: &str) -> &'static [u8] {
    FIXTURES.iter().find(|f| f.0 == name).unwrap().1
}

fn spec(string: &str, font: &str, h: f64) -> Spec {
    Spec { string: string.into(), font: font.into(), height: h, ..Spec::default() }
}

fn closed(l: &Layout) -> bool {
    l.regions.iter().flat_map(|r| r.loops()).all(|lp| {
        let n = lp.len();
        (0..n).all(|i| {
            let (a, b) = (lp[i].end(), lp[(i + 1) % n].start());
            (a[0] - b[0]).hypot(a[1] - b[1]) < 1e-9
        })
    })
}

fn same_layout(a: &Layout, b: &Layout) {
    assert_eq!(a.regions.len(), b.regions.len());
    for (x, y) in a.regions.iter().zip(&b.regions) {
        assert_eq!(x.outer, y.outer);
        assert_eq!(x.holes, y.holes);
    }
    assert_eq!(a.bbox, b.bbox);
}

#[test]
fn every_fixture_loads_and_lays_out() {
    for (n, bytes) in FIXTURES {
        let name = format!("t-fix-{n}");
        let info = load_font(&name, bytes.to_vec(), true).unwrap_or_else(|e| panic!("{n}: {e:?}"));
        assert!(info.glyphs >= 20 && info.glyphs < 80, "{n}: {}", info.glyphs);
        assert!(info.coverage.iter().any(|c| c.0 == "Basic Latin"), "{n}");
        let l = layout(&spec("HAOBRPM 0689%&@xvi", &name, 10.0)).unwrap_or_else(|e| panic!("{n}: {e:?}"));
        assert!(closed(&l), "{n}");
        assert!(l.warnings.iter().all(|w| w.code != "textMissingGlyph"), "{n}: {:?}", l.warnings);
        assert!(l.regions.len() >= 12, "{n}: {}", l.regions.len());
        // outer loops counter-clockwise, holes clockwise
        for r in &l.regions {
            assert!(signed_area(&loop_polygon(&r.outer, 8)) > 0.0, "{n} outer");
            for h in &r.holes {
                assert!(signed_area(&loop_polygon(h, 8)) < 0.0, "{n} hole");
            }
        }
        // the cap height is honoured: "H" is as tall as asked
        let h = layout(&spec("H", &name, 10.0)).unwrap();
        assert!((h.bbox[3] - 10.0).abs() < 0.05, "{n}: {:?}", h.bbox);
        // O has a hole in every face but the scripts' (their O may be open)
        if !n.starts_with("script") {
            let o = layout(&spec("O", &name, 10.0)).unwrap();
            assert_eq!(o.regions.len(), 1, "{n}");
            assert_eq!(o.regions[0].holes.len(), 1, "{n}");
        }
        // accented letters (composites in TrueType, own glyphs in CFF)
        let acc = layout(&spec("\u{e9}\u{c0}", &name, 10.0)).unwrap();
        assert!(acc.regions.len() >= 2, "{n}");
        assert!(acc.warnings.iter().all(|w| w.code != "textMissingGlyph"), "{n}: {:?}", acc.warnings);
        unload_font(&name).unwrap();
    }
}

#[test]
fn cff_outline_matches_the_font() {
    let name = "t-cff";
    let info = load_font(name, fixture("sans-source-otf").to_vec(), true).unwrap();
    assert_eq!(info.outlines, "cff");
    let f = font(name).unwrap();
    assert_eq!(f.kind(), OutlineKind::Cff);
    assert_eq!(f.cap_height(), 656.0);
    // areas in font units (scale 1 when the height is the cap height), from fontTools' AreaPen on the same file
    for (ch, want) in [("H", 131512.0), ("O", 140347.4), ("B", 151504.3), ("g", 142849.1), ("&", 147576.0), ("e", 98987.6), ("%", 150521.2)] {
        let l = layout(&spec(ch, name, 656.0)).unwrap();
        let rel = (l.area() - want).abs() / want;
        assert!(rel < 0.005, "{ch}: area {} vs {want} ({rel})", l.area());
    }
    // advance widths from hmtx
    let g = f.glyph_index('H').unwrap();
    assert_eq!(f.advance(g), 652.0);
    unload_font(name).unwrap();
}

#[test]
fn cff_cubic_to_quadratic_stays_close() {
    // distance of the quadratic outline from the exact cubic one is bounded by the tolerance (0.05 % em)
    let mut out = vec![];
    let (p0, p1, p2, p3) = ((0.0, 0.0), (0.0, 300.0), (500.0, 700.0), (800.0, 0.0));
    ttf::cubic_to_quads_for_test(p0, p1, p2, p3, 0.5, &mut out);
    let mut worst = 0.0f64;
    let quads: Vec<_> = out.chunks(2).collect();
    let mut start = p0;
    for q in &quads {
        let (c, e) = (q[0], q[1]);
        for i in 0..=20 {
            let t = i as f64 / 20.0;
            let u = 1.0 - t;
            let s = (u * u * start.0 + 2.0 * u * t * c.x + t * t * e.x, u * u * start.1 + 2.0 * u * t * c.y + t * t * e.y);
            // nearest point on the cubic
            let mut best = f64::MAX;
            for k in 0..=2000 {
                let t = k as f64 / 2000.0;
                let u = 1.0 - t;
                let cx = u * u * u * p0.0 + 3.0 * u * u * t * p1.0 + 3.0 * u * t * t * p2.0 + t * t * t * p3.0;
                let cy = u * u * u * p0.1 + 3.0 * u * u * t * p1.1 + 3.0 * u * t * t * p2.1 + t * t * t * p3.1;
                best = best.min((cx - s.0).hypot(cy - s.1));
            }
            worst = worst.max(best);
        }
        start = (e.x, e.y);
    }
    assert!(worst < 0.6, "worst {worst}");
    assert!(quads.len() >= 2);
}

#[test]
fn loaded_font_gives_the_same_geometry_as_the_builtin() {
    let info = load_font("t-gauge-copy", GAUGE.to_vec(), true).unwrap();
    assert!(!info.builtin);
    for (s, h) in [("RPM x100", 8.0), ("OIL PSI 0123456789", 3.0), ("\u{c9}\u{e0}%&@", 12.0)] {
        for poly in [false, true] {
            let a = layout(&Spec { polyline: poly, ..spec(s, "gauge", h) }).unwrap();
            let b = layout(&Spec { polyline: poly, ..spec(s, "t-gauge-copy", h) }).unwrap();
            same_layout(&a, &b);
        }
        let ma = metrics(s, "gauge", None, h, 1.0).unwrap();
        let mb = metrics(s, "t-gauge-copy", None, h, 1.0).unwrap();
        assert_eq!(ma.advances, mb.advances);
        assert_eq!(ma.bbox, mb.bbox);
    }
    // same hash as the built-in
    assert_eq!(info.hash, sha256_hex(&font_entry("gauge").unwrap().hash));
    unload_font("t-gauge-copy").unwrap();
}

#[test]
fn missing_font_is_a_clear_error() {
    let e = layout(&spec("A", "t-not-there", 5.0)).unwrap_err();
    assert_eq!(e, Error::FontNotLoaded("t-not-there".into()));
    assert_eq!(e.code(), "textFontNotLoaded");
    let m = e.message();
    assert!(m.contains("\"t-not-there\"") && m.contains("loadFont"), "{m}");
    assert_eq!(metrics("A", "t-not-there", None, 5.0, 1.0).unwrap_err().code(), "textFontNotLoaded");
    // unloading takes the font away again, no silent substitution
    load_font("t-gone", GAUGE.to_vec(), false).unwrap();
    assert!(layout(&spec("A", "t-gone", 5.0)).is_ok());
    assert_eq!(unload_font("t-gone"), Ok(true));
    assert_eq!(unload_font("t-gone"), Ok(false));
    assert_eq!(layout(&spec("A", "t-gone", 5.0)).unwrap_err().code(), "textFontNotLoaded");
}

#[test]
fn name_and_content_hash_key_the_font() {
    let a = fixture("sans-barlow").to_vec();
    let b = fixture("serif-ptserif").to_vec();
    let name = "t-twin";
    let ia = load_font(name, a.clone(), false).unwrap();
    assert!(!ia.already_loaded && !ia.replaced);
    // the same file again is a no-op
    let again = load_font(name, a.clone(), false).unwrap();
    assert!(again.already_loaded);
    assert_eq!(again.hash, ia.hash);
    // a different file under the same name does not silently replace it
    let e = load_font(name, b.clone(), false).unwrap_err();
    assert_eq!(e.code, "fontNameConflict");
    assert!(layout(&spec("RPM", name, 5.0)).unwrap().bbox == layout(&spec("RPM", name, 5.0)).unwrap().bbox);
    let width_a = layout(&spec("RPM", name, 5.0)).unwrap().bbox[2];
    let fp_a = font_fingerprint(name, None);
    // pinned to the first file: fine while it is loaded
    let pin = &ia.hash[..12];
    assert!(layout(&Spec { font_hash: Some(pin.into()), ..spec("RPM", name, 5.0) }).is_ok());
    // replace:true swaps it; the fingerprint (regen cache key) and the geometry change, a stale pin is refused
    let ib = load_font(name, b, true).unwrap();
    assert!(ib.replaced);
    assert_ne!(ib.hash, ia.hash);
    assert_ne!(font_fingerprint(name, None), fp_a);
    let width_b = layout(&spec("RPM", name, 5.0)).unwrap().bbox[2];
    assert!((width_a - width_b).abs() > 1e-3);
    let e = layout(&Spec { font_hash: Some(pin.into()), ..spec("RPM", name, 5.0) }).unwrap_err();
    assert_eq!(e.code(), "textFontMismatch");
    assert!(e.message().contains(name));
    assert!(layout(&Spec { font_hash: Some(ib.hash[..8].to_uppercase()), ..spec("RPM", name, 5.0) }).is_ok());
    // unloaded: the fingerprint says so
    unload_font(name).unwrap();
    assert!(font_fingerprint(name, None).contains("missing"));
    // the same bytes under two names are two fonts with one hash
    load_font("t-twin-1", a.clone(), false).unwrap();
    load_font("t-twin-2", a, false).unwrap();
    assert_ne!(font_fingerprint("t-twin-1", None), font_fingerprint("t-twin-2", None));
    unload_font("t-twin-1").unwrap();
    unload_font("t-twin-2").unwrap();
}

#[test]
fn builtin_names_are_reserved_and_names_are_checked() {
    for n in ["gauge", "engrave", "sans", "serif", "default", ""] {
        assert_eq!(load_font(n, GAUGE.to_vec(), true).unwrap_err().code, if n.is_empty() { "fontBadName" } else { "fontReserved" }, "{n:?}");
    }
    assert_eq!(unload_font("gauge").unwrap_err().code, "fontReserved");
    assert_eq!(load_font(" pad ", GAUGE.to_vec(), false).unwrap_err().code, "fontBadName");
    assert_eq!(load_font("a\nb", GAUGE.to_vec(), false).unwrap_err().code, "fontBadName");
    assert_eq!(load_font(&"x".repeat(65), GAUGE.to_vec(), false).unwrap_err().code, "fontBadName");
    assert_eq!(load_font("t-empty", vec![], false).unwrap_err().code, "fontBadData");
    // the list has the built-ins first
    let l = list_fonts();
    assert_eq!(l[0].name, "gauge");
    assert_eq!(l[1].name, "engrave");
    assert!(l[0].builtin && l[1].builtin);
}

#[test]
fn bad_files_are_errors_not_panics() {
    for (n, bytes) in [
        ("t-bad-txt", b"this is not a font at all".to_vec()),
        ("t-bad-woff", b"wOFF\0\x01\0\0\0\0\0\0".to_vec()),
        ("t-bad-short", vec![0, 1, 0, 0, 0, 3]),
        ("t-bad-trunc", GAUGE[..GAUGE.len() / 2].to_vec()),
        ("t-bad-trunc2", GAUGE[..300].to_vec()),
    ] {
        let e = load_font(n, bytes, false).unwrap_err();
        assert_eq!(e.code, "fontBadData", "{n}: {e:?}");
        assert!(list_fonts().iter().all(|f| f.name != n));
    }
    assert!(load_font("t-woff", b"wOF2....".to_vec(), false).unwrap_err().message.contains("WOFF"));
}

#[test]
fn damaged_fonts_never_panic() {
    // deterministic byte flips and truncations of three fixtures (TrueType, CFF, variable): parse, map, draw, subset
    let mut seed = 0x9E37_79B9_7F4A_7C15u64;
    let mut next = move || {
        seed ^= seed << 13;
        seed ^= seed >> 7;
        seed ^= seed << 17;
        seed
    };
    for n in ["sans-barlow", "sans-source-otf", "variable-inter"] {
        let orig = fixture(n);
        for round in 0..300 {
            let mut b = orig.to_vec();
            if round % 5 == 4 {
                b.truncate((next() as usize) % orig.len());
            } else {
                for _ in 0..1 + round % 4 {
                    let i = (next() as usize) % b.len();
                    b[i] = next() as u8;
                }
            }
            if let Ok(f) = Font::parse(Arc::from(b)) {
                let _ = f.mapped_chars();
                let _ = f.display_name();
                for ch in "HAOBg\u{e9}&".chars() {
                    if let Some(g) = f.glyph_index(ch) {
                        let _ = f.contours(g);
                        let _ = f.advance(g);
                    }
                }
                let _ = f.subset(&['H', 'O']);
            }
        }
    }
}

#[test]
fn collection_reads_its_first_face() {
    let mut b = b"ttcf".to_vec();
    b.extend([0, 1, 0, 0]);
    b.extend(1u32.to_be_bytes());
    b.extend(16u32.to_be_bytes());
    b.extend(GAUGE);
    let n = u16::from_be_bytes([GAUGE[4], GAUGE[5]]) as usize;
    for i in 0..n {
        let p = 16 + 12 + i * 16 + 8;
        let off = u32::from_be_bytes([b[p], b[p + 1], b[p + 2], b[p + 3]]) + 16;
        b[p..p + 4].copy_from_slice(&off.to_be_bytes());
    }
    let info = load_font("t-ttc", b, false).unwrap();
    assert!(info.warnings.iter().any(|w| w.code == "fontCollectionFirstFace"));
    same_layout(&layout(&spec("RPM x100", "t-ttc", 6.0)).unwrap(), &layout(&spec("RPM x100", "gauge", 6.0)).unwrap());
    unload_font("t-ttc").unwrap();
}

#[test]
fn load_warnings_name_what_is_ignored() {
    let w = |n: &str| {
        let name = format!("t-w-{n}");
        let info = load_font(&name, fixture(n).to_vec(), true).unwrap();
        unload_font(&name).unwrap();
        info.warnings.iter().map(|w| w.code).collect::<Vec<_>>()
    };
    assert!(w("variable-inter").contains(&"fontVariableDefaultOnly"));
    assert!(w("variable-inter").contains(&"fontOverlappingContours"));
    assert!(w("slab-specialelite").contains(&"fontOverlappingContours"));
    assert!(!w("sans-barlow").contains(&"fontOverlappingContours"));
    assert!(w("sans-source-otf").contains(&"fontCffApproximated"));
    assert!(w("sans-barlow").contains(&"fontKerningIgnored"));
    assert!(w("serif-ptserif").contains(&"fontKerningIgnored"));
    assert!(list_fonts()[0].warnings.is_empty() && list_fonts()[1].warnings.is_empty());
    assert!(!w("condensed-bebas").contains(&"fontCffApproximated"));
}

#[test]
fn variable_font_reads_its_default_instance() {
    load_font("t-var", fixture("variable-inter").to_vec(), true).unwrap();
    let l = layout(&spec("HOe\u{e9}", "t-var", 10.0)).unwrap();
    assert!(closed(&l));
    assert!(l.regions.len() >= 4);
    unload_font("t-var").unwrap();
}

#[test]
fn subset_of_a_truetype_font_keeps_the_geometry_exactly() {
    load_font("t-sub-src", fixture("sans-barlow").to_vec(), true).unwrap();
    let text = "H\u{e9}\u{c0}O&";
    let (bytes, warns) = subset_font("t-sub-src", text).unwrap();
    assert!(warns.is_empty());
    assert!(bytes.len() < fixture("sans-barlow").len());
    let info = load_font("t-sub", bytes.clone(), true).unwrap();
    // .notdef + the five characters (H is among them)
    assert_eq!(info.glyphs, 6, "{info:?}"); // composites are flattened: no component glyphs
    for s in ["H\u{e9}", "\u{c0}O&H", "&"] {
        same_layout(&layout(&spec(s, "t-sub-src", 7.0)).unwrap(), &layout(&spec(s, "t-sub", 7.0)).unwrap());
    }
    // a character that was not asked for is gone (no silent fallback)
    let l = layout(&spec("HB", "t-sub", 7.0)).unwrap();
    assert!(l.warnings.iter().any(|w| w.code == "textMissingGlyph"));
    // the subset is itself a plain TrueType font: subsetting it again keeps its bytes' meaning
    let again = font("t-sub").unwrap().subset(&['H']).unwrap();
    assert!(Font::parse(Arc::from(again)).is_ok());
    unload_font("t-sub").unwrap();
    unload_font("t-sub-src").unwrap();
}

#[test]
fn subset_of_a_cff_font_becomes_truetype_with_the_same_outline() {
    load_font("t-sub-cff", fixture("sans-source-otf").to_vec(), true).unwrap();
    let (bytes, _) = subset_font("t-sub-cff", "HOe&").unwrap();
    assert_eq!(&bytes[..4], &[0, 1, 0, 0]);
    load_font("t-sub-cff2", bytes, true).unwrap();
    assert_eq!(font("t-sub-cff2").unwrap().kind(), OutlineKind::Glyf);
    assert_eq!(font("t-sub-cff2").unwrap().cap_height(), 656.0);
    for s in ["H", "O", "e&"] {
        let (a, b) = (layout(&spec(s, "t-sub-cff", 656.0)).unwrap(), layout(&spec(s, "t-sub-cff2", 656.0)).unwrap());
        assert!((a.area() - b.area()).abs() / a.area() < 0.002, "{s}: {} vs {}", a.area(), b.area());
        assert_eq!(a.regions.len(), b.regions.len());
    }
    let adv = |n: &str| font(n).unwrap().advance(font(n).unwrap().glyph_index('O').unwrap());
    assert_eq!(adv("t-sub-cff"), adv("t-sub-cff2"));
    unload_font("t-sub-cff").unwrap();
    unload_font("t-sub-cff2").unwrap();
}

#[test]
fn subset_of_the_builtin_font_round_trips() {
    let (bytes, _) = subset_font("gauge", "RPM x100").unwrap();
    let full = font("gauge").unwrap().bytes().len();
    assert!(bytes.len() * 3 < full, "{} vs {full}", bytes.len());
    load_font("t-gauge-sub", bytes, true).unwrap();
    same_layout(&layout(&spec("RPM x100", "gauge", 5.0)).unwrap(), &layout(&spec("RPM x100", "t-gauge-sub", 5.0)).unwrap());
    // the unknown name is the clear error
    assert_eq!(subset_font("t-never", "A").unwrap_err().code, "textFontNotLoaded");
    unload_font("t-gauge-sub").unwrap();
}

#[test]
fn base64_round_trip() {
    for n in 0..20usize {
        let b: Vec<u8> = (0..n).map(|i| (i * 37 + 11) as u8).collect();
        assert_eq!(base64_decode(&base64_encode(&b)).unwrap(), b);
    }
    assert_eq!(base64_encode(b"Man"), "TWFu");
    assert_eq!(base64_encode(b"Ma"), "TWE=");
    assert_eq!(base64_decode("data:font/ttf;base64,TWFu").unwrap(), b"Man");
    assert!(base64_decode("TW*u").is_err());
}

/// Exploratory: every font in `tests/fonts-full/` (whole files, kept out of the repository). Prints one line
/// per font; asserts that nothing panics and that outlines come out closed.
#[test]
fn full_fonts_report() {
    let dir = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fonts-full");
    let Ok(rd) = std::fs::read_dir(dir) else { return };
    let mut files: Vec<_> = rd.filter_map(|e| e.ok()).map(|e| e.path()).collect();
    files.sort();
    let mut report = String::new();
    for p in files {
        let bytes = std::fs::read(&p).unwrap();
        let fname = p.file_name().unwrap().to_string_lossy().to_string();
        let name = format!("t-full-{fname}");
        let t0 = std::time::Instant::now();
        let info = match load_font(&name, bytes.clone(), true) {
            Ok(i) => i,
            Err(e) => {
                report += &format!("FULL {fname}: LOAD FAILED {e:?}\n");
                continue;
            }
        };
        let t_load = t0.elapsed().as_millis();
        let s = "Hamburgefonstiv 0123456789 %&@\u{e9}\u{c0}\u{f1}";
        let t1 = std::time::Instant::now();
        let r = layout(&spec(s, &name, 10.0));
        let t_lay = t1.elapsed().as_millis();
        let (regions, warn, ok) = match &r {
            Ok(l) => (l.regions.len(), l.warnings.iter().map(|w| w.code).collect::<Vec<_>>().join("+"), closed(l)),
            Err(e) => (0, e.code().to_string(), false),
        };
        let f = font(&name).unwrap();
        // glyphs of A-Za-z0-9 whose contours cross each other (overlapping components: the nesting rule fails)
        let mut crossing = vec![];
        for ch in ('A'..='Z').chain('a'..='z').chain('0'..='9') {
            if let Some(g) = f.glyph_index(ch) {
                let ls = glyph_loops(&f, g, None, RUN_SPANS);
                let polys: Vec<Vec<P>> = ls.iter().map(|l| loop_polygon(l, 6)).collect();
                if !polys.iter().all(|p| cross_free(p)) {
                    crossing.push(ch);
                }
            }
        }
        let sub = subset_font(&name, s).map(|(b, _)| b.len()).map_err(|e| e.code);
        report += &(format!(
            "FULL {fname}: {} {} glyphs={} chars={} upm={} cap={} regions={regions} closed={ok} warn=[{warn}] crossing={crossing:?} subset={sub:?} load={t_load}ms layout={t_lay}ms info-warn={:?}",
            info.outlines,
            info.family.clone().unwrap_or_default(),
            info.glyphs,
            info.chars,
            info.units_per_em,
            info.cap_height,
            info.warnings.iter().map(|w| w.code).collect::<Vec<_>>()
        ) + "\n");
        assert!(r.is_ok() && ok, "{fname}: {:?}", r.err());
        unload_font(&name).unwrap();
    }
    if SHOW_REPORT {
        panic!("REPORT\n{report}");
    }
}
const SHOW_REPORT: bool = false;
