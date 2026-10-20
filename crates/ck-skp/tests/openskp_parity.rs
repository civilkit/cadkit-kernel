//! Parity with OpenSKP. The expected numbers are what OpenSKP's own Python package reports for each fixture (entity totals
//! over all definitions and the root; baked primitives, materials, triangle count, surface area and bounding box in mm,
//! Z up) plus the values asserted by OpenSKP's own tests (packages/cpp/tests/{parser,legacy_v20,legacy_v3_format}_test.cpp).
//! `tools/openskp_ref.py` regenerates the table from a clone of OpenSKP.
//!
//! The whole OpenSKP tree (80 files, 26 distinct) is run when `OPENSKP_FIXTURES=<clone dir>` is set: every file must parse.

use ck_skp::{bake, parse, BakeOptions, Container, Limits};
use std::path::{Path, PathBuf};

struct Exp {
    file: &'static str,
    defs: usize,
    v: usize,
    e: usize,
    f: usize,
    i: usize,
    prims: usize,
    mats: usize,
    tris: usize,
    area: f64,
    bbox: Option<[f64; 6]>,
}

const EXPECTED: &[Exp] = &[
    Exp { file: "chair_and_table.skp", defs: 9, v: 144, e: 200, f: 86, i: 15, prims: 38, mats: 6, tris: 704, area: 5498836.484, bbox: Some([-685.800, -254.000, 0.000, 558.800, 254.000, 863.600]) },
    Exp { file: "SU_File.skp", defs: 0, v: 56, e: 84, f: 32, i: 0, prims: 1, mats: 1, tris: 104, area: 13746324.455, bbox: Some([-607.232, -2951.215, -0.000, 1000.000, 0.000, 1000.000]) },
    Exp { file: "Untitled.skp", defs: 46, v: 11216, e: 12762, f: 1588, i: 46, prims: 43, mats: 8, tris: 18560, area: 92762069.224, bbox: Some([118.805, -2059.068, -0.000, 13535.744, 5188.080, 2820.000]) },
    Exp { file: "blank_v17.skp", defs: 0, v: 0, e: 0, f: 0, i: 0, prims: 0, mats: 0, tris: 0, area: 0.000, bbox: None },
    Exp { file: "capilla_quiroz_v17.skp", defs: 2, v: 335, e: 515, f: 181, i: 3, prims: 21, mats: 13, tris: 937, area: 254654677.309, bbox: Some([-8298.663, 0.000, -594.839, 19924.860, 5644.155, 3707.783]) },
    Exp { file: "gondola_v20.skp", defs: 20, v: 6553, e: 9186, f: 1887, i: 53, prims: 201, mats: 16, tris: 57432, area: 248630795.466, bbox: Some([46168.915, 3323.771, -0.000, 49988.522, 8140.882, 3140.218]) },
    Exp { file: "legacy_annotations.skp", defs: 0, v: 4, e: 4, f: 1, i: 0, prims: 1, mats: 1, tris: 2, area: 6451599.806, bbox: Some([0.000, 0.000, 0.000, 2540.000, 2540.000, 0.000]) },
    Exp { file: "legacy_v3_synthetic.skp", defs: 4, v: 16, e: 16, f: 4, i: 11, prims: 17, mats: 4, tris: 34, area: 3903218.317, bbox: Some([0.000, 0.000, 0.000, 9652.000, 762.000, 0.000]) },
    Exp { file: "legacy_v3_textured.skp", defs: 0, v: 4, e: 4, f: 1, i: 0, prims: 2, mats: 2, tris: 4, area: 516128.033, bbox: Some([0.000, 0.000, 0.000, 508.000, 508.000, 0.000]) },
    Exp { file: "legacy_v6_synthetic.skp", defs: 4, v: 16, e: 16, f: 4, i: 11, prims: 17, mats: 4, tris: 34, area: 3903218.317, bbox: Some([0.000, 0.000, 0.000, 9652.000, 762.000, 0.000]) },
    Exp { file: "single_material_v17.skp", defs: 0, v: 0, e: 0, f: 0, i: 0, prims: 0, mats: 0, tris: 0, area: 0.000, bbox: None },
    Exp { file: "zero_material_custom_layer_v18.skp", defs: 0, v: 0, e: 0, f: 0, i: 0, prims: 0, mats: 0, tris: 0, area: 0.000, bbox: None },
    Exp { file: "circle.skp", defs: 0, v: 12, e: 12, f: 1, i: 0, prims: 1, mats: 1, tris: 10, area: 3096768.044, bbox: Some([254.000, 254.000, 0.000, 2286.000, 2286.000, 0.000]) },
    Exp { file: "holes.skp", defs: 0, v: 8, e: 8, f: 1, i: 0, prims: 1, mats: 1, tris: 8, area: 11870943.607, bbox: Some([0.000, 0.000, 0.000, 5080.000, 2540.000, 0.000]) },
    Exp { file: "multi_face_shared_edge.skp", defs: 0, v: 6, e: 7, f: 2, i: 0, prims: 1, mats: 1, tris: 4, area: 12903199.612, bbox: Some([0.000, 0.000, 0.000, 5080.000, 2540.000, 0.000]) },
    Exp { file: "legacy_v2014_synthetic.skp", defs: 4, v: 16, e: 16, f: 4, i: 11, prims: 17, mats: 4, tris: 34, area: 3903218.317, bbox: Some([0.000, 0.000, 0.000, 9652.000, 762.000, 0.000]) },
    Exp { file: "legacy_v7_synthetic.skp", defs: 4, v: 16, e: 16, f: 4, i: 11, prims: 17, mats: 4, tris: 34, area: 3903218.317, bbox: Some([0.000, 0.000, 0.000, 9652.000, 762.000, 0.000]) },
];

fn fixture_dir() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures")
}

fn load(name: &str) -> Vec<u8> {
    std::fs::read(fixture_dir().join(name)).unwrap_or_else(|e| panic!("{name}: {e}"))
}

fn check_against_openskp(name: &str, data: &[u8], e: &Exp) {
    let m = parse(data).unwrap_or_else(|er| panic!("{name}: {er}"));
    let (v, ed, f, i) = m.totals();
    assert_eq!((m.definitions.len(), v, ed, f, i), (e.defs, e.v, e.e, e.f, e.i), "{name}: entity totals");
    let opt = BakeOptions { include_hidden: true, openskp_compat: true, ..Default::default() };
    let s = bake(&m, &Limits::default(), &opt).unwrap_or_else(|er| panic!("{name}: bake {er}"));
    assert_eq!(s.primitives.len(), e.prims, "{name}: primitives");
    assert_eq!(s.materials.len(), e.mats, "{name}: scene materials");
    // triangle counts differ by degenerate (collinear) triangles between ear clippers: 1 percent
    let t = s.triangle_count() as f64;
    assert!((t - e.tris as f64).abs() <= 0.01 * (e.tris as f64).max(10.0), "{name}: triangles {t} vs {}", e.tris);
    // areas of non-planar faces depend on the triangulation: 0.5 percent
    assert!((s.area() - e.area).abs() <= 5e-3 * e.area.max(1.0), "{name}: area {} vs {}", s.area(), e.area);
    match (s.bbox(), e.bbox) {
        (None, None) => {}
        (Some((lo, hi)), Some(b)) => {
            for k in 0..3 {
                assert!((lo[k] - b[k]).abs() < 0.05 && (hi[k] - b[k + 3]).abs() < 0.05, "{name}: bbox {lo:?} {hi:?} vs {b:?}");
            }
        }
        (a, b) => panic!("{name}: bbox {a:?} vs {b:?}"),
    }
}

#[test]
fn committed_fixtures_match_openskp() {
    for e in EXPECTED {
        check_against_openskp(e.file, &load(e.file), e);
    }
    // every committed fixture is in the table
    let n = std::fs::read_dir(fixture_dir()).unwrap().filter(|x| x.as_ref().unwrap().path().extension().map_or(false, |e| e == "skp")).count();
    assert_eq!(n, EXPECTED.len());
}

#[test]
fn containers() {
    for (f, c) in [("SU_File.skp", Container::Vff), ("Untitled.skp", Container::Vff), ("gondola_v20.skp", Container::Legacy), ("legacy_v3_synthetic.skp", Container::Legacy)] {
        assert_eq!(parse(&load(f)).unwrap().container, c, "{f}");
    }
}

// ---- values asserted by OpenSKP's own tests

#[test]
fn untitled_modern() {
    let m = parse(&load("Untitled.skp")).unwrap();
    assert_eq!(m.version, "{25.0.575}");
    assert_eq!(m.units.as_deref(), Some("Millimeter"));
    assert_eq!(m.layers.len(), 14);
    assert_eq!(m.materials.len(), 15);
    assert_eq!(m.definitions.len(), 46);
    let order: Vec<&str> = m.layers.iter().map(|l| l.name.as_str()).collect();
    assert_eq!(order, ["Layer0", "BottomPlate", "TopPlate", "Stud", "Nog", "KingStud", "HeaderJackStud", "HeaderPlate1", "HeaderPlate2", "SillPlate1", "VerticalHeaderStud", "generic_frame", "dimension", "Hat Sections"]);
    assert!(m.layers.iter().all(|l| !l.hidden));
    let d = m.definitions.get(66).expect("definition 66");
    assert_eq!(d.name, "Group200#2");
    assert_eq!(d.guid.len(), 32);
    assert_eq!((d.vertices.len(), d.edges.len(), d.faces.len()), (136, 158, 26));
    assert!(!d.is_image && !d.always_faces_camera);
    let mat = m.resolve_material(Some(26180)).expect("material id 26180");
    assert_eq!(mat.name, "*");
    let lm = m.materials.iter().find(|x| x.name == "Layer_Layer0").unwrap();
    assert_eq!(lm.opacity, 1.0);
    // loops are connected: the end of each coedge is the start of the next
    for f in d.faces.values() {
        for lp in &f.loops {
            let ends: Vec<(u64, u64)> = lp.iter().map(|&(e, o)| { let ed = d.edges.get(e).unwrap(); if o == 1 { (ed.v1, ed.v2) } else { (ed.v2, ed.v1) } }).collect();
            for k in 0..ends.len() {
                assert_eq!(ends[k].1, ends[(k + 1) % ends.len()].0);
            }
        }
    }
}

#[test]
fn gondola_v20() {
    let m = parse(&load("gondola_v20.skp")).unwrap();
    assert_eq!(m.version, "{20.1.235}");
    assert_eq!(m.units, None);
    assert_eq!(m.definitions.len(), 20);
    assert_eq!(m.materials.len(), 24);
    let mut names: Vec<&str> = m.layers.iter().map(|l| l.name.as_str()).collect();
    names.sort();
    assert_eq!(names, ["Gondulas Laterais", "Layer0"]);
    let (mut f, mut e, mut v) = (0, 0, 0);
    for d in m.definitions.values() {
        f += d.faces.len();
        e += d.edges.len();
        v += d.vertices.len();
    }
    assert_eq!((f, e, v), (1887, 9174, 6543));
    assert_eq!(m.root.instances.len(), 23);
    let s = bake(&m, &Limits::default(), &BakeOptions { include_hidden: true, openskp_compat: true, ..Default::default() }).unwrap();
    assert_eq!(s.primitives.len(), 201);
    assert!(!s.materials.is_empty());
}

#[test]
fn legacy_v3_and_v6() {
    let v3 = parse(&load("legacy_v3_synthetic.skp")).unwrap();
    let v6 = parse(&load("legacy_v6_synthetic.skp")).unwrap();
    assert_eq!(v3.version, "{3.0.1}");
    let by_name = |m: &ck_skp::Model, n: &str| m.definitions.values().find(|d| d.name == n).unwrap().clone();
    for n in ["InnerFrame", "Frame", "Panel", "Post"] {
        let d = by_name(&v3, n);
        assert_eq!((d.faces.len(), d.edges.len(), d.vertices.len()), (1, 4, 4), "{n}");
    }
    assert_eq!(v3.definitions.len(), 4);
    let frame = by_name(&v3, "Frame");
    let mut xs: Vec<f64> = frame.vertices.values().map(|p| p[0]).collect();
    let mut ys: Vec<f64> = frame.vertices.values().map(|p| p[1]).collect();
    xs.sort_by(|a, b| a.partial_cmp(b).unwrap());
    ys.sort_by(|a, b| a.partial_cmp(b).unwrap());
    xs.dedup();
    ys.dedup();
    assert_eq!((xs, ys), (vec![0.0, 20.0], vec![0.0, 20.0]));
    let names: Vec<String> = v3.root.instances.iter().map(|i| v3.definitions.get(i.def.unwrap()).unwrap().name.clone()).collect();
    assert_eq!(names, ["Frame", "Panel", "Post", "Frame", "Panel", "Post", "Frame", "Panel", "Post", "Frame"]);
    let panel = by_name(&v3, "Panel");
    assert_eq!(panel.instances.len(), 1);
    assert_eq!(v3.definitions.get(panel.instances[0].def.unwrap()).unwrap().name, "InnerFrame");
    // the painted face takes its material from the face's own pointer
    let painted = frame.faces.values().next().unwrap();
    let red = v3.resolve_material(painted.material_id).expect("Red");
    assert_eq!((red.name.as_str(), red.rgba[0], red.rgba[1], red.rgba[2]), ("Red", 255, 0, 0));
    for n in ["Panel", "Post", "InnerFrame"] {
        assert!(by_name(&v3, n).faces.values().next().unwrap().material_id.is_none(), "{n}");
        assert!(by_name(&v3, n).faces.values().all(|f| !f.hidden));
    }
    // v6 carries the same model
    assert_eq!(v3.definitions.len(), v6.definitions.len());
    for n in ["InnerFrame", "Frame", "Panel", "Post"] {
        let (a, b) = (by_name(&v3, n), by_name(&v6, n));
        assert_eq!((a.faces.len(), a.edges.len(), a.vertices.len()), (b.faces.len(), b.edges.len(), b.vertices.len()));
    }
    assert_eq!(v3.root.instances.len(), v6.root.instances.len());
    for (a, b) in v3.root.instances.iter().zip(&v6.root.instances) {
        assert_eq!(a.matrix, b.matrix);
    }
}

#[test]
fn legacy_textured_v3() {
    let m = parse(&load("legacy_v3_textured.skp")).unwrap();
    assert_eq!(m.version, "{3.0.1}");
    assert_eq!(m.materials.len(), 1);
    assert_eq!(m.materials[0].name, "Checker");
    let t = m.materials[0].texture.as_ref().expect("texture");
    let d = t.data.as_ref().expect("image bytes");
    assert_eq!(&d[..8], b"\x89PNG\r\n\x1a\n");
    assert_eq!(&d[d.len() - 8..], &[0x49, 0x45, 0x4E, 0x44, 0xAE, 0x42, 0x60, 0x82]);
    let f = m.root.faces.values().next().unwrap();
    assert_eq!(m.resolve_material(f.material_id).unwrap().name, "Checker");
    let s = bake(&m, &Limits::default(), &BakeOptions::default()).unwrap();
    assert_eq!(s.textures.len(), 1);
    assert_eq!(s.textures[0].mime, "image/png");
    assert!(s.materials.iter().any(|x| x.texture == Some(0)));
}

#[test]
fn legacy_small_files() {
    let m = parse(&load("single_material_v17.skp")).unwrap();
    assert_eq!(m.version, "{17.0.1}");
    assert_eq!(m.materials.len(), 1);
    assert_eq!(m.materials[0].name, "RedMat");
    assert_eq!(m.layers.len(), 1);
    assert_eq!(m.layers[0].name, "Layer0");
    let m = parse(&load("blank_v17.skp")).unwrap();
    assert_eq!(m.layers.len(), 1);
    assert_eq!(m.layers[0].name, "Layer0");
    let m = parse(&load("zero_material_custom_layer_v18.skp")).unwrap();
    assert_eq!(m.version, "{18.0.16975}");
    assert!(m.materials.is_empty());
    assert!(m.layers.len() >= 2);
    assert!(m.layers.iter().any(|l| l.name == "Layer0") && m.layers.iter().any(|l| l.name == "Guide"));
    assert!(m.root.faces.is_empty());
}

#[test]
fn holes_keep_their_loops() {
    let m = parse(&load("holes.skp")).unwrap();
    let f = m.root.faces.values().next().unwrap();
    assert!(f.loops.len() >= 2, "outer loop plus at least one hole");
    let s = bake(&m, &Limits::default(), &BakeOptions { keep_polygons: true, ..Default::default() }).unwrap();
    assert_eq!(s.polygons.len(), 1);
    assert_eq!(s.polygons[0].loops.len(), f.loops.len());
}

// ---- the whole OpenSKP tree, when available

fn collect(p: &Path, out: &mut Vec<PathBuf>) {
    if p.is_dir() {
        let mut v: Vec<_> = std::fs::read_dir(p).unwrap().filter_map(|e| e.ok().map(|e| e.path())).collect();
        v.sort();
        for c in v {
            collect(&c, out);
        }
    } else if p.extension().map_or(false, |e| e == "skp") {
        out.push(p.to_path_buf());
    }
}

/// `OPENSKP_FIXTURES=/path/to/openskp-clone cargo test -p ck-skp --release -- --nocapture openskp_tree`
#[test]
fn openskp_tree() {
    let Ok(root) = std::env::var("OPENSKP_FIXTURES") else {
        eprintln!("OPENSKP_FIXTURES not set: skipping the full OpenSKP tree");
        return;
    };
    let mut files = vec![];
    collect(Path::new(&root), &mut files);
    assert!(!files.is_empty(), "no .skp under {root}");
    let (mut ok, mut failed) = (0, vec![]);
    println!("{:<60} {:>7} {:>7} {:>8} {:>8} {:>6} {:>7}", "file", "kind", "defs", "verts", "faces", "inst", "tris");
    for f in &files {
        let data = std::fs::read(f).unwrap();
        let rel = f.strip_prefix(&root).unwrap_or(f).display().to_string();
        match parse(&data) {
            Err(e) => {
                println!("{rel:<60} FAIL {e}");
                failed.push(rel);
            }
            Ok(m) => {
                let (v, _, fc, i) = m.totals();
                let s = bake(&m, &Limits::default(), &BakeOptions { include_hidden: true, openskp_compat: true, ..Default::default() }).unwrap();
                println!("{rel:<60} {:>7} {:>7} {:>8} {:>8} {:>6} {:>7}", format!("{:?}", m.container), m.definitions.len(), v, fc, i, s.triangle_count());
                // a committed fixture with the same name must agree with the table
                if let Some(e) = EXPECTED.iter().find(|e| Path::new(&rel).file_name().map_or(false, |n| n == e.file)) {
                    check_against_openskp(&rel, &data, e);
                }
                ok += 1;
            }
        }
    }
    println!("parsed {ok} of {} files", files.len());
    assert!(failed.is_empty(), "failed: {failed:?}");
}
