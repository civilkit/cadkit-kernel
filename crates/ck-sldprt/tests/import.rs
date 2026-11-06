//! Fixture imports (four small permissively licensed parts, see tests/fixtures/SOURCES.txt), container checks and a
//! fuzz-style robustness pass: truncated and corrupted copies of the fixtures must give errors or bodies, never a panic.

use ck_sldprt::container::{self, Kind, TransmitKind};
use ck_sldprt::{import_sldprt, import_sldprt_with, Limits};
use std::path::PathBuf;

fn fixtures() -> Vec<(String, Vec<u8>)> {
    let dir = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures");
    let mut out = vec![];
    for e in std::fs::read_dir(dir).unwrap() {
        let p = e.unwrap().path();
        if p.extension().map_or(false, |x| x == "sldprt") {
            out.push((p.file_stem().unwrap().to_string_lossy().to_string(), std::fs::read(&p).unwrap()));
        }
    }
    out.sort();
    assert_eq!(out.len(), 4);
    out
}

fn fixture(name: &str) -> Vec<u8> {
    fixtures().into_iter().find(|f| f.0 == name).unwrap().1
}

#[test]
fn container_versions_and_streams() {
    let lim = Limits::default();
    let modern = container::open(&fixture("sw2022_cube_10mm"), &lim).unwrap();
    assert_eq!(modern.kind, Kind::Modern);
    assert!(modern.streams.iter().all(|s| s.crc_ok));
    assert!(modern.stream("Contents/Config-0-Partition").is_some(), "{:?}", modern.streams.iter().map(|s| &s.name).collect::<Vec<_>>());
    assert!(modern.streams.len() > 20);
    let ole = container::open(&fixture("sw2011_cube_10mm_ole2"), &lim).unwrap();
    assert_eq!(ole.kind, Kind::Ole2);
    assert!(ole.stream("Config-0-Partition").is_some(), "{:?}", ole.streams.iter().map(|s| &s.name).collect::<Vec<_>>());
    for c in [&modern, &ole] {
        let ts = c.transmits(&lim).unwrap();
        assert!(ts.iter().any(|t| t.kind == TransmitKind::Partition && !t.stream.contains("Ghost") && t.data.starts_with(b"PS")));
    }
}

fn check_body(name: &str, vol: f64, faces: usize, bbox: [f64; 3]) {
    let imp = import_sldprt(&fixture(name)).unwrap_or_else(|e| panic!("{name}: {e}"));
    let b = &imp.body;
    assert_eq!(b.num_faces(), faces, "{name} faces");
    let mp = ck_topo::mass::mass_props(b);
    assert!((mp.volume - vol).abs() <= 1e-6 * vol, "{name}: volume {} want {vol}", mp.volume);
    let (lo, hi) = b.bbox();
    for (i, w) in bbox.iter().enumerate() {
        let d = [hi.x - lo.x, hi.y - lo.y, hi.z - lo.z][i];
        assert!((d - w).abs() < 1e-6, "{name}: bbox {i} {d} want {w}");
    }
    let issues = ck_check::validate(b, 1e-6);
    assert!(issues.is_empty(), "{name}: {issues:?}");
    // every face must tessellate
    let tess = ck_tess::mesh::tessellate(b);
    let mut per = vec![0usize; b.num_faces()];
    for &f in &tess.mesh.tri_face {
        if let Some(c) = per.get_mut(f as usize) {
            *c += 1;
        }
    }
    assert!(per.iter().all(|&c| c > 0), "{name}: faces without triangles: {per:?}");
    assert_eq!(imp.unit_scale, 1000.0);
}

#[test]
fn cube_modern() {
    check_body("sw2022_cube_10mm", 1000.0, 6, [10.0, 10.0, 10.0]);
}

#[test]
fn cube_ole2() {
    check_body("sw2011_cube_10mm_ole2", 1000.0, 6, [10.0, 10.0, 10.0]);
}

#[test]
fn cube_with_hole() {
    // 7 faces: the cylinder is one periodic face with a seam, plus the 6 box faces minus nothing: SolidWorks stores 7
    let v = 1000.0 - std::f64::consts::PI * 2.5 * 2.5 * 10.0;
    check_body("sw2022_cube_hole_5mm", v, 7, [10.0, 10.0, 10.0]);
}

#[test]
fn cone_apex() {
    let v = std::f64::consts::PI * 25.0 * 10.0 / 3.0;
    check_body("sw2022_cone", v, 2, [10.0, 10.0, 10.0]);
}

#[test]
fn not_a_part_is_a_structured_error() {
    for data in [vec![], b"hello".to_vec(), vec![0u8; 4096], b"PK\x03\x04 zip".to_vec(), [0xD0u8, 0xCF, 0x11, 0xE0, 0xA1, 0xB1, 0x1A, 0xE1].to_vec()] {
        let e = import_sldprt(&data).unwrap_err();
        assert!(e.code.starts_with("container."), "{e}");
    }
}

#[test]
fn limits_are_enforced() {
    let data = fixture("sw2022_cube_10mm");
    let lim = Limits { max_stream: 100, ..Limits::default() };
    assert!(import_sldprt_with(&data, &lim).is_err());
    let lim = Limits { max_file: 1000, ..Limits::default() };
    assert_eq!(import_sldprt_with(&data, &lim).unwrap_err().code, "limit.file");
    let lim = Limits { max_total: 1000, ..Limits::default() };
    assert!(import_sldprt_with(&data, &lim).is_err());
    let lim = Limits { max_faces: 2, ..Limits::default() };
    assert_eq!(import_sldprt_with(&data, &lim).unwrap_err().code, "limit.faces");
    let lim = Limits { max_nodes: 20, ..Limits::default() };
    assert!(import_sldprt_with(&data, &lim).is_err());
}

struct Rng(u64);
impl Rng {
    fn next(&mut self) -> u64 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 7;
        self.0 ^= self.0 << 17;
        self.0
    }
    fn below(&mut self, n: usize) -> usize {
        (self.next() % n.max(1) as u64) as usize
    }
}

/// Truncated, bit-flipped and zeroed copies of every fixture: no panic, and a structured error or a body.
#[test]
fn fuzz_no_panic() {
    let mut rng = Rng(0x9E37_79B9_7F4A_7C15);
    let lim = Limits { max_stream: 8 << 20, max_total: 32 << 20, ..Limits::default() };
    let mut errors = 0usize;
    let mut oks = 0usize;
    for (name, data) in fixtures() {
        let mut cases: Vec<Vec<u8>> = vec![];
        for k in 0..40 {
            cases.push(data[..data.len() * k / 40].to_vec()); // truncations
        }
        for _ in 0..60 {
            let mut d = data.clone();
            for _ in 0..1 + rng.below(8) {
                let i = rng.below(d.len());
                d[i] ^= 1 << rng.below(8);
            }
            cases.push(d);
        }
        for _ in 0..20 {
            let mut d = data.clone();
            let a = rng.below(d.len());
            let n = (1 + rng.below(512)).min(d.len() - a);
            for b in &mut d[a..a + n] {
                *b = 0;
            }
            cases.push(d);
        }
        for _ in 0..10 {
            // random garbage after a valid header
            let mut d = data[..64.min(data.len())].to_vec();
            for _ in 0..2048 {
                d.push(rng.next() as u8);
            }
            cases.push(d);
        }
        for d in cases {
            match import_sldprt_with(&d, &lim) {
                Ok(_) => oks += 1,
                Err(e) => {
                    errors += 1;
                    assert!(!e.code.is_empty(), "{name}");
                }
            }
        }
    }
    assert!(errors > 100, "fuzz produced only {errors} errors ({oks} ok)");
}

/// A hand-made modern container: one stored-DEFLATE block with a nibble-swapped name; a flipped payload byte must drop
/// the block (CRC mismatch), not yield a stream.
/// One modern-container block holding `payload` as a stored DEFLATE block (payload below 64 KiB).
fn block(name: &str, payload: &[u8]) -> Vec<u8> {
    let mut deflate = vec![0x01];
    deflate.extend((payload.len() as u16).to_le_bytes());
    deflate.extend((!(payload.len() as u16)).to_le_bytes());
    deflate.extend(payload);
    let swapped: Vec<u8> = name.bytes().map(|b| (b << 4) | (b >> 4)).collect();
    let mut v = container::MARKER.to_vec();
    v.extend(7u32.to_le_bytes());
    v.extend(ck_sldprt::inflate::crc32(payload).to_le_bytes());
    v.extend((deflate.len() as u32).to_le_bytes());
    v.extend((payload.len() as u32).to_le_bytes());
    v.extend((swapped.len() as u32).to_le_bytes());
    v.extend(swapped);
    v.extend(deflate);
    v
}

#[test]
fn synthetic_modern_container() {
    let mut file = vec![0, 0, 0, 0, 0, 0, 0, 4];
    file.extend(block("Contents/Test", b"hello world"));
    let c = container::open(&file, &Limits::default()).unwrap();
    assert_eq!(c.kind, Kind::Modern);
    assert_eq!(c.streams.len(), 1);
    assert_eq!(c.streams[0].name, "Contents/Test");
    assert_eq!(c.streams[0].data, b"hello world");
    let n = file.len();
    file[n - 3] ^= 0x40;
    assert_eq!(container::open(&file, &Limits::default()).unwrap_err().code, "container.empty");
    // a bomb: a stored block cannot expand, but the declared size is checked against the limit
    let lim = Limits { max_stream: 4, ..Limits::default() };
    let mut ok = vec![0, 0, 0, 0, 0, 0, 0, 4];
    ok.extend(block("Contents/Test", b"hello world"));
    assert!(container::open(&ok, &lim).is_err());
}

/// The corpus part (not committed: `<repo>/_corpus`, see ck-heal/tests/import_corpus.rs) whose four spline faces have a
/// vertex 2.5 mm off their surface: those faces are left out and named, the other faces come in (skipped without the corpus).
#[test]
fn a_face_that_cannot_be_built_is_left_out_and_named() {
    let p = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../_corpus/solidworks/samples/nist_ftc_07/part.SLDPRT");
    let Ok(bytes) = std::fs::read(&p) else { return };
    let imp = import_sldprt(&bytes).unwrap();
    assert_eq!(imp.parts.len(), 1);
    let part = &imp.parts[0];
    let ids: Vec<i64> = part.refused.iter().map(|r| r.0).collect();
    assert_eq!(ids, vec![5276, 5282, 5292, 5298]);
    assert!(part.refused.iter().all(|r| r.1.code == "geometry.invalid"));
    assert_eq!(part.faces.len() + 4, part.faces_total);
    assert_eq!(part.refused_surfaces.len(), 4);
}

/// A corpus part (not committed: `SW_CORPUS`, default `<repo>/corpus-local/samples`) whose rolling-ball blends run round
/// closed spines: B-spline faces closed in one parameter, bands between two ring edges and patches across the closing
/// isoline, and ring edges on blend boundaries. Every edge lies on its faces and the body has the file's volume.
#[test]
fn closed_spline_faces_and_blend_boundary_rings() {
    let dir = std::env::var("SW_CORPUS").map(PathBuf::from).unwrap_or_else(|_| PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../corpus-local/samples"));
    let Ok(bytes) = std::fs::read(dir.join("nist_ctc_02/part.SLDPRT")) else { return };
    let imp = import_sldprt(&bytes).unwrap();
    assert_eq!((imp.parts.len(), imp.parts[0].faces.len(), imp.parts[0].faces_total), (1, 488, 488));
    let issues = ck_check::validate(&imp.body, 1e-6);
    assert!(issues.is_empty(), "{:?}", issues.iter().take(5).map(|i| format!("{} {} {}", i.code, i.entity, i.detail)).collect::<Vec<_>>());
    let faces: Vec<_> = imp.body.face_ids().collect();
    let vol = ck_topo::mass::signed_volume(&imp.body, &faces);
    // SolidWorks' stored mass properties
    let file = imp.sw_mass.unwrap().volume_m3 * 1e9;
    assert!((vol - file).abs() < 1e-3 * file, "volume {vol} vs file {file}");
}

#[test]
fn a_complete_part_has_no_refused_faces() {
    let imp = import_sldprt(&fixture("sw2022_cube_10mm")).unwrap();
    assert_eq!(imp.parts.len(), 1);
    assert_eq!((imp.parts[0].faces_total, imp.parts[0].faces.len(), imp.parts[0].refused.len()), (6, 6, 0));
}

// ------------------------------------------------------------------ face tags and metadata

#[test]
fn face_colour_and_feature_tags() {
    // every face says which feature built it; the ids and names come from the feature list
    for (name, want) in [("sw2022_cube_hole_5mm", vec![("Boss-Extrude1", 6), ("Cut-Extrude1", 1)]), ("sw2011_cube_10mm_ole2", vec![("Boss-Extrude1", 6)]), ("sw2022_cone", vec![("Revolve1", 2)])] {
        let imp = import_sldprt(&fixture(name)).unwrap();
        assert_eq!(imp.face_tags.len(), imp.body.num_faces(), "{name}");
        let mut got: Vec<(String, usize)> = vec![];
        for t in &imp.face_tags {
            assert_eq!(t.face, format!("sldprt.F{}", t.xt_face), "{name}");
            assert!(imp.body.faces.iter().any(|f| f.name.id() == t.face), "{name}: {} is not a face of the body", t.face);
            let c = t.colour.unwrap_or_else(|| panic!("{name}: face {} has no colour", t.face));
            assert!(c.iter().all(|v| (0.0..=1.0).contains(v)), "{name}: {c:?}");
            let sw = t.sw_id.as_ref().unwrap();
            let f = imp.meta.feature(sw.feature).unwrap_or_else(|| panic!("{name}: feature id {} not in the feature list", sw.feature));
            assert_eq!(t.feature.as_deref(), Some(f.name.as_str()));
            match got.iter_mut().find(|g| g.0 == f.name) {
                Some(g) => g.1 += 1,
                None => got.push((f.name.clone(), 1)),
            }
        }
        got.sort();
        let want: Vec<(String, usize)> = want.into_iter().map(|(n, k)| (n.to_string(), k)).collect();
        assert_eq!(got, want, "{name}");
        assert_eq!(imp.body_tags.len(), 1, "{name}");
        assert!(imp.body_tags[0].density.map_or(false, |d| d > 0.0), "{name}: {:?}", imp.body_tags);
        // the last feature that changed the body is in the list
        assert!(imp.body_tags[0].last_feature.and_then(|f| imp.meta.feature(f)).is_some(), "{name}: {:?}", imp.body_tags);
    }
}

#[test]
fn metadata_of_both_containers() {
    let modern = import_sldprt(&fixture("sw2022_cube_hole_5mm")).unwrap();
    let m = &modern.meta;
    let feats: Vec<(i64, &str, &str)> = m.features.iter().filter(|f| !matches!(f.kind.as_str(), "Plane" | "Origin" | "3DPoint")).map(|f| (f.id, f.name.as_str(), f.kind.as_str())).collect();
    assert_eq!(feats, vec![(26, "Sketch1", "Sketch"), (32, "Boss-Extrude1", "Boss-Extrude"), (33, "Sketch2", "Sketch"), (46, "Cut-Extrude1", "Extrusion")]);
    let d = &m.feature(32).unwrap().dims;
    assert_eq!((d.len(), d[0].name.as_str(), d[0].text.as_str(), d[0].value), (1, "D1", "10", Some(10.0)));
    assert_eq!(m.configurations.len(), 1);
    assert_eq!((m.configurations[0].name.as_str(), m.configurations[0].material.as_deref()), ("Default", None));
    assert_eq!(m.material, None);
    let u = m.units.as_ref().unwrap();
    assert_eq!((u.system.as_deref(), u.length.as_deref(), u.angle.as_deref(), u.length_decimals), (Some("MMGS"), Some("mm"), Some("deg"), Some(2)));
    assert_eq!((m.sw_version, m.sw_release()), (Some(15000), Some(2022)));
    assert_eq!(modern.parasolid_version().map(|p| p.1), Some("33.01".to_string()));

    let ole = import_sldprt(&fixture("sw2011_cube_10mm_ole2")).unwrap();
    let m = &ole.meta;
    assert!(m.features.iter().any(|f| f.name == "Boss-Extrude1" && f.dims.first().map(|d| d.value) == Some(Some(10.0))), "{:?}", m.features);
    assert_eq!(m.configurations.first().map(|c| c.name.as_str()), Some("Default"));
    assert_eq!((m.sw_version, m.sw_release()), (Some(4700), Some(2011)));
    assert_eq!(ole.parasolid_version().map(|p| p.1), Some("22.01".to_string()));
    for imp in [&modern, &ole] {
        let t = imp.meta.thumbnail.as_ref().expect("PreviewPNG");
        assert_eq!((t.width, t.height, &t.png[1..4]), (640, 480, &b"PNG"[..]));
    }
}

/// The user names and folder the fixture records (read here from the raw streams, so the test names none): modern
/// `SW-Last Saved By`, `SW-Author`, `SW-Folder Name` and the core `lastModifiedBy`; OLE2 the summary set's author (4) and
/// last author (8).
pub fn recorded_private_strings(data: &[u8]) -> Vec<String> {
    let c = container::open(data, &Limits::default()).unwrap();
    let mut out = vec![];
    let after = |t: &str, key: &str| t.find(key).map(|p| &t[p + key.len()..]).map(|v| v[..v.find('<').unwrap_or(0)].to_string());
    if let Some(s) = c.stream("docProps/ISolidWorksInformation.xml") {
        let t = String::from_utf8_lossy(&s.data);
        for key in ["SW-Last Saved By", "SW-Author", "SW-Folder Name"] {
            if let Some(p) = t.find(&format!("name=\"{key}\"")) {
                out.extend(after(&t[p..], "<vt:lpstr>"));
            }
        }
    }
    if let Some(s) = c.stream("docProps/core.xml") {
        out.extend(after(&String::from_utf8_lossy(&s.data), "lastModifiedBy>"));
    }
    if let Some(s) = c.streams.iter().find(|s| s.name == "SummaryInformation") {
        let d = &s.data;
        let u32at = |p: usize| d.get(p..p + 4).map(|b| u32::from_le_bytes([b[0], b[1], b[2], b[3]]) as usize).unwrap_or(0);
        let base = u32at(44);
        for k in 0..u32at(base + 4).min(64) {
            let (pid, off) = (u32at(base + 8 + 8 * k), base + u32at(base + 12 + 8 * k));
            if (pid == 4 || pid == 8) && u32at(off) == 0x1E {
                let l = u32at(off + 4);
                out.extend(d.get(off + 8..off + 8 + l).map(|b| String::from_utf8_lossy(b).trim_end_matches('\0').to_string()));
            }
        }
    }
    out.retain(|s| !s.trim().is_empty());
    out
}

#[test]
fn no_user_name_or_path_in_the_metadata() {
    let mut checked = 0;
    for (name, data) in fixtures() {
        let imp = import_sldprt(&data).unwrap();
        let all = format!("{:?} {:?} {:?}", imp.meta, imp.face_tags, imp.body_tags).to_lowercase();
        for p in recorded_private_strings(&data) {
            assert!(!all.contains(&p.to_lowercase()), "{name}: a recorded user name or folder is in the metadata");
            checked += 1;
        }
        assert!(!all.contains(":\\\\"), "{name}: a path is in the metadata");
    }
    assert!(checked >= 5, "the fixtures record user names and folders ({checked} checked)");
}

/// A container with only metadata streams: user names and paths left out, missing streams empty, no panic.
#[test]
fn synthetic_metadata_privacy_and_missing_streams() {
    let info = r#"<?xml version="1.0"?><Properties xmlns:vt="x"><propertySection name="UserDefinedProperties"><property name="SW-Last Saved By" pid="3"><vt:lpstr>example-user</vt:lpstr></property><property name="SW-Folder Name" pid="4"><vt:lpstr>C:\example\</vt:lpstr></property><property name="SW_UnitSystem" pid="5"><vt:i4>3</vt:i4></property><property name="SW_UnitsLinear" pid="6"><vt:i4>3</vt:i4></property></propertySection></Properties>"#;
    let custom = r#"<?xml version="1.0"?><Properties xmlns:vt="x"><propertySection name="DocumentSummaryInformation"><property name="" pid="1"><vt:i2>65001</vt:i2></property></propertySection><propertySection name="UserDefinedProperties"><property name="Revision" pid="2"><vt:lpstr>B</vt:lpstr></property><property name="Drawn" pid="3"><vt:lpstr>Example-User</vt:lpstr></property><property name="Source" pid="4"><vt:lpstr>\\host\share\part.SLDPRT</vt:lpstr></property><property name="Weight" pid="5"><vt:lpstr>1.5 kg</vt:lpstr></property></propertySection></Properties>"#;
    let kw = "\u{86}<?xml version=\"1.0\"?><Keywords id=\"1\" Name=\"doc\"><Configuration id=\"0\" Name=\"Default\" Material=\"1060 Alloy\"/><Extrusion id=\"32\" Name=\"Boss-Extrude1\" Type=\"Boss-Extrude\"><Dimension Name=\"D1\">25.4</Dimension></Extrusion></Keywords>";
    let mut file = vec![0, 0, 0, 0, 0, 0, 0, 4];
    file.extend(block("docProps/ISolidWorksInformation.xml", info.as_bytes()));
    file.extend(block("docProps/custom.xml", custom.as_bytes()));
    file.extend(block("swXmlContents/KeyWords", kw.as_bytes()));
    file.extend(block("_MO_VERSION_11000/History", b"x"));
    let m = ck_sldprt::read_meta(&file).unwrap();
    assert_eq!(m.custom_properties, vec![("Revision".to_string(), "B".to_string()), ("Weight".to_string(), "1.5 kg".to_string())]);
    assert_eq!(m.omitted, 2);
    assert!(!format!("{m:?}").to_lowercase().contains("example-user") && !format!("{m:?}").contains("example\\"));
    let u = m.units.as_ref().unwrap();
    assert_eq!((u.system.as_deref(), u.length.as_deref(), u.angle.as_deref()), (Some("IPS"), Some("in"), None));
    assert_eq!(m.features.len(), 1);
    assert_eq!(m.features[0].dims[0].value, Some(25.4));
    assert_eq!(m.material.as_ref().map(|x| (x.name.as_str(), x.density_kg_m3)), Some(("1060 Alloy", None)));
    assert_eq!((m.sw_version, m.sw_release()), (Some(11000), Some(2018)));
    assert!(m.thumbnail.is_none() && m.mass.is_none() && m.config_properties.is_empty());
    // the import itself fails (no Parasolid), with a structured error
    assert_eq!(import_sldprt(&file).unwrap_err().code, "container.no_parasolid");
    // only one stream
    let mut one = vec![0, 0, 0, 0, 0, 0, 0, 4];
    one.extend(block("Contents/Other", b"nothing"));
    assert_eq!(ck_sldprt::read_meta(&one).unwrap(), ck_sldprt::SwMeta::default());
}

/// The last file read is remembered: a second import of the same bytes is the same body (and another file is not
/// mistaken for it), whichever order they come in.
#[test]
fn import_of_the_same_bytes_is_remembered() {
    let (cube, cone) = (fixture("sw2022_cube_10mm"), fixture("sw2022_cone"));
    let a = import_sldprt(&cube).unwrap();
    let b = import_sldprt(&cube).unwrap();
    assert_eq!((a.body.num_faces(), a.nodes, a.solids), (b.body.num_faces(), b.nodes, b.solids));
    assert_eq!(a.face_tags.len(), b.face_tags.len());
    let c = import_sldprt(&cone).unwrap();
    assert_ne!(c.body.num_faces(), a.body.num_faces());
    let d = import_sldprt(&cube).unwrap();
    assert_eq!(d.body.num_faces(), a.body.num_faces());
    // a changed byte is another file: the error of the damaged copy is not the remembered body
    let mut bad = cube.clone();
    let n = bad.len() / 2;
    bad[n] ^= 0xff;
    let _ = import_sldprt(&bad);
    assert_eq!(import_sldprt(&cube).unwrap().body.num_faces(), a.body.num_faces());
}
