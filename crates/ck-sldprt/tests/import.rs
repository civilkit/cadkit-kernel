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
#[test]
fn synthetic_modern_container() {
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
