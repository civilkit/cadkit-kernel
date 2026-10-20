//! Untrusted input: seeded byte flips, truncations, splices and zero fills of the fixtures (and of the inflated
//! `model.dat` of modern files, re-wrapped in a valid stored ZIP so the mutations reach the TLV walker) must give a model or
//! a structured error, never a panic, a hang or a runaway allocation. Deterministic (fixed seeds); a few seconds.
//!
//! Run longer with `CK_SKP_FUZZ_ITERS=20000 cargo test -p ck-skp --release --test robustness`.

use ck_skp::body::{build_body, BodyOptions};
use ck_skp::inflate::crc32;
use ck_skp::zip::{find_local_header, Zip};
use ck_skp::{bake, parse, parse_with, BakeOptions, Limits};
use std::panic::{catch_unwind, AssertUnwindSafe};
use std::path::PathBuf;
use std::time::{Duration, Instant};

struct Rng(u64);
impl Rng {
    fn next(&mut self) -> u64 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 7;
        self.0 ^= self.0 << 17;
        self.0
    }
    fn below(&mut self, n: usize) -> usize {
        if n == 0 {
            0
        } else {
            (self.next() % n as u64) as usize
        }
    }
}

fn fixtures() -> Vec<(String, Vec<u8>)> {
    let dir = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures");
    let mut v: Vec<_> = std::fs::read_dir(dir).unwrap().filter_map(|e| e.ok()).map(|e| e.path()).filter(|p| p.extension().map_or(false, |x| x == "skp")).map(|p| (p.file_name().unwrap().to_string_lossy().into_owned(), std::fs::read(&p).unwrap())).collect();
    v.sort();
    v
}

fn mutate(d: &[u8], r: &mut Rng) -> Vec<u8> {
    let mut m = d.to_vec();
    if m.is_empty() {
        return m;
    }
    match r.below(7) {
        0 | 1 => {
            for _ in 0..1 + r.below(8) {
                let i = r.below(m.len());
                m[i] ^= 1 << r.below(8);
            }
        }
        2 => {
            for _ in 0..1 + r.below(4) {
                let i = r.below(m.len());
                m[i] = [0x00, 0xFF, 0x7F, 0x80][r.below(4)];
            }
        }
        3 => {
            let n = r.below(m.len());
            m.truncate(n);
        }
        4 => {
            let a = r.below(m.len());
            let l = r.below(64).min(m.len() - a);
            for b in &mut m[a..a + l] {
                *b = 0;
            }
        }
        5 => {
            // splice a block from elsewhere
            let (a, b) = (r.below(m.len()), r.below(m.len()));
            let l = r.below(128).min(m.len() - a).min(m.len() - b);
            let chunk: Vec<u8> = m[a..a + l].to_vec();
            m[b..b + l].copy_from_slice(&chunk);
        }
        _ => {
            // a 32-bit field set to a huge value (counts, lengths)
            if m.len() >= 4 {
                let i = r.below(m.len() - 3);
                m[i..i + 4].copy_from_slice(&[0xFF, 0xFF, 0xFF, [0xFF, 0x7F, 0x00, 0x80][r.below(4)]]);
            }
        }
    }
    m
}

/// Stored (uncompressed) ZIP of `files`, with valid CRCs.
fn stored_zip(files: &[(String, Vec<u8>)]) -> Vec<u8> {
    let mut out = vec![];
    let mut cd = vec![];
    for (name, data) in files {
        let off = out.len() as u32;
        let crc = crc32(data);
        out.extend(0x0403_4B50u32.to_le_bytes());
        out.extend([20, 0, 0, 0, 0, 0, 0, 0, 0, 0]);
        out.extend(crc.to_le_bytes());
        out.extend((data.len() as u32).to_le_bytes());
        out.extend((data.len() as u32).to_le_bytes());
        out.extend((name.len() as u16).to_le_bytes());
        out.extend([0, 0]);
        out.extend(name.as_bytes());
        out.extend(data);
        cd.extend(0x0201_4B50u32.to_le_bytes());
        cd.extend([20, 0, 20, 0, 0, 0, 0, 0, 0, 0, 0, 0]);
        cd.extend(crc.to_le_bytes());
        cd.extend((data.len() as u32).to_le_bytes());
        cd.extend((data.len() as u32).to_le_bytes());
        cd.extend((name.len() as u16).to_le_bytes());
        cd.extend([0u8; 12]);
        cd.extend(off.to_le_bytes());
        cd.extend(name.as_bytes());
    }
    let cd_off = out.len() as u32;
    out.extend(&cd);
    out.extend(0x0605_4B50u32.to_le_bytes());
    out.extend([0, 0, 0, 0]);
    out.extend((files.len() as u16).to_le_bytes());
    out.extend((files.len() as u16).to_le_bytes());
    out.extend((cd.len() as u32).to_le_bytes());
    out.extend(cd_off.to_le_bytes());
    out.extend([0, 0]);
    out
}

/// A modern file whose `model.dat` has been replaced by `f(model.dat)`.
fn rewrap_vff(d: &[u8], f: &mut dyn FnMut(&[u8]) -> Vec<u8>) -> Vec<u8> {
    let pk = find_local_header(d, 0).unwrap();
    let lim = Limits::default();
    let z = Zip::open(&d[pk..], &lim).unwrap();
    let mut files = vec![];
    for e in &z.entries {
        let data = z.read(e, &lim).unwrap();
        files.push((e.name.clone(), if e.name == "model.dat" { f(&data) } else { data }));
    }
    let mut out = d[..pk].to_vec();
    out.extend(stored_zip(&files));
    out
}

fn run_case(name: &str, case: usize, data: &[u8]) -> Duration {
    let t = Instant::now();
    let r = catch_unwind(AssertUnwindSafe(|| {
        if let Ok(m) = parse(data) {
            if let Ok(s) = bake(&m, &Limits::default(), &BakeOptions { include_hidden: true, keep_polygons: true, max_triangles: 2_000_000, max_instances: 200_000, ..Default::default() }) {
                if s.polygons.len() < 5000 {
                    let _ = build_body(&s, &BodyOptions::default());
                }
                let _ = ck_skp::export::to_glb(&s, true).len();
            }
        }
    }));
    assert!(r.is_ok(), "{name}: case {case} panicked (len {})", data.len());
    let dt = t.elapsed();
    assert!(dt < Duration::from_secs(20), "{name}: case {case} took {dt:?}");
    dt
}

fn iters(default: usize) -> usize {
    std::env::var("CK_SKP_FUZZ_ITERS").ok().and_then(|v| v.parse().ok()).unwrap_or(default)
}

#[test]
fn mutated_fixtures_never_panic_or_hang() {
    let mut worst = Duration::ZERO;
    let mut total = 0;
    for (name, data) in fixtures() {
        let n = if data.len() > 100_000 { iters(12) / 8 + 2 } else { iters(150) };
        let mut r = Rng(0x9E37_79B9_7F4A_7C15 ^ (name.len() as u64 * 7919) ^ data.len() as u64);
        for case in 0..n {
            let m = mutate(&data, &mut r);
            worst = worst.max(run_case(&name, case, &m));
            total += 1;
        }
    }
    eprintln!("{total} mutated files, slowest {worst:?}");
}

#[test]
fn every_prefix_of_small_files() {
    for (name, data) in fixtures() {
        if data.len() > 20_000 {
            continue;
        }
        let step = (data.len() / 400).max(1);
        for n in (0..data.len()).step_by(step) {
            run_case(&name, n, &data[..n]);
        }
    }
}

#[test]
fn mutated_model_dat_reaches_the_tlv_walker() {
    for (name, data) in fixtures() {
        if !data.starts_with(&[0xFF, 0xFE, 0xFF, 0x0E]) || find_local_header(&data, 0).is_none() {
            continue;
        }
        // only the modern files carry a ZIP
        if ck_skp::parse(&data).map(|m| m.container) != Ok(ck_skp::Container::Vff) {
            continue;
        }
        let n = if data.len() > 100_000 { iters(10) } else { iters(120) };
        let mut r = Rng(0xD1B5_4A32_D192_ED03 ^ data.len() as u64);
        for case in 0..n {
            let w = rewrap_vff(&data, &mut |md| mutate(md, &mut r));
            run_case(&name, case, &w);
        }
    }
}

#[test]
fn hostile_constructions() {
    // header only, magic only, empty
    for d in [&b""[..], &[0xFF, 0xFE, 0xFF, 0x0E][..], &[0xFF, 0xFE, 0xFF, 0x0E, 0, 0, 0, 0, 0, 0, 0xFF, 0xFE, 0xFF, 6][..]] {
        assert!(parse(d).is_err());
    }
    // TLV nested far deeper than the limit
    let (_, any) = fixtures().into_iter().find(|f| f.0 == "SU_File.skp").unwrap();
    let w = rewrap_vff(&any, &mut |_| {
        let mut d = vec![0x12, 0x34, 0, 0, 0, 0];
        for _ in 0..100_000 {
            let mut o = vec![0xC4, 0x09];
            o.extend((d.len() as u32).to_le_bytes());
            o.extend(&d);
            d = o;
            if d.len() > 2_000_000 {
                break;
            }
        }
        d
    });
    let e = parse(&w).unwrap_err();
    assert!(e.code == "limit.depth" || e.code == "limit.nodes", "{e}");
    // a lot of tiny sibling records: bounded by the node limit
    let lim = Limits { max_nodes: 1000, ..Default::default() };
    let w = rewrap_vff(&any, &mut |_| {
        let mut d = vec![];
        for _ in 0..5000 {
            d.extend([0xF4, 0x01, 6, 0, 0, 0, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF]);
        }
        d
    });
    assert!(parse_with(&w, &lim).is_err());
    // a ZIP entry that claims a huge size
    let mut z = stored_zip(&[("model.dat".into(), vec![1, 2, 3])]);
    let cd = z.windows(4).position(|w| w == [0x50, 0x4B, 0x01, 0x02]).unwrap();
    z[cd + 24..cd + 28].copy_from_slice(&u32::MAX.to_le_bytes());
    let mut file = vec![0xFF, 0xFE, 0xFF, 0x0E, 0, 0, 0, 0, 0, 0, 0xFF, 0xFE, 0xFF, 6];
    file.extend(&z);
    let e = parse(&file).unwrap_err();
    assert!(e.code.starts_with("limit.") || e.code.starts_with("zip."), "{e}");
    // file size limit
    let lim = Limits { max_file: 100, ..Default::default() };
    assert_eq!(parse_with(&any, &lim).unwrap_err().code, "limit.file");
}

#[test]
fn legacy_nesting_and_counts_are_bounded() {
    // a classic file whose model part is replaced by garbage that keeps the anchors: must fail cleanly
    let (_, d) = fixtures().into_iter().find(|f| f.0 == "capilla_quiroz_v17.skp").unwrap();
    let mut r = Rng(42);
    for case in 0..iters(60) {
        let mut m = d.clone();
        let from = 0x200 + r.below(m.len() - 0x200);
        for b in m[from..].iter_mut().take(256) {
            *b = (r.next() & 0xFF) as u8;
        }
        run_case("capilla garbage tail", case, &m);
    }
}
