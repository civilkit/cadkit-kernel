//! Deterministic byte-level mutation fuzz test for `ck_io::step::read_step`.
//!
//! Takes all `.step` fixtures under `tests/fixtures/` and `tests/golden/`,
//! applies fixed-seed byte flips, truncations, and cross-fixture splices,
//! and asserts that `read_step` never panics and never hangs.
use ck_io::step::{read_step, ImportError, Imported};
use std::path::PathBuf;
use std::sync::mpsc;
use std::time::Duration;

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

fn load_step_fixtures() -> Vec<(String, Vec<u8>)> {
    let base = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests");
    let mut paths = Vec::new();
    for sub in ["fixtures", "golden"] {
        for entry in std::fs::read_dir(base.join(sub)).unwrap() {
            let p = entry.unwrap().path();
            if p.extension().is_some_and(|ext| ext == "step") {
                paths.push(p);
            }
        }
    }
    paths.sort();
    assert!(!paths.is_empty(), "no .step fixtures found");
    paths
        .into_iter()
        .map(|p| {
            let name = p.file_name().unwrap().to_string_lossy().into_owned();
            let bytes = std::fs::read(&p).unwrap();
            (name, bytes)
        })
        .collect()
}

fn guarded(bytes: Vec<u8>) -> Result<Result<Imported, ImportError>, String> {
    let (tx, rx) = mpsc::channel();
    std::thread::Builder::new()
        .stack_size(8 << 20)
        .spawn(move || {
            let text = String::from_utf8_lossy(&bytes).into_owned();
            let r = std::panic::catch_unwind(|| read_step(&text));
            let _ = tx.send(r.map_err(|_| "panic".to_string()));
        })
        .unwrap();
    rx.recv_timeout(Duration::from_secs(10)).unwrap_or_else(|_| Err("hang".to_string()))
}

#[test]
fn byte_level_mutations_truncations_and_splices_never_panic_or_hang() {
    let fixtures = load_step_fixtures();
    let mut rng = Rng(0x5A17_9E37_79B9_7F4A);
    let mut failures = Vec::new();
    let mut cases = 0usize;

    // 1. Byte-level truncations across all fixtures (including mid-line and non-ASCII/arbitrary cuts).
    for (name, bytes) in &fixtures {
        let len = bytes.len();
        let mut cuts = vec![0usize, 1, len.saturating_sub(1), len / 2];
        for _ in 0..4 {
            cuts.push(rng.below(len + 1));
        }
        cuts.sort_unstable();
        cuts.dedup();
        for cut in cuts {
            cases += 1;
            if let Err(err) = guarded(bytes[..cut].to_vec()) {
                failures.push(format!("{name}: truncation@{cut}: {err}"));
            }
        }
    }

    // 2. Single- and multi-byte flips (raw bit flips and byte overwrites).
    for (name, bytes) in &fixtures {
        let len = bytes.len();
        if len == 0 {
            continue;
        }
        for round in 0..8 {
            let mut mutated = bytes.clone();
            let flips = 1 + rng.below(6);
            for _ in 0..flips {
                let idx = rng.below(len);
                if rng.below(2) == 0 {
                    mutated[idx] ^= (1 + rng.below(255)) as u8;
                } else {
                    mutated[idx] = (rng.next() & 0xFF) as u8;
                }
            }
            cases += 1;
            if let Err(err) = guarded(mutated) {
                failures.push(format!("{name}: byte-flip#{round}: {err}"));
            }
        }
    }

    // 3. Cross-fixture byte splices (prefix of A + suffix of B).
    let n = fixtures.len();
    for i in 0..n {
        for step in [1usize, 7] {
            let j = (i + step) % n;
            let (name_a, a) = &fixtures[i];
            let (name_b, b) = &fixtures[j];
            for round in 0..3 {
                let split_a = rng.below(a.len() + 1);
                let split_b = rng.below(b.len() + 1);
                let mut spliced = Vec::with_capacity(split_a + (b.len() - split_b));
                spliced.extend_from_slice(&a[..split_a]);
                spliced.extend_from_slice(&b[split_b..]);
                cases += 1;
                if let Err(err) = guarded(spliced) {
                    failures.push(format!("{name_a}[..{split_a}] + {name_b}[{split_b}..]: splice#{round}: {err}"));
                }
            }
        }
    }

    // 4. Line-aligned splices inside DATA sections (higher chance of reaching geometry/topology).
    for i in 0..n {
        let j = (i + 3) % n;
        let (name_a, a) = &fixtures[i];
        let (name_b, b) = &fixtures[j];
        let lines_a: Vec<usize> = a.iter().enumerate().filter_map(|(k, &c)| (c == b'\n').then_some(k + 1)).collect();
        let lines_b: Vec<usize> = b.iter().enumerate().filter_map(|(k, &c)| (c == b'\n').then_some(k + 1)).collect();
        if lines_a.is_empty() || lines_b.is_empty() {
            continue;
        }
        for round in 0..4 {
            let cut_a = lines_a[rng.below(lines_a.len())];
            let cut_b = lines_b[rng.below(lines_b.len())];
            let mut spliced = Vec::with_capacity(cut_a + (b.len() - cut_b));
            spliced.extend_from_slice(&a[..cut_a]);
            spliced.extend_from_slice(&b[cut_b..]);
            cases += 1;
            if let Err(err) = guarded(spliced) {
                failures.push(format!("{name_a}..@{cut_a} + {name_b}@{cut_b}..: line-splice#{round}: {err}"));
            }
        }
    }

    assert!(cases >= 300, "expected broad mutation coverage, got {cases} cases");
    assert!(
        failures.is_empty(),
        "{} of {cases} mutated STEP cases failed:\n{}",
        failures.len(),
        failures.join("\n")
    );
}
