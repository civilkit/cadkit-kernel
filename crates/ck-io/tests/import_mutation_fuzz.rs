//! Mutation fuzz of the STEP and SAT readers (solidcore #2753, #2745, #2460, #2674 class).
//!
//! Every numeric token of a real file is replaced, one at a time, by a hostile value: infinity
//! (`1.E999`), a huge or negative integer, zero, a tiny and a huge real. The reader must return
//! `Ok` or an `Err`; it must not panic, and must not run for long. An allocation failure aborts
//! the whole test binary, which is also a failure.
use std::time::{Duration, Instant};

const REALS: &[&str] = &["1.E999", "-1.E999", "1.E300", "-1.E300", "1.E-300", "0.", "-1."];
const INTS: &[&str] = &["0", "4294967295", "4294967296", "9223372036854775807", "18446744073709551615", "-1"];

/// Byte ranges of numeric tokens (digits with optional sign, point and exponent) that stand alone.
fn numbers(text: &str) -> Vec<(usize, usize, bool)> {
    let b = text.as_bytes();
    let mut out = Vec::new();
    let mut i = 0;
    while i < b.len() {
        let c = b[i];
        let starts = c.is_ascii_digit() || ((c == b'-' || c == b'+' || c == b'.') && b.get(i + 1).is_some_and(|d| d.is_ascii_digit() || *d == b'.'));
        let prev_ok = i == 0 || !(b[i - 1].is_ascii_alphanumeric() || b[i - 1] == b'_' || b[i - 1] == b'#' || b[i - 1] == b'$');
        if starts && prev_ok {
            let s = i;
            let mut real = false;
            i += 1;
            while i < b.len() {
                match b[i] {
                    b'0'..=b'9' => i += 1,
                    b'.' => { real = true; i += 1 }
                    b'e' | b'E' if b.get(i + 1).is_some_and(|d| d.is_ascii_digit() || *d == b'-' || *d == b'+') => { real = true; i += 2 }
                    _ => break,
                }
            }
            // a `#12` reference is skipped by prev_ok; a number glued to letters is a name
            if i >= b.len() || !(b[i].is_ascii_alphabetic() || b[i] == b'_') {
                out.push((s, i, real));
            }
        } else {
            i += 1;
        }
    }
    out
}

/// Run `read` on every mutation of `text`, at most `cap` positions (evenly strided) to bound the time.
fn fuzz(name: &str, text: &str, cap: usize, read: fn(&str) -> bool) {
    let nums = numbers(text);
    assert!(!nums.is_empty(), "{name}: no numbers found");
    let stride = nums.len().div_ceil(cap).max(1);
    let mut runs = 0usize;
    let mut bad: Vec<String> = Vec::new();
    for (k, &(s, e, real)) in nums.iter().enumerate().step_by(stride) {
        let pool: &[&str] = if real { REALS } else { INTS };
        for rep in pool.iter().chain(if real { INTS } else { REALS }.iter()) {
            let mut m = String::with_capacity(text.len() + 24);
            m.push_str(&text[..s]);
            m.push_str(rep);
            m.push_str(&text[e..]);
            let t0 = Instant::now();
            // a thread per run: a panic is caught by join, a hang shows as the time check
            let h = std::thread::Builder::new().stack_size(64 << 20).spawn(move || read(&m)).unwrap();
            let ok = h.join();
            let dt = t0.elapsed();
            runs += 1;
            if ok.is_err() {
                bad.push(format!("panic: number #{k} `{}` -> `{rep}`", &text[s..e]));
            } else if dt > Duration::from_secs(5) {
                bad.push(format!("slow {dt:?}: number #{k} `{}` -> `{rep}`", &text[s..e]));
            }
        }
    }
    eprintln!("{name}: {} numbers, {runs} mutated reads, {} bad", nums.len(), bad.len());
    assert!(bad.is_empty(), "{name}: {} bad mutations, first 12:\n{}", bad.len(), bad.iter().take(12).cloned().collect::<Vec<_>>().join("\n"));
}

fn step(t: &str) -> bool {
    ck_io::step::read_step(t).is_ok()
}

fn sat(t: &str) -> bool {
    ck_io::sat::import_sat(t).is_ok()
}

macro_rules! step_case {
    ($name:ident, $file:literal) => {
        #[test]
        fn $name() {
            fuzz($file, include_str!(concat!("fixtures/", $file)), 400, step);
        }
    };
}

step_case!(step_nist_ftc_11, "nist_ftc_11.step");
step_case!(step_nurbs_quarter_disc, "nurbs_quarter_disc.step");
step_case!(step_occ_cylinder_and_box, "occ_cylinder_and_box.step");
step_case!(step_inch_block_hole, "inch_block_hole.step");

macro_rules! sat_case {
    ($name:ident, $file:literal) => {
        #[test]
        fn $name() {
            fuzz($file, include_str!(concat!("fixtures/sat/", $file)), 400, sat);
        }
    };
}

sat_case!(sat_box, "box.sat");
sat_case!(sat_box_spline, "box_spline.sat");
sat_case!(sat_cylinder_hole, "cylinder_hole.sat");
sat_case!(sat_frustum, "frustum.sat");
sat_case!(sat_plate_fillet, "plate_fillet.sat");
sat_case!(sat_sphere, "sphere.sat");
sat_case!(sat_torus, "torus.sat");
