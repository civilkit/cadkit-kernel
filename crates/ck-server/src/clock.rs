//! Host clock hook (`ck_now`, performance.now in ms) and the original's
//! elapsed-time arithmetic.
//!
//! Evidence: probing the oracle with a scripted `ck_now` (20 vectors,
//! tools/../scratch clock probes) matched exactly one formula —
//! `Duration::from_secs_f64((end - start).max(0.0) * 0.001).as_secs_f64() * 1000.0`:
//!
//! - diffs are rounded to whole nanoseconds (`[0, 1.0000005]` -> `1.000001`,
//!   `[3.3, 3.30000049]` -> `0`), then re-expanded through `as_secs_f64`
//!   (`[6e-7, 1]` -> `0.9999989999999999`, `[0, 5e-7]` ->
//!   `0.0000010000000000000002`);
//! - negative and NaN diffs saturate to `0` (`f64::max` drops NaN);
//! - an infinite diff panics in `Duration::from_secs_f64` (the oracle traps).

use std::cell::Cell;
use std::time::Duration;

thread_local! {
    static NOW: Cell<fn() -> f64> = const { Cell::new(zero) };
    static UNIX_MS: Cell<fn() -> f64> = const { Cell::new(zero) };
}

fn zero() -> f64 {
    0.0
}

/// Install the monotonic clock (ms). ck-wasm wires this to the `ck_now` import.
pub fn set_now(f: fn() -> f64) {
    NOW.with(|c| c.set(f));
}

/// Install the wall clock (ms since the epoch); `ck_unix_ms` in the wasm.
pub fn set_unix_ms(f: fn() -> f64) {
    UNIX_MS.with(|c| c.set(f));
}

/// Current monotonic time in ms (one host call).
pub fn now() -> f64 {
    NOW.with(|c| c.get())()
}

/// Current wall-clock time in ms since the Unix epoch.
pub fn unix_ms() -> f64 {
    UNIX_MS.with(|c| c.get())()
}

/// Elapsed ms between two `now()` samples, quantised exactly like the
/// original (see module docs).
pub fn elapsed_ms(start: f64, end: f64) -> f64 {
    Duration::from_secs_f64((end - start).max(0.0) * 0.001).as_secs_f64() * 1000.0
}

#[cfg(test)]
mod tests {
    use super::*;

    /// (start, end, oracle "ms" text) — every row captured from kernel.wasm.
    #[test]
    fn elapsed_matches_oracle() {
        let t: &[(f64, f64, &str)] = &[
            (0., 1.0000004, "1"),
            (0., 1.0000006, "1.000001"),
            (0., 1.0000005, "1.000001"),
            (0., 1.0000015, "1.000002"),
            (5., 4., "0"),
            (0., -1., "0"),
            (1e9, 1000000000.1234568, "0.123457"),
            (4e-7, 1.0000004, "1"),
            (6e-7, 1., "0.9999989999999999"),
            (f64::NAN, 1., "0"),
            (123.4567894, 200.1111116, "76.654322"),
            (0., 5e-7, "0.0000010000000000000002"),
            (0., 0.0000015, "0.0000010000000000000002"),
            (0., 0.0000025, "0.000003"),
            (0., 2.0000005, "2.000001"),
            (0., 1e-7, "0"),
            (1., 1.1, "0.1"),
            (1000., 1000.3, "0.3"),
            (0., 123456.789, "123456.789"),
            (0., 1000000000000.3, "1000000000000.3"),
            (3.3, 3.30000049, "0"),
            (1001.23456789, 1002.46913578, "1.2345679999999999"),
        ];
        for &(a, b, want) in t {
            assert_eq!(ck_json::fmt_f64(elapsed_ms(a, b)), want, "{a} -> {b}");
        }
    }
}
