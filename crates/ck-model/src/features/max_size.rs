//! `maxSize` on a refusal whose size is too large: the largest value of that field that builds, so a client
//! can offer it without reading the hint.
//!
//! Error params: `maxSize: {param, value, unit}`. `param` is the feature field to set (a dotted path for a
//! nested one: `thin.thickness`), `value` the number to put there (the sign of the request is kept: a face
//! pushed in by -12 gets -9.96), `unit` `mm` or `deg`. The `hint` says the same in words. It is given only for
//! a value that was built with the operation's own checks and a result no less valid than its input
//! (`ck_check::validate`), or for the operation's exact bound (the closed-form limits of hole and knurl); when no
//! size is found the refusal has no `maxSize`.

use crate::regen::FeatureError;
use ck_json::{Json, Obj};

/// Most builds one search may spend (only on the refusal path).
pub const MAX_BUILDS: usize = 8;

/// Most builds the search for a size ABOVE the asked one may spend (see [`smallest_that_builds`]).
pub const MAX_UP_BUILDS: usize = 16;

/// Longest a search may keep starting builds, in ms of the host clock.
pub const SEARCH_MS: f64 = 3000.0;

static BLEND_SEARCH: std::sync::atomic::AtomicBool = std::sync::atomic::AtomicBool::new(true);

/// Most faces a body may have for a refused fillet to search for its largest size: each search rebuilds the blend up
/// to [`MAX_BUILDS`] more times, which is quick on a part of this size and slow on a big assembly.
pub const BLEND_SEARCH_MAX_FACES: usize = 600;

/// Whether a refused fillet searches for its largest size. On by default (bodies up to [`BLEND_SEARCH_MAX_FACES`]
/// faces); a client that finds the extra builds too slow turns it off.
pub fn blend_search_enabled() -> bool {
    BLEND_SEARCH.load(std::sync::atomic::Ordering::Relaxed)
}

pub fn set_blend_search(on: bool) {
    BLEND_SEARCH.store(on, std::sync::atomic::Ordering::Relaxed);
}

/// Round down to 0.01 (to 0.001 under 0.1).
pub fn floor_nice(x: f64) -> f64 {
    let s = if x < 0.1 { 1000.0 } else { 100.0 };
    (x * s).floor() / s
}

/// The largest size in (0, `asked`) for which `builds` succeeds. `known` is a size the operation itself named
/// (tried first). Bisection from there stops within 3% (at least 0.02) of a size that failed, after at most
/// [`MAX_BUILDS`] builds. Every value tried is already rounded with [`floor_nice`], so the answer is a value
/// that was built. `None` when nothing tried builds.
pub fn largest_that_builds(asked: f64, known: Option<f64>, mut builds: impl FnMut(f64) -> bool) -> Option<f64> {
    if !(asked > 0.0) || !asked.is_finite() {
        return None;
    }
    let (mut lo, mut hi) = (0.0f64, asked);
    let mut n = 0;
    if let Some(k) = known.map(floor_nice).filter(|k| *k > 0.0 && *k < asked) {
        n += 1;
        if builds(k) {
            lo = k;
        } else {
            hi = k;
        }
    }
    // (a build that cannot succeed can itself take seconds: a refusal must not wait for the whole search then;
    // without a host clock the build count is the only bound)
    let started = ck_core::budget::mark();
    while n < MAX_BUILDS {
        if lo > 0.0 && hi - lo <= (0.03 * lo).max(0.02) {
            break;
        }
        if ck_core::budget::since(started) > SEARCH_MS {
            break;
        }
        let mid = floor_nice(0.5 * (lo + hi));
        if !(mid > lo && mid < hi) {
            break;
        }
        n += 1;
        if builds(mid) {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    // (a build can succeed at a larger size than one that failed: the corner of a rim and a wall edge builds at 0.5 and 2 mm and
    // not at 1 mm. Bisection alone would then name a size that is not the largest; up to three probes above it (within the build
    // budget) keep the claim honest for such non-monotonic limits)
    if lo > 0.0 && ck_core::budget::since(started) <= SEARCH_MS {
        let base = lo;
        for k in [0.25, 0.5, 0.75] {
            let probe = floor_nice(base + (asked - base) * k);
            if n >= MAX_BUILDS {
                break;
            }
            if probe > lo && probe < asked {
                n += 1;
                if builds(probe) {
                    lo = probe;
                }
            }
        }
    }
    (lo > 0.0).then_some(lo)
}

/// Most extra builds the verification sweep of a blend size hint may spend (only on the refusal path).
pub const SWEEP_BUILDS: usize = 6;

/// Longest the verification sweep may keep starting builds, in ms of the host clock (the build count is the real bound).
pub const SWEEP_MS: f64 = 1500.0;

/// What a verification sweep found: sizes that built and sizes that were refused, each ascending.
#[derive(Debug, Default, Clone, PartialEq)]
pub struct Sweep {
    pub built: Vec<f64>,
    pub refused: Vec<f64>,
}

/// Check, with at most [`SWEEP_BUILDS`] builds, how building behaves around `found` (a size that built, below `asked`,
/// which was refused): up to six evenly spaced sizes from `found / 4` to `min(2 * found, asked)`, each rounded with
/// [`floor_nice`] and not within 1 % of `found`. Passing is not monotonic in a blend's radius (a tee of pipes builds at 0.2 to
/// 1.87, fails at 1.9 to 3.1 and builds again from 3.2), so the search's answer alone could mislead.
pub fn sweep_around(asked: f64, found: f64, mut builds: impl FnMut(f64) -> bool) -> Sweep {
    let mut out = Sweep::default();
    if !(found > 0.0) || !(asked > found) || !asked.is_finite() {
        return out;
    }
    let (lo, hi) = (0.25 * found, (2.0 * found).min(asked));
    let started = ck_core::budget::mark();
    let mut seen: Vec<f64> = vec![];
    for i in 0..SWEEP_BUILDS {
        if ck_core::budget::since(started) > SWEEP_MS {
            break;
        }
        let v = floor_nice(lo + (hi - lo) * i as f64 / (SWEEP_BUILDS - 1) as f64);
        if !(v > 0.0) || v >= asked || (v - found).abs() <= 0.01 * found || seen.contains(&v) {
            continue;
        }
        seen.push(v);
        if builds(v) {
            out.built.push(v);
        } else {
            out.refused.push(v);
        }
    }
    out
}

/// `fe` with the radius hint of a blend whose search found `found` (a size that built), told by what `sweep` verified. When
/// no size checked is refused below one that builds, this is [`with_max_size`] (building is monotonic as far as checked: the
/// old wording and `maxSize` = the largest size that built). Otherwise `maxSize` is still the largest size that was built, and
/// the hint lists the sizes that build and the ones refused and says smaller sizes are not guaranteed to build; `sizeChecks
/// {builds, refused}` carries the same lists.
pub fn with_blend_sizes(fe: FeatureError, found: f64, sweep: &Sweep) -> FeatureError {
    let mut built = sweep.built.clone();
    built.push(found);
    built.sort_by(|a, b| a.total_cmp(b));
    let top = *built.last().unwrap();
    if !sweep.refused.iter().any(|r| *r < top) {
        return with_max_size(fe, "radius", top, "mm");
    }
    let list = |v: &[f64]| v.iter().map(|x| x.to_string()).collect::<Vec<_>>().join(", ");
    let say = format!(
        "a radius of {top} mm builds (checked: builds at {} mm; refused at {} mm); larger radii may fail and smaller ones are not guaranteed to build",
        list(&built),
        list(&sweep.refused)
    );
    let nums = |v: &[f64]| Json::Arr(v.iter().map(|x| Json::from(*x)).collect());
    let mut fe = with_max_size(fe, "radius", top, "mm");
    // (replace the monotonic wording with the checked one)
    if let Some((_, Json::Str(h))) = fe.params.iter_mut().find(|(k, _)| k == "hint") {
        let old = format!("the largest radius that builds is {top} mm");
        *h = match h.rfind(&old) {
            Some(i) => format!("{}{}{}", &h[..i], say, &h[i + old.len()..]),
            None => format!("{h}; {say}"),
        };
    }
    fe.with("sizeChecks", Obj::new().put("builds", nums(&built)).put("refused", nums(&sweep.refused)).build())
}

/// Round up to two significant digits.
pub fn ceil_nice(x: f64) -> f64 {
    if !(x > 0.0) || !x.is_finite() {
        return x;
    }
    let e = x.log10().floor();
    let s = 10f64.powf(1.0 - e);
    (x * s).ceil() / s
}

/// A size above `asked` for which `builds` succeeds: for a refusal whose size is too SMALL (a round the kernel's
/// tolerance cannot tell from the sharp edge). Doubles from `asked` until a build succeeds (at most [`MAX_UP_BUILDS`]
/// builds; a build of a size too small to change the part is quick), then bisects back towards the last size that
/// failed in at most three more, within [`SEARCH_MS`] in all. Every value tried is
/// rounded up with [`ceil_nice`], so the answer is a value that was built. `None` when nothing tried builds.
pub fn smallest_that_builds(asked: f64, mut builds: impl FnMut(f64) -> bool) -> Option<f64> {
    if !(asked > 0.0) || !asked.is_finite() {
        return None;
    }
    let started = ck_core::budget::mark();
    let (mut lo, mut hi) = (asked, 0.0f64);
    let mut n = 0;
    let mut r = asked;
    while n < MAX_UP_BUILDS && ck_core::budget::since(started) <= SEARCH_MS {
        let next = ceil_nice(r * 2.0);
        if !(next > r) || !next.is_finite() {
            break;
        }
        r = next;
        n += 1;
        if builds(r) {
            hi = r;
            break;
        }
        lo = r;
    }
    if hi == 0.0 {
        return None;
    }
    // (a size that builds can lie above one that fails and below another that fails: take the smaller of what was built)
    for _ in 0..3 {
        if ck_core::budget::since(started) > SEARCH_MS {
            break;
        }
        let mid = ceil_nice(0.5 * (lo + hi));
        if !(mid > lo && mid < hi) {
            break;
        }
        if builds(mid) {
            hi = mid;
        } else {
            lo = mid;
        }
    }
    Some(hi)
}

/// `fe` with `minSize {param, value, unit}` and the hint naming it: the size asked is too small to change the part.
pub fn with_min_size(mut fe: FeatureError, param: &str, value: f64, unit: &str) -> FeatureError {
    let say = format!("the {} is too small to change the part beyond the kernel's tolerance; {value} {unit} builds", words(param));
    fe.params.retain(|(k, _)| k != "minSize" && k != "hint");
    fe = fe.with("hint", Json::Str(say));
    fe.with("minSize", Obj::new().put("param", param).put("value", value).put("unit", unit).build())
}

/// The field in words, for the hint.
fn words(param: &str) -> &str {
    match param {
        "cboreDepth" => "counterbore depth",
        "cskDiameter" => "countersink diameter",
        "draft" | "angle" => "draft angle",
        "thin.thickness" => "wall thickness",
        p => p,
    }
}

/// `fe` with `maxSize {param, value, unit}` and the hint naming it (appended to a hint already there).
pub fn with_max_size(mut fe: FeatureError, param: &str, value: f64, unit: &str) -> FeatureError {
    let say = format!("the largest {} that builds is {value} {unit}", words(param));
    fe.params.retain(|(k, _)| k != "maxSize");
    match fe.params.iter_mut().find(|(k, _)| k == "hint") {
        Some((_, Json::Str(h))) => {
            h.push_str("; ");
            h.push_str(&say);
        }
        _ => fe = fe.with("hint", Json::Str(say)),
    }
    fe.with("maxSize", Obj::new().put("param", param).put("value", value).put("unit", unit).build())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_smallest_size_search_grows_from_the_asked_size_and_names_a_built_value() {
        // builds from 0.08 up, except the band 0.3..0.4 (not monotone)
        let mut tried = vec![];
        let v = smallest_that_builds(0.001, |x| {
            tried.push(x);
            x >= 0.08
        })
        .unwrap();
        assert!(v >= 0.08 && v < 0.2, "{v}");
        assert!(tried.contains(&v), "the answer is a value that was built: {tried:?}");
        assert_eq!(smallest_that_builds(0.001, |_| false), None);
        assert_eq!(smallest_that_builds(f64::NAN, |_| true), None);
    }

    #[test]
    fn bisection_finds_a_built_value_near_the_limit() {
        let mut calls = 0;
        let v = largest_that_builds(12.0, None, |x| {
            calls += 1;
            x <= 10.0
        })
        .unwrap();
        assert!(v <= 10.0 && v > 10.0 / 1.03 - 0.02, "{v}");
        assert!(calls <= MAX_BUILDS, "{calls}");
        // a named size that works is the start; one that does not is an upper bound
        let v = largest_that_builds(12.0, Some(9.5), |x| x <= 10.0).unwrap();
        assert!(v >= 9.5 && v <= 10.0, "{v}");
        let v = largest_that_builds(12.0, Some(11.0), |x| x <= 4.0).unwrap();
        assert!(v <= 4.0 && v > 3.8, "{v}");
        assert_eq!(largest_that_builds(12.0, None, |_| false), None);
        assert_eq!(largest_that_builds(0.0, None, |_| true), None);
    }

    #[test]
    fn the_blend_sweep_reports_a_window_that_fails_between_sizes_that_build() {
        // builds below 1.87 and from 3.2 up (the tee case), asked 5
        let ok = |x: f64| x <= 1.87 || x >= 3.2;
        let mut calls = 0;
        let found = largest_that_builds(5.0, None, |x| ok(x)).unwrap();
        let sw = sweep_around(5.0, found, |x| {
            calls += 1;
            ok(x)
        });
        assert!(calls <= SWEEP_BUILDS, "{calls}");
        assert!(sw.built.iter().all(|x| ok(*x)) && sw.refused.iter().all(|x| !ok(*x)), "{sw:?}");
        let fe = with_blend_sizes(FeatureError::new("blendFailed"), found, &Sweep { built: vec![0.5, 3.4], refused: vec![2.0, 2.8] });
        let h = fe.params.iter().find(|(k, _)| k == "hint").unwrap().1.as_str().unwrap().to_string();
        assert!(h.contains("a radius of 3.4 mm builds") && h.contains("refused at 2, 2.8") && h.contains("not guaranteed"), "{h}");
        assert!(!h.contains("largest radius"), "{h}");
        let m = &fe.params.iter().find(|(k, _)| k == "maxSize").unwrap().1;
        assert_eq!(m.get("value").and_then(|v| v.as_f64()), Some(3.4));
        assert!(fe.params.iter().any(|(k, _)| k == "sizeChecks"));
    }

    #[test]
    fn the_blend_sweep_keeps_the_old_wording_when_building_is_monotonic_as_far_as_checked() {
        let mut calls = 0;
        let found = largest_that_builds(12.0, None, |x| x <= 10.0).unwrap();
        let sw = sweep_around(12.0, found, |x| {
            calls += 1;
            x <= 10.0
        });
        assert!(calls <= SWEEP_BUILDS && !sw.built.is_empty(), "{calls} {sw:?}");
        let fe = with_blend_sizes(FeatureError::new("blendFailed"), found, &sw);
        let h = fe.params.iter().find(|(k, _)| k == "hint").unwrap().1.as_str().unwrap().to_string();
        assert!(h.contains("the largest radius that builds is"), "{h}");
        assert!(!fe.params.iter().any(|(k, _)| k == "sizeChecks"));
        // the sweep never builds beyond its count, nor at or above the asked size, nor with nothing found
        assert_eq!(sweep_around(12.0, 0.0, |_| panic!()), Sweep::default());
        let mut n = 0;
        sweep_around(1.0, 0.9, |x| {
            n += 1;
            assert!(x < 1.0);
            true
        });
        assert!(n <= SWEEP_BUILDS);
    }

    #[test]
    fn the_hint_names_the_size() {
        let fe = with_max_size(FeatureError::new("x").with("hint", Json::Str("reduce it".into())), "thickness", 2.5, "mm");
        let h = fe.params.iter().find(|(k, _)| k == "hint").unwrap().1.as_str().unwrap().to_string();
        assert_eq!(h, "reduce it; the largest thickness that builds is 2.5 mm");
        assert_eq!(fe.params.iter().filter(|(k, _)| k == "hint").count(), 1);
        let m = &fe.params.iter().find(|(k, _)| k == "maxSize").unwrap().1;
        assert_eq!(m.get("value").and_then(|v| v.as_f64()), Some(2.5));
    }
}
