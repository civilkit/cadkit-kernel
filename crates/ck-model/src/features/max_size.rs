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
    while n < MAX_BUILDS {
        if lo > 0.0 && hi - lo <= (0.03 * lo).max(0.02) {
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
    (lo > 0.0).then_some(lo)
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
    fn the_hint_names_the_size() {
        let fe = with_max_size(FeatureError::new("x").with("hint", Json::Str("reduce it".into())), "thickness", 2.5, "mm");
        let h = fe.params.iter().find(|(k, _)| k == "hint").unwrap().1.as_str().unwrap().to_string();
        assert_eq!(h, "reduce it; the largest thickness that builds is 2.5 mm");
        assert_eq!(fe.params.iter().filter(|(k, _)| k == "hint").count(), 1);
        let m = &fe.params.iter().find(|(k, _)| k == "maxSize").unwrap().1;
        assert_eq!(m.get("value").and_then(|v| v.as_f64()), Some(2.5));
    }
}
