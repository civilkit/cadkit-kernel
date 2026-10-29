//! Field names each feature type reads, for the `unknownField` warning: a misspelled field
//! (`depht` for `depth`) is otherwise ignored and the feature builds with the default.
//!
//! One table, [`TABLE`]; a type that is not in it is not checked. Keep it in step with what the
//! executors read (the field tables in the part-format book). The check only warns: older files
//! may carry fields this table does not know yet, and the result must not change.

use crate::Feature;
use ck_json::Json;

/// Fields that every feature may carry: identity, display and editor bookkeeping (cosmetic keys
/// of the regen cache), suppression, and the boolean tolerance read by the features that combine
/// a tool with the body.
/// Every field the regeneration cache ignores (`COSMETIC_KEYS` in ck-wasm) must be listed here: a cached reply replays the
/// warnings of the state it was made from, so a field that is ignored by the cache key but unknown here would warn on a cold
/// run and not on a hit.
const COMMON: &[&str] = &["id", "type", "num", "name", "suppressed", "group", "autoName", "visible", "userHidden", "color", "after", "tolerance", "autoFuzzy"];

/// Known fields per feature type, beyond [`COMMON`]. Nested keys (inside `thin`, `endRef`, the
/// edge picks, ...) are not checked.
pub const TABLE: &[(&str, &[&str])] = &[
    // `seq`, `prop` and `gear` are carried by the sketch editor and echoed back
    ("sketch", &["plane", "points", "entities", "constraints", "seq", "prop", "gear"]),
    (
        "extrude",
        &["sketch", "op", "end", "depth", "flip", "draft", "draftOut", "twist", "twistCentre", "regions", "thin", "endRef", "offset", "strictEnd", "body"],
    ),
    ("revolve", &["sketch", "op", "axis", "angle", "end", "flip", "regions", "thin", "body"]),
    (
        "fillet",
        &["radius", "edges", "profile", "propagate", "filletType", "radii", "rho", "rhos", "shape", "side1", "center", "side2"],
    ),
    ("importStep", &["blob", "step", "body", "heal"]),
    ("chamfer", &["chamferType", "distance", "distance2", "distance3", "angle", "face", "flip", "propagate", "edges"]),
    (
        "hole",
        &["sketch", "standard", "size", "fit", "kind", "diameter", "end", "depth", "tip", "flip", "cboreDiameter", "cboreDepth", "cskDiameter", "cskAngle"],
    ),
];

/// The fields of `f` that its type does not know, in document order, each with the known field it
/// is probably a misspelling of (a case difference or an edit distance of at most 2).
/// Empty for a type that is not in [`TABLE`].
pub fn unknown_fields(f: &Feature) -> Vec<(String, Option<&'static str>)> {
    let (Some((_, known)), Json::Obj(members)) = (TABLE.iter().find(|(t, _)| *t == f.type_name), &f.raw) else { return vec![] };
    members
        .iter()
        .map(|(k, _)| k.as_str())
        .filter(|k| !COMMON.contains(k) && !known.contains(k))
        .map(|k| (k.to_string(), known.iter().copied().find(|c| c.eq_ignore_ascii_case(k)).or_else(|| nearest(k, known))))
        .collect()
}

/// The known field closest to `k` within an edit distance of 2 (and at most a third of its length,
/// so short names do not match everything); the first on a tie.
fn nearest(k: &str, known: &[&'static str]) -> Option<&'static str> {
    let k = k.to_ascii_lowercase();
    let mut best: Option<(usize, &'static str)> = None;
    for c in known {
        let d = distance(&k, &c.to_ascii_lowercase());
        if d <= 2 && d * 3 <= c.len() && best.map_or(true, |(b, _)| d < b) {
            best = Some((d, c));
        }
    }
    best.map(|(_, c)| c)
}

/// Edit distance counting a swap of two neighbouring letters as one edit (optimal string alignment).
fn distance(a: &str, b: &str) -> usize {
    let (a, b): (Vec<char>, Vec<char>) = (a.chars().collect(), b.chars().collect());
    let mut d = vec![vec![0usize; b.len() + 1]; a.len() + 1];
    for (i, row) in d.iter_mut().enumerate() {
        row[0] = i;
    }
    for j in 0..=b.len() {
        d[0][j] = j;
    }
    for i in 1..=a.len() {
        for j in 1..=b.len() {
            let cost = usize::from(a[i - 1] != b[j - 1]);
            d[i][j] = (d[i - 1][j] + 1).min(d[i][j - 1] + 1).min(d[i - 1][j - 1] + cost);
            if i > 1 && j > 1 && a[i - 1] == b[j - 2] && a[i - 2] == b[j - 1] {
                d[i][j] = d[i][j].min(d[i - 2][j - 2] + 1);
            }
        }
    }
    d[a.len()][b.len()]
}
