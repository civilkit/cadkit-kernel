//! Body names and the `newBody` operation (docs/adr/0004-multiple-bodies-in-one-part.md, step 1).
//!
//! * `op: "newBody"` (aliases `new`, `separate`; written as `newBody`) on `extrude`, `revolve`, `loft` and `sweep`:
//!   the result is added as its own lump and is not united with lumps it touches or crosses. Every other
//!   solid-producing feature refuses it with `invalidValue {detail, hint}`.
//! * `body: "<name>"` names the lump(s) a feature creates: a `newBody` feature, a `moveBody` with `copy:true`, and a
//!   `combine` (its result; default: the target's name stays). The name is stored on the lump (`Lump::name`) and
//!   travels with it: after every feature the names of the lumps present before are put back on the lumps that
//!   have the same origin.
//! * A lump's origin is the lowest-numbered feature that owns one of its faces; a face made inside an instance
//!   (`F2.C1@F9.1`) is owned by the feature that made the instance (`F9`). A cut, a fillet or a regen after an
//!   unrelated edit does not change it.
//! * Selector `{ "body": "<name>" }` next to `{face}`, `{feature}`, `{lump}`: every lump with that name.
//!
//! Errors: `bodyNameConflict {name, features}`, `missingReference {name, nearest}`, `ambiguousBody {name,
//! candidates}`, `invalidValue {detail}`.

use crate::regen::{codes, FeatureError};
use ck_json::Json;
use ck_topo::body::{Body, LumpId};

/// Wire values of `op` that mean "its own lump".
pub fn is_new_body_op(s: &str) -> bool {
    matches!(s, "newBody" | "new" | "separate")
}

pub const BODY_NAME_CONFLICT: &str = "bodyNameConflict";
pub const AMBIGUOUS_BODY: &str = "ambiguousBody";

fn invalid(detail: &str) -> FeatureError {
    FeatureError::new(codes::INVALID_VALUE).with("detail", Json::Str(detail.into()))
}

/// The `body` name of a feature: `None` when absent, `invalidValue` when it is not a non-empty string.
pub fn body_name(raw: &Json) -> Result<Option<String>, FeatureError> {
    match raw.get("body") {
        None | Some(Json::Null) => Ok(None),
        Some(Json::Str(s)) if !s.trim().is_empty() => Ok(Some(s.clone())),
        Some(_) => Err(invalid("body is a non-empty name (string)")),
    }
}

/// Feature types that take `op: "newBody"`.
const NEW_BODY_TYPES: &[&str] = &["extrude", "revolve", "loft", "sweep", "sphere"];

/// Up-front checks of a feature's operation and `body` name, before it runs: `newBody` on a feature that does not
/// support it, and a `body` name on an extrude / revolve / loft / sweep that is not `newBody`, are `invalidValue`.
/// Features without `op: newBody` and without `body` pass untouched.
pub fn check_feature(f: &crate::Feature) -> Result<(), FeatureError> {
    let t = f.type_name.as_str();
    if t == "combine" || t == "bodyBoolean" || t == "moveBody" {
        return Ok(());
    }
    let new_body = f.raw.get("op").and_then(|v| v.as_str()).is_some_and(is_new_body_op);
    if new_body && !NEW_BODY_TYPES.contains(&t) {
        return Err(invalid(&format!("operation newBody is not supported by {t}"))
            .with("hint", Json::Str("newBody is accepted by extrude, revolve, loft and sweep; build the solid with one of them (op newBody), then use combine, moveBody or a pattern".into())));
    }
    if NEW_BODY_TYPES.contains(&t) && f.raw.get("body").is_some_and(|b| !matches!(b, Json::Null)) {
        body_name(&f.raw)?;
        if !new_body {
            return Err(invalid("body names the lump a feature creates and needs op newBody")
                .with("hint", Json::Str("set op to newBody, or remove body".into())));
        }
    }
    Ok(())
}

fn rank(id: &str) -> (u64, &str) {
    // the first digit run, parsed out of the slice: `origin` asks for this once per face of a body
    // on every feature (`named` + `restore`), so the run must not be collected into a String first
    let b = id.as_bytes();
    let Some(s) = b.iter().position(u8::is_ascii_digit) else { return (u64::MAX, id) };
    let e = b[s..].iter().position(|c| !c.is_ascii_digit()).map_or(b.len(), |i| s + i);
    (std::str::from_utf8(&b[s..e]).ok().and_then(|d| d.parse().ok()).unwrap_or(u64::MAX), id)
}

/// Feature that owns a face for the origin rule: the feature that made the outermost instance, else the face's own.
fn face_owner(name: &ck_topo::body::FaceName) -> &str {
    match name.suffix.rsplit('@').next().filter(|s| !s.is_empty()) {
        Some(last) => last.split('.').next().unwrap_or(last),
        None => name.feature.as_str(),
    }
}

/// Origin feature of a lump (see the module doc).
pub fn origin(b: &Body, l: LumpId) -> Option<&str> {
    ck_topo::mass::lump_faces(b, l).into_iter().map(|f| face_owner(&b.face(f).name)).min_by(|x, y| rank(x).cmp(&rank(y)))
}

/// Does the body carry any lump name?
pub fn any_named(b: &Body) -> bool {
    b.lump_ids().any(|l| b.lump(l).name.is_some())
}

/// `(origin, name)` of every named lump.
pub fn named(b: &Body) -> Vec<(String, String)> {
    // the name is asked for first: `origin` walks the lump's faces, and a body is mostly lumps
    // nobody named
    if !any_named(b) {
        return vec![];
    }
    b.lump_ids().filter_map(|l| {
        let name = b.lump(l).name.clone()?;
        Some((origin(b, l)?.to_string(), name))
    }).collect()
}

/// Name every live lump of `b` whose origin is `feature` and that has no name yet.
pub fn stamp_origin(b: &mut Body, feature: &str, name: &str) {
    let ls: Vec<LumpId> = b.lump_ids().collect();
    for l in ls {
        if b.lump(l).name.is_none() && origin(b, l) == Some(feature) {
            b.lumps[l.idx()].name = Some(name.to_string());
        }
    }
}

/// Put the names of `prior` back on the lumps of `b` that have the same origin and no name; a name is given
/// to one lump only.
pub fn restore(b: &mut Body, prior: &[(String, String)]) {
    if prior.is_empty() {
        return;
    }
    // origin -> name, and the names a lump already carries: both maps are rebuilt per feature, so a
    // lump scan and a linear walk of `prior` per unnamed lump are what they replace
    let mut want: std::collections::HashMap<&str, &str> = std::collections::HashMap::with_capacity(prior.len());
    for (o, n) in prior {
        want.entry(o.as_str()).or_insert(n.as_str());
    }
    // owned: the names a lump already carries are borrowed from `b`, which the loop then writes to
    let mut taken: std::collections::HashSet<String> = b.lump_ids().filter_map(|l| b.lump(l).name.clone()).collect();
    let ls: Vec<LumpId> = b.lump_ids().collect();
    for l in ls {
        if b.lump(l).name.is_some() {
            continue;
        }
        let Some(o) = origin(b, l) else { continue };
        let Some(&n) = want.get(o) else { continue };
        if !taken.contains(n) {
            taken.insert(n.to_string());
            b.lumps[l.idx()].name = Some(n.to_string());
        }
    }
}

/// `bodyNameConflict` when a live lump (not in `except`) already carries `name`. `feature` is the feature that
/// wants the name; `features` in the error lists the feature that made the existing lump, then this one.
pub fn check_free(b: Option<&Body>, name: &str, feature: &str, except: &[LumpId]) -> Result<(), FeatureError> {
    let Some(b) = b else { return Ok(()) };
    for l in b.lump_ids() {
        if except.contains(&l) || b.lump(l).name.as_deref() != Some(name) {
            continue;
        }
        let mut fs = vec![];
        if let Some(o) = origin(b, l) {
            fs.push(Json::Str(o.to_string()));
        }
        fs.push(Json::Str(feature.to_string()));
        return Err(FeatureError::new(BODY_NAME_CONFLICT).with("name", Json::Str(name.into())).with("features", Json::Arr(fs)));
    }
    Ok(())
}

fn edit_distance(a: &str, b: &str) -> usize {
    let (a, b): (Vec<char>, Vec<char>) = (a.to_lowercase().chars().collect(), b.to_lowercase().chars().collect());
    let mut prev: Vec<usize> = (0..=b.len()).collect();
    for i in 1..=a.len() {
        let mut cur = vec![i];
        for j in 1..=b.len() {
            let sub = prev[j - 1] + usize::from(a[i - 1] != b[j - 1]);
            cur.push(sub.min(prev[j] + 1).min(cur[j - 1] + 1));
        }
        prev = cur;
    }
    prev[b.len()]
}

/// Names of the live lumps, nearest to `name` first (at most three), distinct.
pub fn nearest(b: Option<&Body>, name: &str) -> Vec<String> {
    let mut v: Vec<String> = vec![];
    if let Some(b) = b {
        for l in b.lump_ids() {
            if let Some(n) = &b.lump(l).name {
                if !v.contains(n) {
                    v.push(n.clone());
                }
            }
        }
    }
    v.sort_by_key(|n| (edit_distance(n, name), n.clone()));
    v.truncate(3);
    v
}

/// Every live lump named `name`; none: `missingReference {name, nearest}`.
pub fn lumps_named(b: Option<&Body>, name: &str) -> Result<Vec<LumpId>, FeatureError> {
    let hit: Vec<LumpId> = b.map(|b| b.lump_ids().filter(|&l| b.lump(l).name.as_deref() == Some(name)).collect()).unwrap_or_default();
    if hit.is_empty() {
        return Err(FeatureError::new(codes::MISSING_REFERENCE)
            .with("name", Json::Str(name.into()))
            .with("nearest", Json::Arr(nearest(b, name).into_iter().map(Json::Str).collect())));
    }
    Ok(hit)
}

/// `ambiguousBody {name, candidates}` for a name that must select one lump but selects several. Each candidate is
/// `{lump, face}` (wire lump index, one face id of it).
pub fn ambiguous(b: &Body, name: &str, lumps: &[LumpId]) -> FeatureError {
    let c = lumps
        .iter()
        .map(|&l| {
            let face = ck_topo::mass::lump_faces(b, l).first().map(|&f| b.face_id_str(f)).unwrap_or_default();
            Json::Obj(vec![("lump".into(), Json::Num(b.lump_index(l) as f64)), ("face".into(), Json::Str(face))])
        })
        .collect();
    FeatureError::new(AMBIGUOUS_BODY).with("name", Json::Str(name.into())).with("candidates", Json::Arr(c))
}
