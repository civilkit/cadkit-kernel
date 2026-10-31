//! `repairPart`: Repair Part on the surface bodies an `importStep` feature left (the parts of its file that did not make
//! valid solids, see `import_step.rs`). One undoable feature; the repair itself is `ck_heal::repair`:
//!
//! ```text
//! {"type":"repairPart","id":"F2","target":"F1"}
//! {"type":"repairPart","id":"F2","target":"F1","gap":0.5,"patchHoles":false}
//! ```
//!
//! * `target`: the `importStep` feature whose surface bodies (`<target>.P<k>`) are repaired.
//! * In order: faces the reader left out are rebuilt on their own surface, free edges within `gap` (mm, default 0.2,
//!   at most 1) are sewn, flat holes are patched (`patchHoles`, default true). A part that comes out a valid solid
//!   joins the solid body (and leaves the surface list); one that does not stays a surface body, repaired as far as
//!   it went.
//! * Report: the warning `repairReport {parts:[{surface, label, valid, closed, rebuilt, notRebuilt, sewn, holes, facesBefore,
//!   facesAfter, tolBefore, tolAfter, volume, remaining}]}`, one entry per surface body.
//! * Errors: `repairTarget {detail}` (no or wrong target), `repairNothing {detail}` (the import has no surface body
//!   left to repair), `repairGap {detail}` (gap out of range).

use super::{extrude::Combine, import_step, Brep};
use crate::regen::{FeatureError, RegenError, RegenState};
use crate::Feature;
use ck_heal::honest::Reason;
use ck_heal::repair::{self, RepairOpts};
use ck_json::Json;

pub const REPAIR_TARGET: &str = "repairTarget";
pub const REPAIR_NOTHING: &str = "repairNothing";
pub const REPAIR_GAP: &str = "repairGap";
pub const REPAIR_REPORT: &str = "repairReport";

fn err(code: &str, detail: impl Into<String>) -> FeatureError {
    FeatureError::new(code).with("detail", Json::Str(detail.into()))
}

fn reasons(rs: &[Reason]) -> Json {
    Json::Arr(
        rs.iter()
            .map(|r| {
                Json::Obj(vec![
                    ("code".into(), Json::Str(r.code.clone())),
                    ("faces".into(), Json::Arr(r.faces.iter().map(|f| Json::Str(f.clone())).collect())),
                    ("message".into(), Json::Str(r.message.clone())),
                ])
            })
            .collect(),
    )
}

pub fn exec_repair_part(br: &mut Brep, state: &mut RegenState, f: &Feature) -> Result<(), FeatureError> {
    let target = match f.raw.get("target") {
        Some(Json::Str(t)) => t.clone(),
        _ => return Err(err(REPAIR_TARGET, "`target` must name the importStep feature whose surface bodies are repaired")),
    };
    let gap = match f.raw.get("gap") {
        None => repair::DEFAULT_GAP,
        Some(v) => v.as_f64().ok_or_else(|| err(REPAIR_GAP, "`gap` must be a number of mm"))?,
    };
    if !(gap > 0.0 && gap <= repair::MAX_GAP) {
        return Err(err(REPAIR_GAP, format!("the gap must be more than 0 and at most {} mm (got {gap})", repair::MAX_GAP)));
    }
    let patch_holes = match f.raw.get("patchHoles") {
        None => true,
        Some(Json::Bool(b)) => *b,
        Some(_) => return Err(err(REPAIR_GAP, "`patchHoles` must be true or false")),
    };
    let tf = state
        .part
        .features
        .iter()
        .find(|x| x.id == target)
        .cloned()
        .ok_or_else(|| err(REPAIR_TARGET, format!("there is no feature `{target}`")))?;
    if tf.type_name != "importStep" {
        return Err(err(REPAIR_TARGET, format!("`{target}` is a {} feature, not an importStep", tf.type_name)));
    }
    let prefix = format!("{target}.P");
    let ids: Vec<String> = br.surfs.iter().map(|s| s.0.clone()).filter(|id| id.starts_with(&prefix)).collect();
    if ids.is_empty() {
        return Err(err(REPAIR_NOTHING, format!("`{target}` has no surface body to repair: every part of its file is a solid")));
    }
    // the faces the reader left out (with their surfaces) come from reading the target's file again
    let heal = matches!(tf.raw.get("heal"), Some(Json::Bool(true)));
    let read = import_step::read_parts(&tf.id, import_step::data(state, &tf)?, heal)?;
    let opts = RepairOpts { gap, patch_holes, ..Default::default() };
    let mut out = vec![];
    let mut first_err: Option<Reason> = None;
    for id in ids {
        let k: usize = id[prefix.len()..].parse::<usize>().unwrap_or(1).saturating_sub(1);
        let label = read.sorted.parts.get(k).map_or(id.clone(), |p| p.label.clone());
        let Some(i) = br.surf_index(&id) else { continue };
        let mut b = br.surfs[i].1.clone();
        let left = read.left_out.get(k).map_or(&[][..], |v| &v[..]);
        match repair::repair(&mut b, left, &opts) {
            Err(r) => {
                out.push(Json::Obj(vec![("surface".into(), Json::Str(id.clone())), ("label".into(), Json::Str(label)), ("refused".into(), reasons(std::slice::from_ref(&r)))]));
                first_err.get_or_insert(r);
            }
            Ok(rep) => {
                if rep.valid {
                    br.drop_surfaces(vec![i]);
                    br.combine(&b, Combine::Separate, false)?;
                } else {
                    br.set_surface(&id, b);
                }
                let holes = rep
                    .holes
                    .iter()
                    .map(|h| {
                        Json::Obj(vec![
                            ("edges".into(), Json::Num(h.edges as f64)),
                            ("size".into(), Json::Num(h.size)),
                            ("planar".into(), Json::Bool(h.planar)),
                            ("patched".into(), Json::Bool(h.patched)),
                            ("reason".into(), Json::Str(h.reason.clone())),
                        ])
                    })
                    .collect();
                out.push(Json::Obj(vec![
                    ("surface".into(), Json::Str(id.clone())),
                    ("label".into(), Json::Str(label)),
                    ("valid".into(), Json::Bool(rep.valid)),
                    ("closed".into(), Json::Bool(rep.closed)),
                    ("rebuilt".into(), Json::Arr(rep.rebuilt.iter().map(|s| Json::Str(s.clone())).collect())),
                    ("notRebuilt".into(), reasons(&rep.not_rebuilt)),
                    ("sewn".into(), Json::Num(rep.sewn as f64)),
                    ("fixed".into(), Json::Num(rep.fixed.len() as f64)),
                    ("holes".into(), Json::Arr(holes)),
                    ("facesBefore".into(), Json::Num(rep.faces_before as f64)),
                    ("facesAfter".into(), Json::Num(rep.faces_after as f64)),
                    ("tolBefore".into(), Json::Num(rep.tol_before)),
                    ("tolAfter".into(), Json::Num(rep.tol_after)),
                    ("volume".into(), rep.volume.map_or(Json::Null, Json::Num)),
                    ("remaining".into(), reasons(&rep.remaining)),
                ]));
            }
        }
    }
    // every surface body refused the repair (nothing to do, or a bad gap): the feature is refused with the reason
    if out.iter().all(|o| o.get("refused").is_some()) {
        let r = first_err.unwrap();
        return Err(err(&r.code, r.message));
    }
    state.result.warnings.push(RegenError { feature: f.id.clone(), error: FeatureError::new(REPAIR_REPORT).with("parts", Json::Arr(out)) });
    Ok(())
}
