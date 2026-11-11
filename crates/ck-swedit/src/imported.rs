//! A part whose geometry is one or more imported bodies (`Imported`, `Multi_CAD_Imported`: a foreign solid brought
//! into the part as its first feature) and nothing else: every face of the file's final body belongs to such a
//! feature. The file's solid IS the body of the history then, so the history is one `importBody` step holding it
//! (the same reader and sorting as the plain import) and the file reads as an exact, editable part with that start
//! body, instead of an anonymous plain solid.
//!
//! Where later features add or remove material (cuts and bosses on the imported body), the body at the imported
//! step is not in the file (only the final body is), so those files are left to the remainder logic.

use crate::{Outcome, Status, Step, Tagged};
use ck_json::{Json, Obj};
use ck_recog::rebuild::Verify;
use std::collections::HashSet;

/// The feature kinds that bring a foreign body in.
pub fn is_imported(kind: &str) -> bool {
    matches!(kind, "Imported" | "Multi_CAD_Imported")
}

/// The `importBody` feature that starts a history (`blob` left empty: [`Outcome::part_json`] fills it).
pub fn start_feature(id: &str, name: &str) -> Json {
    Obj::new().put("type", "importBody").put("id", id).put("name", name).put("format", "sldprt").put("blob", "").put("replace", true).build()
}

/// The history of a file whose whole body is the imported one: `None` when other features made faces, or the body
/// is no valid solid (the import then comes in as surface bodies and the plain import stays as it is).
pub fn rebuild(t: &Tagged) -> Option<Outcome> {
    let ids: HashSet<i64> = t.features.iter().filter(|f| is_imported(&f.kind)).map(|f| f.id).collect();
    if ids.is_empty() || t.body.num_faces() == 0 {
        return None;
    }
    // (a face may also carry the id of a feature the file's tree does not list, such as a copy of an imported body:
    // nothing reads it back, so it stays part of the imported body; at least one face must be an imported one's)
    let listed: HashSet<i64> = t.features.iter().map(|f| f.id).collect();
    let all = (0..t.body.faces.len()).all(|i| {
        let tg = &t.tags[i];
        !tg.is_empty() && tg.iter().all(|x| ids.contains(&x.0) || !listed.contains(&x.0))
    }) && t.tags.iter().flatten().any(|x| ids.contains(&x.0));
    if !all || !ck_check::validate(t.body, 1e-6).is_empty() {
        return None;
    }
    let first = t.features.iter().find(|f| ids.contains(&f.id))?;
    let mut steps = vec![];
    for f in t.features.iter().filter(|f| ids.contains(&f.id)) {
        let faces = t.tags.iter().filter(|v| v.iter().any(|x| x.0 == f.id)).count();
        let (status, note) = if f.id == first.id {
            (Status::Rebuilt, "the file's solid as the starting body (importBody)".to_string())
        } else {
            (Status::Skipped, "another imported body: already part of the file's solid".to_string())
        };
        steps.push(Step { sw_id: f.id, name: f.name.clone(), kind: f.kind.clone(), faces, status, note });
    }
    let v = ck_topo::mass::mass_props(t.body).volume;
    let verify = Verify { vol_orig: v, vol_new: v, sym_diff: Some(0.0), rel: 0.0, bbox_dev: 0.0, area_rel: 0.0, exact: true, close: true };
    let ranges = steps.iter().map(|s| if s.status == Status::Rebuilt { (0, 1) } else { (1, 1) }).collect();
    Some(Outcome {
        features: vec![start_feature("F1", &first.name)],
        steps,
        body: None,
        verify: Some(verify),
        errors: vec![],
        fallback: None,
        notes: vec!["the history is the imported body: the file's solid is the part's starting body".into()],
        prefix: 0,
        remainder: None,
        checks: vec![],
        ranges,
    })
}
