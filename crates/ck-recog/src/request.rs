//! The `recognize` request body: recognise, rebuild, verify.

use ck_json::{Json, Obj};
use ck_topo::body::Body;

/// `msg`: `{tag, opts?: {apply?}, apply?}`. Returns the reply `result` and, for `apply`, the rebuilt body.
pub fn run(body: &Body, msg: &Json) -> (Json, Option<Body>) {
    let apply = msg.get_bool("apply") == Some(true) || msg.get("opts").map_or(false, |o| o.get_bool("apply") == Some(true));
    // try the candidates in preference order, keep the first exact one, else the best
    let mut best: Option<(f64, crate::Recognition, Option<Body>, Option<crate::rebuild::Verify>, Vec<Json>)> = None;
    for (i, rec) in crate::candidates(body).into_iter().enumerate() {
        if i >= 4 {
            break;
        }
        let mut errors: Vec<Json> = vec![];
        let mut rebuilt = None;
        let mut verify = None;
        let mut score = f64::INFINITY;
        if !rec.features.is_empty() {
            match crate::rebuild::rebuild(&rec.features) {
                Ok((b, errs)) => {
                    errors = errs.into_iter().map(Json::Str).collect();
                    if let Some(nb) = b {
                        let v = crate::rebuild::verify(body, &nb);
                        score = v.rel + rec.unexplained.len() as f64 * 1e-9;
                        verify = Some(v);
                        rebuilt = Some(nb);
                    }
                }
                Err(e) => errors.push(Json::Str(e)),
            }
        }
        let done = verify.as_ref().map_or(false, |v| v.exact) && rec.unexplained.is_empty();
        if best.as_ref().map_or(true, |b| score < b.0) {
            best = Some((score, rec, rebuilt, verify, errors));
        }
        if done {
            break;
        }
    }
    let (_, rec, rebuilt, verify, errors) = best.unwrap();
    let report = crate::report(body, &rec);
    let verify = verify.map_or(Json::Null, |v| v.json());
    let mut unexplained_area = 0.0;
    if let Some(arr) = report.get_arr("unexplained") {
        unexplained_area = arr.iter().filter_map(|u| u.get_f64("area")).sum();
    }
    let total = ck_topo::mass::mass_props(body).area.max(1e-12);
    let out = Obj::new()
        .put("features", Json::Arr(rec.features.clone()))
        .put("report", report)
        .put("verify", verify)
        .put("errors", Json::Arr(errors))
        .put("unexplainedArea", unexplained_area)
        .put("unexplainedAreaFraction", unexplained_area / total)
        .put("applied", apply && rebuilt.is_some())
        .build();
    (out, if apply { rebuilt } else { None })
}
