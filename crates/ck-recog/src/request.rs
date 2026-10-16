//! The `recognize` request body: recognise, rebuild, verify.

use ck_json::{Json, Obj};
use ck_topo::body::Body;

/// `msg`: `{tag, opts?: {apply?}, apply?}`. Returns the reply `result` and, for `apply`, the rebuilt body.
pub fn run(body: &Body, msg: &Json) -> (Json, Option<Body>) {
    let apply = msg.get_bool("apply") == Some(true) || msg.get("opts").map_or(false, |o| o.get_bool("apply") == Some(true));
    let rec = crate::recognize(body);
    let report = crate::report(body, &rec);
    let mut errors: Vec<Json> = vec![];
    let mut verify = Json::Null;
    let mut rebuilt = None;
    let mut unexplained_area = 0.0;
    if let Json::Obj(_) = &report {
        if let Some(arr) = report.get_arr("unexplained") {
            unexplained_area = arr.iter().filter_map(|u| u.get_f64("area")).sum();
        }
    }
    if !rec.features.is_empty() {
        match crate::rebuild::rebuild(&rec.features) {
            Ok((b, errs)) => {
                errors = errs.into_iter().map(Json::Str).collect();
                if let Some(nb) = b {
                    verify = crate::rebuild::verify(body, &nb).json();
                    rebuilt = Some(nb);
                }
            }
            Err(e) => errors.push(Json::Str(e)),
        }
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
