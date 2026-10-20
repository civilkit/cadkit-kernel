//! `checkProfile` / `profiles` (rule engine, ck-ops `profile`). With `tag`
//! the profile runs on that tag's regenerated body; with `part` the part is
//! regenerated privately through the same incremental regen cache as
//! `regen`, so re-checking an edited document only replays the changed
//! tail of the feature program.

use crate::engine::{Exec, TraceSink};
use crate::regen_cache;
use ck_json::{Json, Obj};
use ck_model::regen::{self, RegenOpts, SketchResult};
use ck_model::Part;
use ck_ops::profile;
use ck_server::session::Cx;
use ck_server::trace::Trace;
use ck_topo::body::Body;

/// Regenerate `part_json` without tessellating or touching any tag.
/// Returns the body, the `(feature, code)` errors and the sketches.
pub(crate) fn private_regen(part_json: &Json) -> Result<(Option<Body>, Vec<(String, String)>, Vec<SketchResult>), String> {
    let part = Part::from_json(part_json)?;
    let opts = RegenOpts::default();
    let run_n = regen::run_len(&part, &opts);
    let mut trace = Trace::off();
    let mut ex = Exec { brep: ck_model::features::Brep::new(), sink: TraceSink::new(&mut trace), auto_heal: None };
    let mut resume = None;
    let mut rec = None;
    if run_n > 0 {
        let ko = regen_cache::KeyOpts { sketch_status: opts.sketch_status, solve_sketches: opts.solve_sketches, auto_heal: None };
        let keys = regen_cache::chain_keys(part_json, &ko, run_n);
        let mut chained = None;
        if let Some(h) = regen_cache::with(|c| c.lookup(&keys)) {
            ex.brep.load(&h.snap);
            chained = Some((h.result.clone(), h.snap));
            resume = Some(regen::Resume { next: h.next, result: h.result, axes: h.axes });
        }
        rec = Some(regen_cache::Recorder::new(keys, chained));
    }
    let ck = rec.as_mut().map(|r| r as &mut dyn regen::Checkpoint);
    let res = regen::regen_from(&part, &opts, &mut ex, resume, ck);
    let errors = res.errors.iter().map(|e| (e.feature.clone(), e.error.code.clone())).collect();
    Ok((ex.brep.body, errors, res.sketches))
}

/// `{type:"checkProfile", tag? | part?, profile, opts?}`
pub fn check_profile(cx: &Cx, snap_body: Option<&Body>) -> Result<Json, String> {
    let prof = profile::select_profile(cx.msg.get("profile"))?;
    let opts = profile::RunOpts::from_json(cx.msg.get("opts"));
    let owned;
    let body: &Body = match (cx.msg.get("part"), snap_body) {
        (Some(p), _) => {
            let (b, errors, _) = private_regen(p)?;
            match b {
                Some(b) => {
                    owned = b;
                    &owned
                }
                None => {
                    let errs = errors.iter().map(|(f, c)| Json::Arr(vec![Json::from(f.as_str()), Json::from(c.as_str())])).collect();
                    return Ok(Obj::new().put("profile", Obj::new().put("id", prof.id.as_str()).put("name", prof.name.as_str()).build()).put("noBody", true).put("errors", Json::Arr(errs)).build());
                }
            }
        }
        (None, Some(b)) => b,
        (None, None) => return Err("checkProfile needs a regenerated tag or a \"part\"".into()),
    };
    profile::run(body, &prof, &opts)
}

pub fn profiles() -> Json {
    profile::profiles_reply()
}
