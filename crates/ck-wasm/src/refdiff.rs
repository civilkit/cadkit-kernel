//! `refDiff {base, part}`: regenerate both parts privately and report
//! references (face ids, `{faces, at}` edge refs) that, at the feature that
//! uses them, now resolve to a geometrically different entity than before
//! (`referenceChanged`) or to nothing (`referenceLost`). This is the "a
//! delete / reorder / edit silently re-pointed a reference" warning of
//! ADR 0002. Comparison rules are those of tools/naming_stability.mjs
//! (kind, direction, neighbour kinds, size ratio, position), so ordinary
//! dimension edits that keep the topology do not warn.

use crate::engine::{Exec, TraceSink};
use ck_core::linalg::Vec3;
use ck_json::{Json, Obj};
use ck_model::regen::{self, FeatureError, FeatureExec, RegenOpts, RegenState};
use ck_model::{Feature, Part};
use ck_ops::agent::{self, Sig};
use ck_server::session::Cx;
use ck_server::trace::Trace;

/// A reference found in a feature's JSON.
#[derive(Clone, Debug, PartialEq)]
enum RefKey {
    Face(String),
    /// Sorted face pair + ordinal among the feature's refs to that pair.
    Edge(Vec<String>, usize),
}

impl RefKey {
    fn to_json(&self, at: Option<[f64; 3]>) -> Json {
        match self {
            RefKey::Face(id) => Json::from(id.as_str()),
            RefKey::Edge(fs, _) => {
                let mut o = Obj::new().put("faces", Json::Arr(fs.iter().map(|s| Json::from(s.as_str())).collect()));
                if let Some(a) = at {
                    o = o.put("at", Json::Arr(a.iter().map(|&x| Json::Num(x)).collect()));
                }
                o.build()
            }
        }
    }
}

fn is_face_name(s: &str) -> bool {
    let b = s.as_bytes();
    if b.first() != Some(&b'F') {
        return false;
    }
    let digits = b[1..].iter().take_while(|c| c.is_ascii_digit()).count();
    digits > 0 && b.get(1 + digits) == Some(&b'.') && b.get(2 + digits).is_some_and(|c| c.is_ascii_alphabetic())
}

/// Every face-name string and `{faces:[a,b], at}` edge ref inside `j`.
fn collect(j: &Json, faces: &mut Vec<String>, edges: &mut Vec<(Vec<String>, Option<[f64; 3]>)>) {
    match j {
        Json::Str(s) if is_face_name(s) => {
            if !faces.contains(s) {
                faces.push(s.clone())
            }
        }
        Json::Arr(a) => a.iter().for_each(|x| collect(x, faces, edges)),
        Json::Obj(m) => {
            if let (Some(fs), Some(at)) = (j.get_arr("faces"), j.get("at").and_then(Json::as_vec3)) {
                let names: Vec<String> = fs.iter().filter_map(|x| x.as_str().map(String::from)).collect();
                if names.len() == 2 {
                    edges.push((names, Some(at)));
                    return;
                }
            }
            m.iter().for_each(|(_, v)| collect(v, faces, edges));
        }
        _ => {}
    }
}

type Recorded = (RefKey, Option<[f64; 3]>, Option<Sig>);

struct Recorder<'a> {
    inner: Exec<'a>,
    /// feature id -> recorded refs
    refs: Vec<(String, Vec<Recorded>)>,
    /// signatures captured when a sketch/plane asked for a face frame
    frames: Vec<(String, Option<Sig>)>,
}

impl Recorder<'_> {
    fn snapshot(&self, f: &Feature) -> Vec<Recorded> {
        let (mut faces, mut edges) = (vec![], vec![]);
        collect(&f.raw, &mut faces, &mut edges);
        if faces.is_empty() && edges.is_empty() {
            return vec![];
        }
        let Some(body) = self.inner.brep.body.as_ref() else { return vec![] };
        let facts = agent::gather(body);
        let mut out: Vec<Recorded> = faces.into_iter().map(|id| {
            let sig = agent::face_sig(&facts, &id);
            (RefKey::Face(id), None, sig)
        }).collect();
        let mut seen: Vec<(Vec<String>, usize)> = vec![];
        for (names, at) in edges {
            let mut key = names.clone();
            key.sort();
            let ord = seen.iter().filter(|(k, _)| *k == key).count();
            seen.push((key.clone(), ord));
            let sig = agent::edge_sig(&facts, &key, at.map(Vec3::from_array));
            out.push((RefKey::Edge(key, ord), at, sig));
        }
        out
    }
}

impl FeatureExec for Recorder<'_> {
    fn tracer(&mut self) -> Option<&mut dyn regen::Tracer> {
        self.inner.tracer()
    }
    fn face_count(&self) -> Option<usize> {
        self.inner.face_count()
    }
    fn face_frame(&mut self, state: &RegenState, face: &str) -> Option<ck_core::linalg::Frame> {
        if !self.frames.iter().any(|(f, _)| f == face) {
            let sig = self.inner.brep.body.as_ref().and_then(|b| agent::face_sig(&agent::gather(b), face));
            self.frames.push((face.to_string(), sig));
        }
        self.inner.face_frame(state, face)
    }
    fn prepare(&mut self, state: &RegenState, features: &[&Feature]) {
        self.inner.prepare(state, features)
    }
    fn exec(&mut self, state: &mut RegenState, f: &Feature) -> Result<(), FeatureError> {
        let rec = self.snapshot(f);
        if !rec.is_empty() {
            self.refs.push((f.id.clone(), rec));
        }
        self.inner.exec(state, f)
    }
}

struct Run {
    refs: Vec<(String, Vec<Recorded>)>,
    errors: Vec<(String, String)>,
    ids: Vec<String>,
}

fn run(part: &Part) -> Run {
    let mut trace = Trace::off();
    let mut rec = Recorder {
        inner: Exec { brep: ck_model::features::Brep::new(), sink: TraceSink::new(&mut trace), auto_heal: None },
        refs: vec![],
        frames: vec![],
    };
    let res = regen::regen(part, &RegenOpts::default(), &mut rec);
    // sketch / plane features reach faces through face_frame, not exec
    for f in &part.features {
        if f.suppressed || rec.refs.iter().any(|(id, _)| *id == f.id) {
            continue;
        }
        let (mut faces, mut edges) = (vec![], vec![]);
        collect(&f.raw, &mut faces, &mut edges);
        let got: Vec<Recorded> = faces
            .into_iter()
            .filter_map(|id| rec.frames.iter().find(|(n, _)| *n == id).map(|(n, s)| (RefKey::Face(n.clone()), None, s.clone())))
            .collect();
        if !got.is_empty() {
            rec.refs.push((f.id.clone(), got));
        }
    }
    Run {
        refs: rec.refs,
        errors: res.errors.iter().map(|e| (e.feature.clone(), e.error.code.clone())).collect(),
        ids: part.features.iter().map(|f| f.id.clone()).collect(),
    }
}

pub fn reply(cx: &mut Cx) -> Result<Json, String> {
    let base = Part::from_json(cx.msg.get("base").unwrap_or(&Json::Null))?;
    let part = Part::from_json(cx.msg.get("part").unwrap_or(&Json::Null))?;
    let (a, b) = (run(&base), run(&part));
    let mut warnings = vec![];
    let mut checked = 0usize;
    let mut new_runs: Vec<&(String, Vec<Recorded>)> = b.refs.iter().collect();
    new_runs.sort_by_key(|(id, _)| b.ids.iter().position(|x| x == id));
    for (fid, new_refs) in new_runs {
        let Some((_, old_refs)) = a.refs.iter().find(|(id, _)| id == fid) else { continue };
        for (key, at, after) in new_refs {
            let Some((_, _, before)) = old_refs.iter().find(|(k, _, _)| k == key) else { continue };
            let Some(before) = before else { continue };
            checked += 1;
            let (code, reason, after_text) = match after {
                None => ("referenceLost", "the reference no longer resolves to anything".to_string(), Json::Null),
                Some(af) => match agent::sig_diff(before, af) {
                    Some(why) => ("referenceChanged", why, Json::from(af.text.as_str())),
                    None => continue,
                },
            };
            warnings.push(
                Obj::new()
                    .put("code", code)
                    .put("feature", fid.as_str())
                    .put("ref", key.to_json(*at))
                    .put("reason", reason)
                    .put("before", before.text.as_str())
                    .put("after", after_text)
                    .build(),
            );
        }
    }
    let removed: Vec<Json> = a.ids.iter().filter(|i| !b.ids.contains(i)).map(|s| Json::from(s.as_str())).collect();
    let added: Vec<Json> = b.ids.iter().filter(|i| !a.ids.contains(i)).map(|s| Json::from(s.as_str())).collect();
    let common_a: Vec<&String> = a.ids.iter().filter(|i| b.ids.contains(i)).collect();
    let common_b: Vec<&String> = b.ids.iter().filter(|i| a.ids.contains(i)).collect();
    let errs = b.errors.iter().map(|(f, c)| Json::Arr(vec![Json::from(f.as_str()), Json::from(c.as_str())])).collect();
    Ok(Obj::new()
        .put("ok", warnings.is_empty())
        .put("checked", checked as f64)
        .put("warnings", Json::Arr(warnings))
        .put("features", Obj::new().put("removed", Json::Arr(removed)).put("added", Json::Arr(added)).put("reordered", common_a != common_b))
        .put("errors", Json::Arr(errs))
        .build())
}
