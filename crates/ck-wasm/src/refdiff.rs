//! `refDiff {base, part}`: regenerate both parts privately and report
//! references (face ids, `{faces, at}` edge refs) that, at the feature that
//! uses them, now resolve to a geometrically different entity than before
//! (`referenceChanged`) or to nothing (`referenceLost`). This is the "a
//! delete / reorder / edit silently re-pointed a reference" warning of
//! ADR 0002. Comparison rules are those of tools/naming_stability.mjs
//! (kind, direction, neighbour kinds, size ratio, position), so ordinary
//! dimension edits that keep the topology do not warn.

use crate::engine::{Exec, TraceSink};
use crate::regen_cache;
use ck_core::linalg::Vec3;
use ck_json::{Json, Obj};
use ck_model::regen::{self, FeatureError, FeatureExec, RegenOpts, RegenState};
use ck_model::{Feature, Part};
use ck_ops::agent::{self, Sig};
use ck_server::session::Cx;
use ck_server::trace::Trace;
use std::rc::Rc;

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

/// A reference, where it was pointing (its signature), and for one that resolves to nothing the facts of the body at that
/// feature, to look for what it may mean now.
type Recorded = (RefKey, Option<[f64; 3]>, Option<Sig>, Option<Rc<agent::Facts>>);

struct Recorder<'a> {
    inner: Exec<'a>,
    /// feature id -> recorded refs
    refs: Vec<(String, Vec<Recorded>)>,
    /// signatures captured when a sketch/plane asked for a face frame
    frames: Vec<(String, Option<Sig>, Option<Rc<agent::Facts>>)>,
    /// gather the facts of the referenced entities only (the reference path gathers the whole body)
    light: bool,
}

impl Recorder<'_> {
    fn snapshot(&self, f: &Feature) -> Vec<Recorded> {
        let (mut faces, mut edges) = (vec![], vec![]);
        collect(&f.raw, &mut faces, &mut edges);
        if faces.is_empty() && edges.is_empty() {
            return vec![];
        }
        let Some(body) = self.inner.brep.body.as_ref() else { return vec![] };
        let pairs: Vec<Vec<String>> = edges
            .iter()
            .map(|(names, _)| {
                let mut k = names.clone();
                k.sort();
                k.dedup();
                k
            })
            .collect();
        let facts = if self.light { agent::gather_for(body, &agent::Want { faces: &faces, pairs: &pairs }) } else { agent::gather(body) };
        // the whole body's facts, gathered once and only for a reference that resolves to nothing
        let full: std::cell::OnceCell<Rc<agent::Facts>> = std::cell::OnceCell::new();
        let pool = || full.get_or_init(|| Rc::new(agent::gather(body))).clone();
        let mut out: Vec<Recorded> = faces.into_iter().map(|id| {
            let sig = agent::face_sig(&facts, &id);
            let lost = sig.is_none().then(&pool);
            (RefKey::Face(id), None, sig, lost)
        }).collect();
        let mut seen: Vec<(Vec<String>, usize)> = vec![];
        for (names, at) in edges {
            let mut key = names.clone();
            key.sort();
            let ord = seen.iter().filter(|(k, _)| *k == key).count();
            seen.push((key.clone(), ord));
            let sig = agent::edge_sig(&facts, &key, at.map(Vec3::from_array));
            let lost = sig.is_none().then(&pool);
            out.push((RefKey::Edge(key, ord), at, sig, lost));
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
        if !self.frames.iter().any(|(f, _, _)| f == face) {
            let light = self.light;
            let sig = self.inner.brep.body.as_ref().and_then(|b| {
                let want = [face.to_string()];
                let facts = if light { agent::gather_for(b, &agent::Want { faces: &want, pairs: &[] }) } else { agent::gather(b) };
                agent::face_sig(&facts, face)
            });
            let lost = if sig.is_none() { self.inner.brep.body.as_ref().map(|b| Rc::new(agent::gather(b))) } else { None };
            self.frames.push((face.to_string(), sig, lost));
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

/// Regenerate `part` privately and record its references. `hit`: a cached state of a prefix of `part` to continue from
/// (the features before it are not run, so their references are not recorded: both documents share them unchanged).
fn run(part: &Part, hit: Option<regen_cache::Hit>, light: bool) -> Run {
    let mut trace = Trace::off();
    let mut rec = Recorder {
        inner: Exec { brep: ck_model::features::Brep::new(), sink: TraceSink::new(&mut trace), auto_heal: None },
        refs: vec![],
        frames: vec![],
        light,
    };
    let resume = hit.map(|h| {
        rec.inner.brep.load(&h.snap);
        regen::Resume { next: h.next, result: h.result, axes: h.axes }
    });
    let res = regen::regen_from(part, &RegenOpts::default(), &mut rec, resume, None);
    // sketch / plane features reach faces through face_frame, not exec
    for f in &part.features {
        if f.suppressed || rec.refs.iter().any(|(id, _)| *id == f.id) {
            continue;
        }
        let (mut faces, mut edges) = (vec![], vec![]);
        collect(&f.raw, &mut faces, &mut edges);
        let got: Vec<Recorded> = faces
            .into_iter()
            .filter_map(|id| rec.frames.iter().find(|(n, _, _)| *n == id).map(|(n, s, p)| (RefKey::Face(n.clone()), None, s.clone(), p.clone())))
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

/// The cached state of the prefix both documents share (so neither has to rebuild it), when there is one.
/// `None` (a cold or evicted prefix, `noCache`): both are regenerated from the start.
fn shared_prefix(base: &Part, base_json: &Json, part: &Part, part_json: &Json) -> Option<(usize, regen_cache::Hit, regen_cache::Hit)> {
    let ko = regen_cache::KeyOpts { sketch_status: false, solve_sketches: false, auto_heal: None };
    let o = RegenOpts::default();
    let (na, nb) = (regen::run_len(base, &o), regen::run_len(part, &o));
    let (ka, kb) = (regen_cache::chain_keys(base_json, &ko, na), regen_cache::chain_keys(part_json, &ko, nb));
    // keys[i] covers the document level and features[..i]: the longest i where both agree
    let p = (0..ka.len().min(kb.len())).take_while(|&i| ka[i] == kb[i]).count().saturating_sub(1);
    if p == 0 {
        return None;
    }
    let ha = regen_cache::with(|c| c.lookup(&ka[..=p]))?;
    let hb = regen_cache::with(|c| c.lookup(&kb[..=p]))?;
    Some((p, ha, hb))
}

/// One unambiguous repair: in `feature`, the reference `from` becomes `to` (one face id, or the two face ids of an edge pick).
struct Repair {
    feature: String,
    from: RefKey,
    to: Vec<String>,
}

/// Rewrite the reference in the feature of `doc`. A face id is replaced wherever the feature's JSON holds it; an edge pick is
/// found by its sorted face pair (the ordinal says which of several picks of one pair) and given the new pair, `at` kept.
fn apply_repair(doc: &mut Json, r: &Repair) -> bool {
    let Some(feats) = doc.get_mut("features").and_then(Json::as_arr_mut) else { return false };
    let Some(feat) = feats.iter_mut().find(|f| f.get_str("id") == Some(r.feature.as_str())) else { return false };
    match &r.from {
        RefKey::Face(id) => replace_face(feat, id, &r.to[0]),
        RefKey::Edge(pair, ord) => {
            let mut seen = 0usize;
            replace_edge(feat, pair, *ord, &r.to, &mut seen)
        }
    }
}

fn replace_face(j: &mut Json, from: &str, to: &str) -> bool {
    match j {
        Json::Str(s) if s == from => {
            *s = to.to_string();
            true
        }
        Json::Arr(a) => a.iter_mut().fold(false, |acc, x| replace_face(x, from, to) | acc),
        Json::Obj(m) => m.iter_mut().fold(false, |acc, (_, v)| replace_face(v, from, to) | acc),
        _ => false,
    }
}

fn replace_edge(j: &mut Json, pair: &[String], ord: usize, to: &[String], seen: &mut usize) -> bool {
    match j {
        Json::Arr(a) => a.iter_mut().fold(false, |acc, x| replace_edge(x, pair, ord, to, seen) | acc),
        Json::Obj(m) => {
            let names: Option<Vec<String>> = m.iter().find(|(k, _)| k == "faces").and_then(|(_, v)| v.as_arr()).map(|a| a.iter().filter_map(|x| x.as_str().map(String::from)).collect());
            let is_pick = m.iter().any(|(k, _)| k == "at");
            if let (Some(mut names), true) = (names, is_pick) {
                if names.len() == 2 {
                    names.sort();
                    names.dedup();
                    if names == pair {
                        let hit = *seen == ord;
                        *seen += 1;
                        if hit {
                            if let Some((_, v)) = m.iter_mut().find(|(k, _)| k == "faces") {
                                *v = Json::Arr(to.iter().map(|f| Json::from(f.as_str())).collect());
                            }
                            return true;
                        }
                        return false;
                    }
                }
            }
            m.iter_mut().fold(false, |acc, (_, v)| replace_edge(v, pair, ord, to, seen) | acc)
        }
        _ => false,
    }
}

pub fn reply(cx: &mut Cx) -> Result<Json, String> {
    let (base_json, part_json) = (cx.msg.get("base").unwrap_or(&Json::Null), cx.msg.get("part").unwrap_or(&Json::Null));
    let base = Part::from_json(base_json)?;
    let part = Part::from_json(part_json)?;
    // Everything before the first changed feature is identical in both documents, so a reference there cannot have been
    // re-pointed: both runs continue from the cached state of that prefix and only the rest is rebuilt. `noCache: true`
    // regenerates both from the start (the reference behaviour).
    let shared_allowed = cx.msg.get_bool("noCache") != Some(true);
    let shared = if shared_allowed { shared_prefix(&base, base_json, &part, part_json) } else { None };
    let (resumed_at, (ha, hb)) = match shared {
        Some((_, ha, hb)) => (ha.next, (Some(ha), Some(hb))),
        None => (0, (None, None)),
    };
    let light = shared_allowed;
    let (a, b) = (run(&base, ha, light), run(&part, hb, light));
    let mut warnings = vec![];
    let mut repairs: Vec<Repair> = vec![];
    let mut checked = 0usize;
    let mut new_runs: Vec<&(String, Vec<Recorded>)> = b.refs.iter().collect();
    new_runs.sort_by_key(|(id, _)| b.ids.iter().position(|x| x == id));
    for (fid, new_refs) in new_runs {
        let Some((_, old_refs)) = a.refs.iter().find(|(id, _)| id == fid) else { continue };
        for (key, at, after, pool) in new_refs {
            let Some((_, _, before, _)) = old_refs.iter().find(|(k, _, _, _)| k == key) else { continue };
            let Some(before) = before else { continue };
            checked += 1;
            let (code, reason, after_text) = match after {
                None => ("referenceLost", "the reference no longer resolves to anything".to_string(), Json::Null),
                Some(af) => match agent::sig_diff(before, af) {
                    Some(why) => ("referenceChanged", why, Json::from(af.text.as_str())),
                    None => continue,
                },
            };
            let mut w = Obj::new()
                .put("code", code)
                .put("feature", fid.as_str())
                .put("ref", key.to_json(*at))
                .put("reason", reason)
                .put("before", before.text.as_str())
                .put("after", after_text);
            // a lost reference: the faces (or edges) of the body at that feature that the old one could be, closest first. `suggest`
            // only when exactly one fits, so a repair is never a guess between two.
            if let Some(pool) = pool.as_ref().filter(|_| after.is_none()) {
                let ms = agent::matches_for(before, pool);
                let show = |m: &agent::Match| -> Json {
                    if m.faces.is_empty() {
                        Obj::new().put("id", m.id.as_str()).put("text", m.text.as_str()).build()
                    } else {
                        Obj::new().put("faces", Json::Arr(m.faces.iter().map(|f| Json::from(f.as_str())).collect())).put("text", m.text.as_str()).build()
                    }
                };
                w = w.put("candidates", Json::Arr(ms.iter().take(5).map(show).collect()));
                if let [only] = ms.as_slice() {
                    w = w.put("suggest", show(only));
                    repairs.push(Repair { feature: fid.clone(), from: key.clone(), to: if only.faces.is_empty() { vec![only.id.clone()] } else { only.faces.clone() } });
                }
            }
            warnings.push(w.build());
        }
    }
    let removed: Vec<Json> = a.ids.iter().filter(|i| !b.ids.contains(i)).map(|s| Json::from(s.as_str())).collect();
    let added: Vec<Json> = b.ids.iter().filter(|i| !a.ids.contains(i)).map(|s| Json::from(s.as_str())).collect();
    let common_a: Vec<&String> = a.ids.iter().filter(|i| b.ids.contains(i)).collect();
    let common_b: Vec<&String> = b.ids.iter().filter(|i| a.ids.contains(i)).collect();
    let errs = b.errors.iter().map(|(f, c)| Json::Arr(vec![Json::from(f.as_str()), Json::from(c.as_str())])).collect();
    let mut out = Obj::new()
        .put("ok", warnings.is_empty())
        .put("checked", checked as f64)
        .put("warnings", Json::Arr(warnings))
        .put("features", Obj::new().put("removed", Json::Arr(removed)).put("added", Json::Arr(added)).put("reordered", common_a != common_b))
        .put("errors", Json::Arr(errs));
    // `repair: true`: the document with every unambiguous suggestion applied (the rest untouched), and the list of what was changed
    if cx.msg.get_bool("repair") == Some(true) {
        let mut fixed = part_json.clone();
        let mut done = vec![];
        for r in &repairs {
            if apply_repair(&mut fixed, r) {
                let from = match &r.from {
                    RefKey::Face(id) => Json::from(id.as_str()),
                    RefKey::Edge(fs, _) => Json::Arr(fs.iter().map(|f| Json::from(f.as_str())).collect()),
                };
                let to = if r.to.len() == 1 { Json::from(r.to[0].as_str()) } else { Json::Arr(r.to.iter().map(|f| Json::from(f.as_str())).collect()) };
                done.push(Obj::new().put("feature", r.feature.as_str()).put("from", from).put("to", to).build());
            }
        }
        out = out.put("repaired", fixed).put("repairs", Json::Arr(done));
    }
    // which features were skipped: `checked` then counts only the references from the first changed feature on
    if cx.msg.get_bool("cacheStats") == Some(true) {
        out = out.put("cacheStats", Obj::new().put("resumedAt", resumed_at as f64).build());
    }
    Ok(out.build())
}
