//! `combine` (alias `bodyBoolean`): a boolean between whole lumps of one document.
//!
//! ```json
//! {"type":"combine","id":"F9","op":"cut","target":{"face":"F2.C1"},
//!  "tools":[{"face":"F4.S.e1"},{"lump":2}],"keepTools":false,
//!  "tolerance":1e-5,"autoFuzzy":true}
//! ```
//!
//! * `op`: `union` (`add`, `join`), `cut` (`subtract`), `intersect` (`common`) or `split`.
//! * `target` (also `a`): which lump is operated on: a face name (the bare string or `{face}`; the lump holding
//!   that face), `{lump: n}` (index in `components` order, as `massProps {bodies:true}` lists them) or `{body: name}`
//!   (the lump with that body name; several lumps with the name: `ambiguousBody {name, candidates}`). `tools`
//!   (also `b`, a single selection or an array): the lumps that act on it (`{body: name}` selects every lump
//!   with the name).
//! * `body` (optional): the name of the result lump (the first result lump of a `split`); default: the target's
//!   name, if it has one. A name that another lump that stays already has is `bodyNameConflict`.
//! * `union`: target united with every tool, one after the other (the machinery is binary, so the n-ary union is
//!   sequential, in the listed order; tools that only overlap each other still end up in one lump).
//!   `cut`: target minus every tool. `intersect`: target inside every tool. `split`: the target cut into the part
//!   outside all tools and the part inside them, two lumps.
//! * `keepTools` (default false): the tool lumps stay as they were next to the result; otherwise they are consumed.
//! * `union` of tools that do not touch the target (or each other): a boolean union cannot fuse them, so they stay
//!   separate lumps under their own names (as documents written before this rule expect: later features select them by
//!   name) and the warning `unionDisjoint {tools, kept}` lists them. `disjoint: "join"` makes the union one body of
//!   several lumps instead: the tools are consumed, every lump of the result takes the result name (`body`, else the
//!   target's). With `keepTools` a tool that touches nothing is not merged (merging would duplicate it): it stays as the
//!   kept tool, listed in the warning with `kept: true`.
//! * `cut` whose tools remove nothing from the target (a tool that misses it, is tangent, touches only along an edge
//!   or a point, or lies wholly inside a cavity): not an error. The result is the target unchanged (same faces, names
//!   and lump), the tools are consumed unless `keepTools` (as in any cut), and the warning `cutNoEffect {tools, kept}`
//!   names the tools. `split` that changes nothing stays the error `cutNoEffect`: it promises two lumps.
//! * Faces keep their persistent names (a face split by the operation gets the usual `~k` suffix), so references to
//!   the faces of both operands survive; the result lumps come after the untouched lumps.
//! * `tolerance` / `autoFuzzy`: the fuzzy value of the boolean, as on every other boolean-capable feature.
//! * Errors: `nothingToCut` (no body), `missingReference` (selection not found), `invalidValue {detail}` (unknown op,
//!   no tools, a tool that is the target), `booleanFailed`, `booleanVolumeMismatch`, `cutNoEffect` (split changed
//!   nothing; for `cut` it is the warning above), `combineEmpty {detail}` (intersect of lumps that do not overlap).

use super::{boolean_span, fuzzy_boolean, volume_mismatch, Brep, CUT_NO_EFFECT, TOL};
use crate::regen::{codes, FeatureError, RegenState};
use crate::Feature;
use ck_json::Json;
use ck_ops::boolean::Op as BOp;
use ck_topo::body::{Body, FaceId};

pub const COMBINE_EMPTY: &str = "combineEmpty";

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CombineOp {
    Union,
    Cut,
    Intersect,
    Split,
}

fn invalid(detail: &str) -> FeatureError {
    FeatureError::new(codes::INVALID_VALUE).with("detail", Json::Str(detail.into()))
}

fn missing() -> FeatureError {
    FeatureError::new(codes::MISSING_REFERENCE)
}

/// One lump selector: a face name or a lump index.
#[derive(Debug, Clone, PartialEq)]
pub enum Sel {
    Face(String),
    Lump(usize),
    Body(String),
}

fn sels(j: Option<&Json>) -> Result<Vec<Sel>, FeatureError> {
    let one = |j: &Json| -> Result<Sel, FeatureError> {
        if let Some(s) = j.as_str() {
            return Ok(Sel::Face(s.to_string()));
        }
        if let Some(s) = j.get_str("face") {
            return Ok(Sel::Face(s.to_string()));
        }
        if let Some(n) = j.get("lump").and_then(|n| n.as_f64()) {
            return Ok(Sel::Lump(n as usize));
        }
        if let Some(n) = j.get_str("body") {
            return Ok(Sel::Body(n.to_string()));
        }
        Err(invalid("a selection is a face name, {face}, {lump} or {body}"))
    };
    match j {
        None | Some(Json::Null) => Ok(vec![]),
        Some(Json::Arr(a)) => a.iter().map(one).collect(),
        Some(j) => Ok(vec![one(j)?]),
    }
}

pub struct Spec {
    pub op: CombineOp,
    pub target: Vec<Sel>,
    pub tools: Vec<Sel>,
    pub keep_tools: bool,
    /// Name of the result lump.
    pub body: Option<String>,
    /// `disjoint: "join"` (union only): tools that touch nothing are consumed into the one result body.
    pub join_disjoint: bool,
}

pub fn parse(raw: &Json) -> Result<Spec, FeatureError> {
    let op = match raw.get_str("op").unwrap_or("") {
        "union" | "add" | "join" | "unite" | "fuse" => CombineOp::Union,
        "cut" | "subtract" | "minus" => CombineOp::Cut,
        "intersect" | "common" => CombineOp::Intersect,
        "split" => CombineOp::Split,
        other => return Err(invalid(&format!("unknown op {other:?} (union, cut, intersect, split)"))),
    };
    let target = sels(raw.get("target").or_else(|| raw.get("a")))?;
    let tools = sels(raw.get("tools").or_else(|| raw.get("tool")).or_else(|| raw.get("b")))?;
    if target.len() != 1 {
        return Err(invalid("target selects exactly one lump"));
    }
    if tools.is_empty() {
        return Err(invalid("tools lists at least one lump"));
    }
    Ok(Spec { op, target, tools, keep_tools: crate::sketch::jtruthy(raw.get("keepTools")), body: super::bodies::body_name(raw)?, join_disjoint: raw.get_str("disjoint") == Some("join") })
}

/// Result of [`combine_bodies`].
pub struct Out {
    pub body: Body,
    /// Fuzzy value that decided one of the booleans, if any.
    pub tolerance_used: Option<f64>,
    pub spans: Vec<super::Span>,
    /// `union` tools that did not touch what they were united with (see [`UNION_DISJOINT`]).
    pub disjoint: Vec<String>,
    /// `cut` tools that removed nothing from the target, listed only when none of them did (see [`CUT_NO_EFFECT`]).
    pub no_effect: Vec<String>,
}

/// Warning of a `union` whose tool does not touch the target (or the earlier tools): the result holds it as a lump of
/// its own. `{tools: [name or "lump n", ..], kept: bool}`.
pub const UNION_DISJOINT: &str = "unionDisjoint";

/// Warning of a `cut` none of whose tools removed anything from the target: the result is the target unchanged.
/// `{tools: [name or "lump n", ..], kept: bool}` (`kept`: `keepTools`, the tools stay next to the target).
pub const CUT_NO_EFFECT_WARNING: &str = CUT_NO_EFFECT;

fn comp_of(b: &Body, comps: &[Vec<FaceId>], s: &Sel) -> Result<usize, FeatureError> {
    match s {
        Sel::Lump(n) => (*n < comps.len()).then_some(*n).ok_or_else(missing),
        Sel::Face(name) => {
            let fid = b.find_face(name).ok_or_else(|| crate::refs::missing_face([b], name))?;
            comps.iter().position(|c| c.contains(&fid)).ok_or_else(missing)
        }
        Sel::Body(name) => {
            let ix = comps_of(b, comps, s)?;
            if ix.len() > 1 {
                let lumps: Vec<_> = ix.iter().map(|&i| b.face_lump(comps[i][0])).collect();
                return Err(super::bodies::ambiguous(b, name, &lumps));
            }
            Ok(ix[0])
        }
    }
}

/// Every component a selector selects (a `{body}` name may select several lumps).
fn comps_of(b: &Body, comps: &[Vec<FaceId>], s: &Sel) -> Result<Vec<usize>, FeatureError> {
    match s {
        Sel::Body(name) => {
            let lumps = super::bodies::lumps_named(Some(b), name)?;
            let ix: Vec<usize> = (0..comps.len()).filter(|&i| comps[i].first().is_some_and(|&f| lumps.contains(&b.face_lump(f)))).collect();
            if ix.is_empty() {
                return Err(missing());
            }
            Ok(ix)
        }
        _ => Ok(vec![comp_of(b, comps, s)?]),
    }
}

/// The faces of each lump, in `components` order of the lump's first component. A sealed void inside a lump is a
/// connected component of its own, but it belongs to the lump around it: merging them keeps the void with its solid
/// (a `{body}` selector must not read it as a second lump, and a cut must not treat it as a tool).
fn lump_components(b: &Body) -> Vec<Vec<FaceId>> {
    let mut out: Vec<Vec<FaceId>> = vec![];
    let mut lumps: Vec<ck_topo::body::LumpId> = vec![];
    for c in ck_topo::region::components(b) {
        let l = b.face_lump(c[0]);
        match lumps.iter().position(|&x| x == l) {
            Some(i) => {
                out[i].extend(c);
                out[i].sort();
            }
            None => {
                lumps.push(l);
                out.push(c);
            }
        }
    }
    out
}

fn extract(b: &Body, faces: &[FaceId]) -> Body {
    // `rebuild_in_order` already rebuilds the shells and lumps of what it built
    ck_ops::boolean::assemble::rebuild_in_order(b, faces)
}

/// Run the operation on the lumps of `body`.
/// Faces of `b` that are planar with a mean width (twice the area over the perimeter) under `SLIVER_WIDTH`.
const SLIVER_WIDTH: f64 = 1e-5;
fn sliver_faces(b: &Body) -> usize {
    use ck_geom::surface::Surface;
    b.face_ids()
        .filter(|&f| {
            let face = b.face(f);
            let Some(&l) = face.loops.first() else { return false };
            if !matches!(face.surface, Surface::Plane { .. }) {
                return false;
            }
            let area = ck_topo::orient::loop_uv_area(b, l).abs();
            let per: f64 = b.lp(l).coedges.iter().map(|&c| b.edge_length(b.coedge(c).edge)).sum();
            area > 0.0 && per > 0.0 && 2.0 * area / per < SLIVER_WIDTH
        })
        .count()
}

/// The same boolean with the tool moved by a small step (1e-4 mm) along each axis, the first
/// result that has fewer sliver faces, is no more invalid than the operands and keeps the volume to within 1e-3 of the first result's.
fn unsliver(a: &Body, b: &Body, op: BOp, first: &ck_ops::boolean::BoolOut) -> Option<ck_ops::boolean::BoolOut> {
    use ck_core::linalg::Vec3;
    const STEP: f64 = 1e-4;
    let base = ck_topo::check::check(a, TOL).len() + ck_topo::check::check(b, TOL).len();
    let v0 = ck_topo::mass::mass_props(&first.body).volume;
    let n0 = sliver_faces(&first.body);
    let mut best: Option<(usize, ck_ops::boolean::BoolOut)> = None;
    for d in [Vec3::new(0.0, 0.0, STEP), Vec3::new(0.0, 0.0, -STEP), Vec3::new(STEP, 0.0, 0.0), Vec3::new(-STEP, 0.0, 0.0), Vec3::new(0.0, STEP, 0.0), Vec3::new(0.0, -STEP, 0.0)] {
        let mut t = b.clone();
        t.transform(&ck_core::xform::Xform::translation(d));
        let Ok(o) = ck_ops::boolean::boolean(a, &t, op, TOL) else { continue };
        if o.no_effect || o.mismatch.is_some() || sliver_faces(&o.body) >= n0 || ck_topo::check::check(&o.body, TOL).len() > base {
            continue;
        }
        let v = ck_topo::mass::mass_props(&o.body).volume;
        if (v - v0).abs() > 1e-3 * v0.abs().max(1.0) {
            continue;
        }
        let key = sliver_faces(&o.body) * 100000 + o.body.num_faces();
        if best.as_ref().is_none_or(|(k, _)| key < *k) {
            best = Some((key, o));
        }
    }
    best.map(|(_, o)| o)
}

pub fn combine_bodies(body: &Body, spec: &Spec, fid: &str, fuzz: f64, auto: bool) -> Result<Out, FeatureError> {
    let comps = lump_components(body);
    let t_ix = comp_of(body, &comps, &spec.target[0])?;
    let mut tool_ix: Vec<usize> = vec![];
    for s in &spec.tools {
        for i in comps_of(body, &comps, s)? {
            if i == t_ix {
                return Err(invalid("a tool is the target lump"));
            }
            if !tool_ix.contains(&i) {
                tool_ix.push(i);
            }
        }
    }
    // the name of the result: `body`, else the target's own; a name a surviving lump has is a conflict
    let target_name = body.lump(body.face_lump(comps[t_ix][0])).name.clone();
    let result_name = spec.body.clone().or(target_name);
    if let Some(n) = &spec.body {
        let mut gone: Vec<ck_topo::body::LumpId> = comps[t_ix].iter().map(|&f| body.face_lump(f)).collect();
        if !spec.keep_tools {
            gone.extend(tool_ix.iter().flat_map(|&i| comps[i].iter().map(|&f| body.face_lump(f))));
        }
        super::bodies::check_free(Some(body), n, fid, &gone)?;
    }
    let target = extract(body, &comps[t_ix]);
    let tools: Vec<Body> = tool_ix.iter().map(|&i| extract(body, &comps[i])).collect();
    let mut spans = vec![];
    let mut used: Option<f64> = None;

    let mut run = |a: &Body, b: &Body, op: BOp| -> Result<ck_ops::boolean::BoolOut, FeatureError> {
        let (mut r, u) = fuzzy_boolean(a, b, op, TOL, fuzz, auto);
        if u.is_some() {
            used = u;
        }
        // a tool wall that runs through an edge of the host to within a few welding tolerances leaves a face
        // thinner than the tolerance in the result (a dovetail tail whose slanted wall ends on the corner of its
        // blank), which the next boolean on that body cannot close: move the tool a hair off the edge instead
        if let Ok(out) = &r {
            if !out.no_effect && fuzz == 0.0 && sliver_faces(&out.body) > sliver_faces(a).max(sliver_faces(b)) {
                if let Some(better) = unsliver(a, b, op, out) {
                    r = Ok(better);
                }
            }
        }
        match r {
            Ok(out) => {
                spans.push(boolean_span(op, a.num_faces(), b.num_faces(), out.body.num_faces()));
                match &out.mismatch {
                    Some(m) => Err(volume_mismatch(m)),
                    None => Ok(out),
                }
            }
            Err(e) => {
                spans.push(boolean_span(op, a.num_faces(), b.num_faces(), a.num_faces()));
                Err(FeatureError::new(&e))
            }
        }
    };

    let mut pieces: Vec<Body> = vec![];
    let mut disjoint: Vec<String> = vec![];
    let mut no_effect: Vec<String> = vec![];
    // tools (indices into `tool_ix`) that were united into the result; with `keepTools` a tool that touches nothing is
    // not merged, it stays the kept tool it is (merging it would duplicate it as a second lump)
    let mut merged = vec![true; tool_ix.len()];
    match spec.op {
        CombineOp::Union => {
            let mut cur = target;
            for (k, t) in tools.iter().enumerate() {
                let out = run(&cur, t, BOp::Union)?;
                if !out.no_effect {
                    // no lump gained or lost: the tool shares nothing with what it was united with
                    if out.body.num_lumps() >= cur.num_lumps() + t.num_lumps() {
                        disjoint.push(match &body.lump(body.face_lump(comps[tool_ix[k]][0])).name {
                            Some(n) => n.clone(),
                            None => format!("lump {}", tool_ix[k]),
                        });
                        if spec.keep_tools {
                            merged[k] = false;
                            continue;
                        }
                    }
                    cur = out.body;
                }
            }
            pieces.push(cur);
        }
        CombineOp::Cut => {
            let mut cur = target;
            let mut changed = false;
            for t in &tools {
                let out = run(&cur, t, BOp::Cut)?;
                if !out.no_effect {
                    changed = true;
                    cur = out.body;
                }
            }
            // no tool removed anything (misses the target, tangent, touches along an edge or a point, lies in a
            // cavity): the target stays as it was and the warning names the tools; the tools are consumed unless
            // `keepTools`, exactly as in a cut that removes something
            if !changed {
                no_effect = tool_ix
                    .iter()
                    .map(|&i| match &body.lump(body.face_lump(comps[i][0])).name {
                        Some(n) => n.clone(),
                        None => format!("lump {i}"),
                    })
                    .collect();
            }
            pieces.push(cur);
        }
        CombineOp::Intersect => {
            let mut cur = target;
            for t in &tools {
                let out = run(&cur, t, BOp::Intersect)?;
                if out.body.num_faces() == 0 || (out.no_effect && out.body.num_faces() == 0) {
                    return Err(FeatureError::new(COMBINE_EMPTY).with("detail", Json::Str("the lumps do not overlap".into())));
                }
                if !out.no_effect {
                    cur = out.body;
                }
            }
            pieces.push(cur);
        }
        CombineOp::Split => {
            let mut cutter = tools[0].clone();
            for t in &tools[1..] {
                let out = run(&cutter, t, BOp::Union)?;
                if !out.no_effect {
                    cutter = out.body;
                }
            }
            let outside = run(&target, &cutter, BOp::Cut)?;
            let inside = run(&target, &cutter, BOp::Intersect)?;
            if outside.no_effect || inside.no_effect || outside.body.num_faces() == 0 || inside.body.num_faces() == 0 {
                return Err(FeatureError::new(CUT_NO_EFFECT));
            }
            pieces.push(outside.body);
            pieces.push(inside.body);
        }
    }

    // untouched lumps first, then the result pieces, then (keepTools) the tools as they were
    let mut consumed = vec![false; body.faces.len()];
    for i in std::iter::once(t_ix).chain(tool_ix.iter().copied()) {
        for &f in &comps[i] {
            consumed[f.idx()] = true;
        }
    }
    let rest: Vec<FaceId> = body.face_ids().filter(|&f| !consumed[f.idx()]).collect();
    let mut out = if rest.is_empty() { Body::new() } else { extract(body, &rest) };
    // The lumps the operation did not touch keep their names as they are: `restore` (bodies.rs) gives a name back by
    // the lump's origin feature, which two lumps share when a result piece was cut from a lump that is also kept
    // (an intersect with `keepTools` made from the tool's own faces), and one of them would lose its name.
    if !rest.is_empty() {
        let by_face: std::collections::HashMap<String, String> = body
            .face_ids()
            .filter(|&f| !consumed[f.idx()])
            .filter_map(|f| Some((body.face_id_str(f), body.lump(body.face_lump(f)).name.clone()?)))
            .collect();
        let ls: Vec<ck_topo::body::LumpId> = out.lump_ids().collect();
        for l in ls {
            let name = ck_topo::mass::lump_faces(&out, l).into_iter().find_map(|f| by_face.get(&out.face_id_str(f)).cloned());
            out.lumps[l.idx()].name = name;
        }
    }
    let mut first_result: Option<ck_topo::body::LumpId> = None;
    let mut result_lumps: Vec<ck_topo::body::LumpId> = vec![];
    for p in &pieces {
        if p.num_faces() > 0 {
            let ids = out.append(p);
            first_result = first_result.or(ids.first().copied());
            result_lumps.extend(ids);
        }
    }
    if spec.keep_tools {
        for (t, &i) in tools.iter().zip(&tool_ix) {
            let name = body.lump(body.face_lump(comps[i][0])).name.clone();
            if let Some(&l) = out.append(t).first() {
                out.lumps[l.idx()].name = name;
            }
        }
    }
    renumber(&mut out);
    if let (Some(l), Some(n)) = (first_result, &result_name) {
        out.lumps[l.idx()].name = Some(n.clone());
    }
    // `disjoint: "join"`: a union is one body; a lump of it that touched nothing (a disjoint tool) is part of the result
    // and takes its name; the consumed tools' own names are gone with them. Without it such a lump keeps the tool's name
    // (what documents written before the option rely on)
    if spec.op == CombineOp::Union && spec.join_disjoint {
        for &l in &result_lumps {
            out.lumps[l.idx()].name = result_name.clone();
        }
    }
    Ok(Out { body: out, tolerance_used: used, spans, disjoint, no_effect })
}

fn renumber(b: &mut Body) {
    super::renumber_duplicates(b);
}

pub fn exec_combine(br: &mut Brep, _state: &mut RegenState, f: &Feature) -> Result<(), FeatureError> {
    let Some(body) = br.body.as_ref() else { return Err(FeatureError::new(codes::NOTHING_TO_CUT)) };
    let spec = parse(&f.raw)?;
    let out = combine_bodies(body, &spec, &f.id, br.fuzz, br.auto_fuzz)?;
    br.spans.extend(out.spans);
    if !out.disjoint.is_empty() {
        let w = FeatureError::new(UNION_DISJOINT)
            .with("tools", Json::Arr(out.disjoint.iter().map(|n| Json::Str(n.clone())).collect()))
            .with("kept", Json::Bool(spec.keep_tools));
        _state.result.warnings.push(crate::regen::RegenError { feature: f.id.clone(), error: w });
    }
    if !out.no_effect.is_empty() {
        let w = FeatureError::new(CUT_NO_EFFECT_WARNING)
            .with("tools", Json::Arr(out.no_effect.iter().map(|n| Json::Str(n.clone())).collect()))
            .with("kept", Json::Bool(spec.keep_tools));
        _state.result.warnings.push(crate::regen::RegenError { feature: f.id.clone(), error: w });
    }
    if out.tolerance_used.is_some() {
        br.fuzz_used = out.tolerance_used;
    }
    br.body = (out.body.num_faces() > 0).then_some(out.body);
    Ok(())
}

/// The `bodyBoolean` request: the same operation on a copy of the current body, nothing committed. Reply:
/// `{ok, op, lumps, faces, volume, toleranceUsed?}` or `{ok:false, error:{code, ...}}`.
pub fn preview(body: &Body, msg: &Json) -> Json {
    use ck_json::Obj;
    let fuzz = msg.get("tolerance").and_then(|t| t.as_f64()).filter(|t| t.is_finite() && *t > 0.0).unwrap_or(0.0);
    let auto = crate::sketch::jtruthy(msg.get("autoFuzzy"));
    let r = parse(msg).and_then(|s| combine_bodies(body, &s, "bodyBoolean", fuzz, auto));
    match r {
        Ok(o) => {
            let mp = ck_topo::mass::mass_props(&o.body);
            let faces: Vec<Json> = o.body.face_ids().map(|f| Json::Str(o.body.face_id_str(f))).collect();
            let mut ob = Obj::new()
                .put("ok", true)
                .put("lumps", ck_topo::region::components(&o.body).len() as f64)
                .put("volume", mp.volume)
                .put("area", mp.area)
                .put("faces", Json::Arr(faces));
            if let Some(t) = o.tolerance_used {
                ob = ob.put("toleranceUsed", t);
            }
            ob.build()
        }
        Err(e) => Obj::new().put("ok", false).put("error", Json::Obj(vec![("code".into(), Json::Str(e.code.clone())), ("params".into(), Json::Obj(e.params.clone()))])).build(),
    }
}
