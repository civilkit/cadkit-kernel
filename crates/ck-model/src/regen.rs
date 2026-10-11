//! Regen program execution: feature ordering, upTo, rollback, suppression,
//! sketch solving + profiles, reference planes, error collection.
//!
//! Input: a `crate::Part` + options (`{upTo}`) + a [`FeatureExec`] hook that
//! executes geometry features (wired to ck-ops later). Output:
//! [`RegenResult`], whose `to_json` reproduces the oracle's key order:
//! `{"errors","components","planes","markers","sketches","sketches3",
//! "curves","body","surfaces"}`.
//!
//! Pinned by oracle probes:
//! * `opts.upTo = n` (number >= 0) regenerates `features[..n]`; numeric
//!   `part.rollback = n` likewise (the smaller wins); non-numbers ignored;
//! * `suppressed: true` features are skipped;
//! * error wire form is `["F1", {"code": "...", "params": {...}}]`, keyed by
//!   the feature id (`""` when missing); a failing feature produces no output;
//! * unknown `type` -> `unknownFeature` with `params.type`;
//! * sketch: plane missing/unresolvable -> `missingReference`; a constraint
//!   `expr` that does not evaluate -> `badExpression`; when an `expr` changes
//!   a constraint `value` (|Δ| > 1e-12, client `applyVariablesToSketches`)
//!   the sketch is re-solved (failure leaves the points unchanged, no error);
//!   output `["F1", {"frame", "profiles", "sketch"}]`;
//! * `plane` features: base frame (std / earlier plane feature / face via the
//!   hook), `flip` negates x and z, then `o += z * offset`; output
//!   `["F12", {"o","x","y","z"}]` even when `visible:false`;
//! * `sketch3d` features echo `["F1", {"sketch": ...}]` into `sketches3`;
//! * sketch-driven features whose `sketch` does not name an already
//!   regenerated sketch -> `missingSketch`; `insertPart` without a valid
//!   `part` -> `insertPartMissing`.

use crate::expr::{self, Vars};
use crate::features::FeatureKind;
use crate::sketch::profile::{profiles, Profiles};
use crate::sketch::solver::{solve_sketch, SolveOpts};
use crate::{Feature, GeomRef, Part, Sketch};
use ck_core::linalg::{Frame, Vec3};
use ck_json::Json;

/// Regen options (`msg.opts`).
#[derive(Debug, Clone, Default)]
pub struct RegenOpts {
    /// Regenerate only the first `up_to` features.
    pub up_to: Option<u32>,
    /// Ignore `part.rollback` (pinned: nested insertPart regens do).
    pub ignore_rollback: bool,
}

impl RegenOpts {
    /// Parse `msg.opts` (`{"upTo": n}`; negative / non-numbers ignored).
    pub fn from_json(j: Option<&Json>) -> RegenOpts {
        let up_to = j.and_then(|o| o.get("upTo")).and_then(|v| v.as_f64()).filter(|v| *v >= 0.0).map(|v| v as u32);
        RegenOpts { up_to, ignore_rollback: false }
    }
}

/// Cache tags seen in app.pretty.js: main, asm, bridge, plain, base, sketch, preview.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RegenTag {
    Main,
    Asm,
    Bridge,
    Plain,
    Base,
    Sketch,
    Preview,
}

impl RegenTag {
    pub fn from_wire(s: &str) -> Option<RegenTag> {
        Some(match s {
            "main" => RegenTag::Main,
            "asm" => RegenTag::Asm,
            "bridge" => RegenTag::Bridge,
            "plain" => RegenTag::Plain,
            "base" => RegenTag::Base,
            "sketch" => RegenTag::Sketch,
            "preview" => RegenTag::Preview,
            _ => return None,
        })
    }
}

/// Error codes used by the driver (kernel.strings).
pub mod codes {
    pub const UNKNOWN_FEATURE: &str = "unknownFeature";
    pub const MISSING_REFERENCE: &str = "missingReference";
    pub const MISSING_SKETCH: &str = "missingSketch";
    pub const BAD_EXPRESSION: &str = "badExpression";
    pub const INSERT_PART_MISSING: &str = "insertPartMissing";
    pub const NO_CLOSED_PROFILE: &str = "noClosedProfile";
    pub const NOTHING_TO_CUT: &str = "nothingToCut";
    pub const INVALID_VALUE: &str = "invalidValue";
}

/// Feature types the kernel's regen knows (anything else -> `unknownFeature`).
/// Pinned by probing each type with an empty payload; e.g. `convert`,
/// `rebuildFace`, `matchEdges`, `refCurve` are separate messages, not features.
pub const KNOWN_TYPES: &[&str] = &[
    "sketch", "sketch3d", "plane", "axis", "extrude", "revolve", "sweep", "loft", "fillet", "chamfer", "shell", "rib",
    "lip", "dome", "splitLine", "draft", "moveFace", "hole", "lpattern", "cpattern", "mirror", "spattern",
    "fillpattern", "curvepattern", "moveBody", "mate", "insertPart", "scale", "surfaceExtrude", "surfaceRevolve",
    "surfacePlanar", "surfaceOffset", "deleteFace", "fill", "knit", "extend", "freeform", "trimSurface", "thicken",
    "cutWithSurface", "pipeCut", "deleteBody", "editCurve", "projectCurve", "helix",
];

/// The error the oracle reports for each known type when the payload is
/// empty (no references at all, no body yet) — a checklist for the geometry
/// implementations of [`FeatureExec`].
pub const EMPTY_FEATURE_ERRORS: &[(&str, &str)] = &[
    ("axis", "missingReference"),
    ("extrude", "missingSketch"),
    ("revolve", "missingSketch"),
    ("sweep", "missingSketch"),
    ("loft", "loftProfiles"),
    ("fillet", "invalidValue"),
    ("chamfer", "invalidValue"),
    ("shell", "nothingToCut"),
    ("rib", "nothingToCut"),
    ("lip", "nothingToCut"),
    ("draft", "nothingToCut"),
    ("dome", "nothingToCut"),
    ("splitLine", "nothingToCut"),
    ("moveFace", "nothingToCut"),
    ("hole", "missingSketch"),
    ("lpattern", "missingReference"),
    ("cpattern", "missingReference"),
    ("mirror", "missingReference"),
    ("spattern", "missingSketch"),
    ("fillpattern", "nothingToCut"),
    ("curvepattern", "missingPath"),
    ("moveBody", "nothingToCut"),
    ("scale", "nothingToCut"),
    ("mate", "nothingToCut"),
    ("insertPart", "insertPartMissing"),
    ("plane", "missingReference"),
    ("surfaceExtrude", "noClosedProfile"),
    ("surfaceRevolve", "missingSketch"),
    ("surfacePlanar", "missingSketch"),
    ("surfaceOffset", "missingReference"),
    ("deleteFace", "missingReference"),
    ("fill", "missingReference"),
    ("knit", "missingReference"),
    ("extend", "missingReference"),
    ("freeform", "missingReference"),
    ("trimSurface", "missingReference"),
    ("thicken", "missingReference"),
    ("cutWithSurface", "missingReference"),
    ("pipeCut", "invalidValue"),
    ("deleteBody", "noBodies"),
    ("editCurve", "editCurveNoInput"),
    ("projectCurve", "projectCurveNoFaces"),
    ("helix", "missingSketch"),
];

/// Sketch-driven types for which the driver checks `sketch` itself.
const SKETCH_CONSUMERS: &[&str] =
    &["extrude", "revolve", "sweep", "hole", "helix", "surfaceRevolve", "surfacePlanar", "spattern"];

/// A feature failure: `{"code": "...", "params": {...}}`.
#[derive(Debug, Clone, PartialEq)]
pub struct FeatureError {
    pub code: String,
    pub params: Vec<(String, Json)>,
}

impl FeatureError {
    pub fn new(code: &str) -> FeatureError {
        FeatureError { code: code.to_string(), params: Vec::new() }
    }
    pub fn with(mut self, k: &str, v: Json) -> FeatureError {
        self.params.push((k.to_string(), v));
        self
    }
    /// Trace form: `code` or `code {"k":v}` (oracle span labels).
    pub fn describe(&self) -> String {
        if self.params.is_empty() {
            self.code.clone()
        } else {
            format!("{} {}", self.code, ck_json::to_string(&Json::Obj(self.params.clone())))
        }
    }
}

/// One error entry, wire form `["F4", {"code":..,"params":{..}}]`.
#[derive(Debug, Clone, PartialEq)]
pub struct RegenError {
    pub feature: String,
    pub error: FeatureError,
}

impl RegenError {
    pub fn to_json(&self) -> Json {
        Json::Arr(vec![
            Json::Str(self.feature.clone()),
            Json::Obj(vec![
                ("code".into(), Json::Str(self.error.code.clone())),
                ("params".into(), Json::Obj(self.error.params.clone())),
            ]),
        ])
    }
}

/// A regenerated sketch.
#[derive(Debug, Clone)]
pub struct SketchResult {
    pub id: String,
    pub frame: Frame,
    pub profiles: Profiles,
    /// The (possibly re-solved) sketch.
    pub sketch: Sketch,
}

/// `{"o":[..],"x":[..],"y":[..],"z":[..]}`.
pub fn frame_json(f: &Frame) -> Json {
    let v = |v: Vec3| Json::Arr(vec![Json::Num(v.x), Json::Num(v.y), Json::Num(v.z)]);
    Json::Obj(vec![("o".into(), v(f.o)), ("x".into(), v(f.x)), ("y".into(), v(f.y)), ("z".into(), v(f.z))])
}

/// Everything a regen produces. `errors`, `planes`, `sketches` and
/// `sketches3` are maps keyed by feature id in the kernel (oracle: output is
/// sorted by id string — `F17 < F20 < F3` — and a repeated id overwrites);
/// use the `put_*` helpers, `to_json` sorts. Geometry hooks push into `components`,
/// `markers`, `curves`, `body`, `surfaces` (kept as JSON until the geometry
/// crates define their own result types).
#[derive(Debug, Clone, Default)]
pub struct RegenResult {
    pub errors: Vec<RegenError>,
    pub components: Vec<Json>,
    pub planes: Vec<(String, Frame)>,
    pub markers: Vec<Json>,
    pub sketches: Vec<SketchResult>,
    pub sketches3: Vec<(String, Json)>,
    pub curves: Vec<Json>,
    pub body: Option<Json>,
    pub surfaces: Vec<Json>,
}

fn upsert<T>(v: &mut Vec<T>, key: impl Fn(&T) -> &str, item: T) {
    let k = key(&item).to_string();
    if let Some(slot) = v.iter_mut().find(|x| key(x) == k) {
        *slot = item;
    } else {
        v.push(item);
    }
}

fn sorted<'a, T>(v: &'a [T], key: impl Fn(&T) -> &str) -> Vec<&'a T> {
    let mut out: Vec<&T> = v.iter().collect();
    out.sort_by(|a, b| key(a).cmp(key(b)));
    out
}

impl RegenResult {
    pub fn put_error(&mut self, e: RegenError) {
        upsert(&mut self.errors, |x| &x.feature, e);
    }
    pub fn put_plane(&mut self, id: &str, f: Frame) {
        upsert(&mut self.planes, |x| &x.0, (id.to_string(), f));
    }
    pub fn put_sketch(&mut self, s: SketchResult) {
        upsert(&mut self.sketches, |x| &x.id, s);
    }
    pub fn put_sketch3(&mut self, id: &str, j: Json) {
        upsert(&mut self.sketches3, |x| &x.0, (id.to_string(), j));
    }

    pub fn to_json(&self) -> Json {
        Json::Obj(vec![
            ("errors".into(), Json::Arr(sorted(&self.errors, |e| &e.feature).into_iter().map(|e| e.to_json()).collect())),
            ("components".into(), Json::Arr(self.components.clone())),
            (
                "planes".into(),
                Json::Arr(
                    sorted(&self.planes, |p| &p.0)
                        .into_iter()
                        .map(|(id, f)| Json::Arr(vec![Json::Str(id.clone()), frame_json(f)]))
                        .collect(),
                ),
            ),
            ("markers".into(), Json::Arr(self.markers.clone())),
            (
                "sketches".into(),
                Json::Arr(
                    sorted(&self.sketches, |s| &s.id)
                        .into_iter()
                        .map(|s| {
                            Json::Arr(vec![
                                Json::Str(s.id.clone()),
                                Json::Obj(vec![
                                    ("frame".into(), frame_json(&s.frame)),
                                    ("profiles".into(), s.profiles.to_json()),
                                    ("sketch".into(), s.sketch.to_json()),
                                ]),
                            ])
                        })
                        .collect(),
                ),
            ),
            (
                "sketches3".into(),
                Json::Arr(
                    sorted(&self.sketches3, |s| &s.0)
                        .into_iter()
                        .map(|(id, s)| Json::Arr(vec![Json::Str(id.clone()), Json::Obj(vec![("sketch".into(), s.clone())])]))
                        .collect(),
                ),
            ),
            ("curves".into(), Json::Arr(self.curves.clone())),
            ("body".into(), self.body.clone().unwrap_or(Json::Null)),
            ("surfaces".into(), Json::Arr(self.surfaces.clone())),
        ])
    }
}

/// State visible to feature hooks during a regen.
pub struct RegenState<'p> {
    pub part: &'p Part,
    /// Resolved global variables.
    pub vars: Vars,
    pub result: RegenResult,
}

impl<'p> RegenState<'p> {
    pub fn sketch(&self, id: &str) -> Option<&SketchResult> {
        self.result.sketches.iter().find(|s| s.id == id)
    }
    pub fn plane(&self, id: &str) -> Option<Frame> {
        self.result.planes.iter().find(|p| p.0 == id).map(|p| p.1)
    }
    /// Evaluate a numeric field of a feature (`expr::eval_field`).
    pub fn num(&self, f: &Feature, key: &str, default: f64) -> Result<f64, FeatureError> {
        expr::eval_field(f.raw.get(key), &self.vars, default).map_err(FeatureError::new)
    }
}

/// Debug-trace sink for the per-feature spans (`"F2 extrude → 6 faces"`,
/// `"face names"`, ...). Ids are the sink's own.
pub trait Tracer {
    fn begin(&mut self, label: &str) -> usize;
    fn relabel(&mut self, id: usize, label: &str);
    fn end(&mut self, id: usize);
}

/// Geometry hook: executes everything the document layer cannot.
pub trait FeatureExec {
    /// Trace sink (None = no per-feature spans).
    fn tracer(&mut self) -> Option<&mut dyn Tracer> {
        None
    }
    /// Live face count of the current body; `None` while there is no body.
    /// Pinned: once a body exists every successful feature's span reads
    /// `"<id> <type> → N faces"` and is followed by a `"face names"` span.
    fn face_count(&self) -> Option<usize> {
        None
    }
    /// Frame of a planar face (`{"kind":"face","id":"F2.C1"}` sketch planes
    /// and plane bases). `None` = unresolvable (`missingReference`).
    fn face_frame(&mut self, _state: &RegenState, _face: &str) -> Option<Frame> {
        None
    }
    /// Called once before the feature loop with the features that will run
    /// (assembly pre-pass, see `features::mate`).
    fn prepare(&mut self, _state: &RegenState, _features: &[&Feature]) {}
    /// Execute a geometry feature (not sketch/sketch3d/plane).
    fn exec(&mut self, state: &mut RegenState, f: &Feature) -> Result<(), FeatureError>;
}

/// Placeholder hook until the geometry crates are wired: geometry features
/// are skipped silently (no body, no error).
pub struct NoGeometry;

impl FeatureExec for NoGeometry {
    fn exec(&mut self, _state: &mut RegenState, _f: &Feature) -> Result<(), FeatureError> {
        Ok(())
    }
}

fn resolve_plane(state: &RegenState, exec: &mut dyn FeatureExec, r: &GeomRef) -> Option<Frame> {
    match r {
        GeomRef::Std(p) => Some(p.frame()),
        GeomRef::Feature(id) => state.plane(id),
        GeomRef::Face(id) => exec.face_frame(state, id),
        _ => None,
    }
}

fn regen_sketch(state: &mut RegenState, exec: &mut dyn FeatureExec, f: &Feature, sk: &Sketch) -> Result<(), FeatureError> {
    let frame = sk
        .plane
        .as_ref()
        .and_then(|p| resolve_plane(state, exec, p))
        .ok_or_else(|| FeatureError::new(codes::MISSING_REFERENCE))?;
    let mut sk = sk.clone();
    let mut changed = false;
    for c in sk.constraints.iter_mut() {
        let Some(e) = &c.expr else { continue };
        let v = expr::eval(e, Some(&state.vars)).map_err(|_| FeatureError::new(codes::BAD_EXPRESSION))?;
        // GUESS: a missing/non-numeric value counts as changed.
        if c.value.map_or(true, |old| (v - old).abs() > 1e-12) {
            c.value = Some(v);
            changed = true;
        }
    }
    if changed {
        let _ = solve_sketch(&mut sk, &SolveOpts::default());
    }
    let profiles = profiles(&sk);
    let _ = f;
    state.result.put_sketch(SketchResult { id: sk.id.clone(), frame, profiles, sketch: sk });
    Ok(())
}

fn regen_plane(state: &mut RegenState, exec: &mut dyn FeatureExec, f: &Feature) -> Result<(), FeatureError> {
    let FeatureKind::RefPlane { base, flip, .. } = &f.kind else { unreachable!() };
    let base = base
        .as_ref()
        .and_then(|b| resolve_plane(state, exec, b))
        .ok_or_else(|| FeatureError::new(codes::MISSING_REFERENCE))?;
    // GUESS: every mode currently behaves as "offset" (a bogus mode does).
    let offset = state.num(f, "offset", 0.0)?;
    let mut fr = base;
    if *flip {
        fr.x = -fr.x;
        fr.z = -fr.z;
    }
    fr.o = fr.o + fr.z.scale(offset);
    state.result.put_plane(&f.id, fr);
    Ok(())
}

/// Regen a sketch3d feature (kernel f1156): evaluate the constraint `expr`s,
/// re-solve when a value changed (failure leaves the points unchanged, no
/// error), and echo the solved sketch.
fn regen_sketch3(state: &mut RegenState, f: &Feature, sk: &crate::sketch3::Sketch3) -> Result<(), FeatureError> {
    let mut sk = sk.clone();
    let mut changed = false;
    for c in sk.constraints.iter_mut() {
        let Some(e) = &c.expr else { continue };
        let v = expr::eval(e, Some(&state.vars)).map_err(|_| FeatureError::new(codes::BAD_EXPRESSION))?;
        if c.value.map_or(true, |old| (v - old).abs() > 1e-12) {
            c.value = Some(v);
            changed = true;
        }
    }
    if changed {
        let _ = crate::sketch3::solver::solve_sketch3(&mut sk, &crate::sketch::solver::SolveOpts::default());
    }
    state.result.put_sketch3(&f.id, sk.to_json());
    Ok(())
}

/// Normalised echo of a sketch3d feature (missing arrays appended).
fn sketch3d_echo(raw: &Json) -> Json {
    let mut o = if let Json::Obj(_) = raw { raw.clone() } else { Json::Obj(vec![]) };
    for k in ["points", "entities", "constraints"] {
        if o.get(k).and_then(|v| v.as_arr()).is_none() {
            crate::sketch::obj_set(&mut o, k, Json::Arr(vec![]));
        }
    }
    o
}

fn run_feature(state: &mut RegenState, exec: &mut dyn FeatureExec, f: &Feature) -> Result<(), FeatureError> {
    if !KNOWN_TYPES.contains(&f.type_name.as_str()) {
        return Err(FeatureError::new(codes::UNKNOWN_FEATURE).with("type", Json::Str(f.type_name.clone())));
    }
    match &f.kind {
        FeatureKind::Sketch(sk) => return regen_sketch(state, exec, f, sk),
        FeatureKind::RefPlane { .. } => return regen_plane(state, exec, f),
        _ => {}
    }
    if let FeatureKind::Sketch3(sk) = &f.kind {
        return regen_sketch3(state, f, sk);
    }
    match f.type_name.as_str() {
        "sketch3d" => {
            state.result.put_sketch3(&f.id, sketch3d_echo(&f.raw));
            return Ok(());
        }
        "insertPart" => {
            if !matches!(&f.kind, FeatureKind::InsertPart { part: Some(_), .. }) {
                return Err(FeatureError::new(codes::INSERT_PART_MISSING));
            }
        }
        t if SKETCH_CONSUMERS.contains(&t) => {
            let ok = f.raw.get("sketch").and_then(|s| s.as_str()).map_or(false, |s| state.sketch(s).is_some());
            if !ok {
                return Err(FeatureError::new(codes::MISSING_SKETCH));
            }
        }
        _ => {}
    }
    exec.exec(state, f)
}

/// Run the regen program.
pub fn regen(part: &Part, opts: &RegenOpts, exec: &mut dyn FeatureExec) -> RegenResult {
    let res = expr::resolve_variables(&part.variables);
    let mut state = RegenState { part, vars: res.values, result: RegenResult::default() };
    let mut n = part.features.len();
    if let Some(u) = opts.up_to {
        n = n.min(u as usize);
    }
    if let Some(r) = part.rollback.filter(|_| !opts.ignore_rollback) {
        n = n.min(r);
    }
    let run: Vec<&Feature> = part.features[..n].iter().filter(|f| !f.suppressed).collect();
    exec.prepare(&state, &run);
    for f in &part.features[..n] {
        if f.suppressed {
            continue;
        }
        let label = format!("{} {}", f.id, f.type_name);
        let span = exec.tracer().map(|t| t.begin(&label));
        let r = run_feature(&mut state, exec, f);
        let faces = exec.face_count();
        if let (Some(id), Some(t)) = (span, exec.tracer()) {
            match &r {
                Ok(()) => {
                    if let Some(n) = faces {
                        t.relabel(id, &format!("{label} → {n} faces"));
                    }
                }
                Err(e) => {
                    let what = e.describe();
                    t.relabel(id, &format!("{label}: {what}"));
                    let c = t.begin(&format!("{} failed: {what}", f.id));
                    t.end(c);
                }
            }
            t.end(id);
            if r.is_ok() && faces.is_some() {
                let c = t.begin("face names");
                t.end(c);
            }
        }
        if let Err(e) = r {
            state.result.put_error(RegenError { feature: f.id.clone(), error: e });
        }
    }
    state.result
}

#[cfg(test)]
mod tests {
    use super::*;

    fn run(src: &str, opts: &str) -> String {
        let part = Part::parse(src).unwrap();
        let o = ck_json::parse(opts).unwrap();
        ck_json::to_string(&regen(&part, &RegenOpts::from_json(Some(&o)), &mut NoGeometry).to_json())
    }

    const EMPTY: &str = r#"{"errors":[],"components":[],"planes":[],"markers":[],"sketches":[],"sketches3":[],"curves":[],"body":null,"surfaces":[]}"#;

    #[test]
    fn empty_part() {
        assert_eq!(run(r#"{"format":"ck-part","features":[]}"#, "{}"), EMPTY);
    }

    /// Oracle replies, verbatim.
    #[test]
    fn oracle_errors() {
        let p = |f: &str| format!(r#"{{"format":"ck-part","features":[{f}]}}"#);
        assert_eq!(
            run(&p(r#"{"type":"bogus","id":"F1"}"#), "{}"),
            r#"{"errors":[["F1",{"code":"unknownFeature","params":{"type":"bogus"}}]],"components":[],"planes":[],"markers":[],"sketches":[],"sketches3":[],"curves":[],"body":null,"surfaces":[]}"#
        );
        assert!(run(&p(r#"{"id":"F1"}"#), "{}").starts_with(r#"{"errors":[["F1",{"code":"unknownFeature","params":{"type":""}}]]"#));
        assert!(run(&p("3"), "{}").starts_with(r#"{"errors":[["",{"code":"unknownFeature","params":{"type":""}}]]"#));
        assert!(run(&p(r#"{"type":"sketch","id":"F1"}"#), "{}").starts_with(r#"{"errors":[["F1",{"code":"missingReference","params":{}}]]"#));
        assert!(run(&p(r#"{"type":"extrude","id":"F1","sketch":"F9"}"#), "{}").starts_with(r#"{"errors":[["F1",{"code":"missingSketch","params":{}}]]"#));
        assert!(run(&p(r#"{"type":"insertPart","id":"F1"}"#), "{}").starts_with(r#"{"errors":[["F1",{"code":"insertPartMissing","params":{}}]]"#));
        assert!(run(&p(r#"{"type":"plane","id":"F1"}"#), "{}").starts_with(r#"{"errors":[["F1",{"code":"missingReference","params":{}}]]"#));
        assert_eq!(run(&p(r#"{"type":"insertPart","id":"F1","part":{"format":"ck-part","features":[]}}"#), "{}"), EMPTY);
        assert_eq!(
            run(&p(r#"{"type":"sketch3d","id":"F1"}"#), "{}"),
            r#"{"errors":[],"components":[],"planes":[],"markers":[],"sketches":[],"sketches3":[["F1",{"sketch":{"type":"sketch3d","id":"F1","points":[],"entities":[],"constraints":[]}}]],"curves":[],"body":null,"surfaces":[]}"#
        );
    }

    /// Oracle: outputs are id-keyed maps (sorted, later id overwrites).
    #[test]
    fn oracle_map_order() {
        let src = r#"{"format":"ck-part","features":[{"type":"sketch","id":"F9","plane":{"kind":"std","id":"front"}},{"type":"sketch","id":"F10","plane":{"kind":"std","id":"front"}},{"type":"sketch3d","id":"F8"},{"type":"sketch3d","id":"F11"},{"type":"bogus","id":"F7"},{"type":"bogus","id":"F12"},{"type":"sketch","id":"F9","plane":{"kind":"std","id":"top"}}]}"#;
        let r = regen(&Part::parse(src).unwrap(), &RegenOpts::default(), &mut NoGeometry);
        let j = r.to_json();
        let ids = |k: &str| -> Vec<String> {
            j.get(k).unwrap().as_arr().unwrap().iter().map(|e| e.as_arr().unwrap()[0].as_str().unwrap().to_string()).collect()
        };
        assert_eq!(ids("sketches"), vec!["F10", "F9"]);
        assert_eq!(ids("sketches3"), vec!["F11", "F8"]);
        assert_eq!(ids("errors"), vec!["F12", "F7"]);
        assert_eq!(r.sketches.iter().find(|s| s.id == "F9").unwrap().frame.z.y, 1.0);
    }

    /// Oracle: plane features and sketches on them.
    #[test]
    fn oracle_planes() {
        let src = r#"{"format":"ck-part","features":[
{"type":"plane","mode":"offset","base":{"kind":"std","id":"top"},"offset":20,"name":"Set screw plane","id":"F12","num":1,"visible":false,"userHidden":true},
{"type":"sketch","id":"F1","plane":{"kind":"feature","id":"F12"},"points":[{"id":"a","x":0,"y":0},{"id":"b","x":1,"y":0}],"entities":[{"type":"line","id":"e1","p1":"a","p2":"b"}],"constraints":[]}]}"#;
        assert_eq!(
            run(src, "{}"),
            r#"{"errors":[],"components":[],"planes":[["F12",{"o":[0,20,0],"x":[1,0,0],"y":[0,0,-1],"z":[0,1,0]}]],"markers":[],"sketches":[["F1",{"frame":{"o":[0,20,0],"x":[1,0,0],"y":[0,0,-1],"z":[0,1,0]},"profiles":{"regions":[],"open":true},"sketch":{"type":"sketch","id":"F1","plane":{"kind":"feature","id":"F12"},"points":[{"id":"a","x":0,"y":0},{"id":"b","x":1,"y":0}],"entities":[{"type":"line","id":"e1","p1":"a","p2":"b"}],"constraints":[]}}]],"sketches3":[],"curves":[],"body":null,"surfaces":[]}"#
        );
        let flipped = src.replace(r#""offset":20,"#, r#""offset":"=5*2","flip":true,"#);
        assert!(run(&flipped, "{}").contains(r#"["F12",{"o":[0,-10,0],"x":[-1,0,0],"y":[0,0,-1],"z":[0,-1,0]}]"#));
        let missing = src.replace(r#""id":"F12"}"#, r#""id":"F99"}"#);
        assert!(run(&missing, "{}").starts_with(r#"{"errors":[["F1",{"code":"missingReference","params":{}}]]"#));
    }

    const RECT: &str = r#"{"format":"ck-part","version":1,VARS"features":[
{"type":"sketch","id":"F1","num":1,"name":"S","plane":{"kind":"std","id":"front"},"seq":10,"visible":true,
"points":[{"id":"p1","x":0,"y":0},{"id":"p2","x":10,"y":1},{"id":"p3","x":11,"y":5},{"id":"p4","x":1,"y":6}],
"entities":[{"type":"line","id":"e1","p1":"p1","p2":"p2"},{"type":"line","id":"e2","p1":"p2","p2":"p3"},{"type":"line","id":"e3","p1":"p3","p2":"p4"},{"type":"line","id":"e4","p1":"p4","p2":"p1"}],
"constraints":[{"type":"horizontal","id":"c1","refs":["e1"]},{"type":"vertical","id":"c2","refs":["e2"]},{"type":"horizontal","id":"c3","refs":["e3"]},{"type":"vertical","id":"c4","refs":["e4"]},{"type":"distance","id":"c5","refs":["p1","p2"],"value":20,"offset":nullEXPR}]},
{"type":"extrude","id":"F2","num":1,"name":"E","op":"add","sketch":"F1","end":"blind","depth":10}]}"#;

    fn rect(vars: &str, expr: &str) -> String {
        RECT.replace("VARS", vars).replace("EXPR", expr)
    }

    /// Oracle: variables + expr re-solve the sketch (points bit-exact).
    #[test]
    fn oracle_expr_resolve() {
        let out = run(&rect(r#""variables":[{"name":"w","expr":"30"}],"#, r#","expr":"w""#), "{}");
        assert!(out.contains(r#""points":[{"id":"p1","x":-9.499999902516906,"y":0.50000009027764},{"id":"p2","x":20.500000097511588,"y":0.50000009027764},{"id":"p3","x":20.500000097513997,"y":5.499999999894265},{"id":"p4","x":-9.499999902526271,"y":5.499999999894264}]"#), "{out}");
        assert!(out.contains(r#""value":30,"offset":null,"expr":"w""#));
        // expr equal to the value: no solve
        let out = run(&rect("", r#","expr":"20""#), "{}");
        assert!(out.contains(r#"{"id":"p2","x":10,"y":1}"#));
        // without expr nothing is solved even if violated
        assert!(run(&rect("", ""), "{}").contains(r#"{"id":"p2","x":10,"y":1}"#));
        // bad expressions
        for (v, e) in [("", r#","expr":"q""#), (r#""variables":[{"name":"w","expr":"w+1"}],"#, r#","expr":"w""#)] {
            // oracle: the dependent extrude then reports missingSketch
            assert!(run(&rect(v, e), "{}").starts_with(r#"{"errors":[["F1",{"code":"badExpression","params":{}}],["F2",{"code":"missingSketch","params":{}}]],"components":[],"planes":[],"markers":[],"sketches":[]"#));
        }
    }

    /// Oracle: upTo / rollback / suppressed.
    #[test]
    fn oracle_up_to_rollback() {
        let base = rect("", "");
        let ids = |s: &str| -> usize { s.matches(r#"["F1",{"frame""#).count() };
        assert_eq!(ids(&run(&base, r#"{"upTo":0}"#)), 0);
        assert_eq!(ids(&run(&base, r#"{"upTo":1}"#)), 1);
        assert_eq!(ids(&run(&base, r#"{"upTo":-1}"#)), 1);
        assert_eq!(ids(&run(&base, r#"{"upTo":"F1"}"#)), 1);
        assert_eq!(ids(&run(&base.replace(r#""version":1,"#, r#""version":1,"rollback":0,"#), "{}")), 0);
        assert_eq!(ids(&run(&base.replace(r#""type":"sketch","id":"F1","#, r#""type":"sketch","suppressed":true,"id":"F1","#), "{}")), 0);
    }

    /// The bracket's std-plane sketches regen exactly like the oracle
    /// (frames, region counts, open flags); F7 is on a face (needs geometry).
    #[test]
    fn bracket_sketches() {
        let path = format!("{}/../../../bracket.ckpart", env!("CARGO_MANIFEST_DIR"));
        let Ok(src) = std::fs::read_to_string(path) else { return };
        let part = Part::parse(&src).unwrap();
        let r = regen(&part, &RegenOpts::default(), &mut NoGeometry);
        let got: Vec<(String, usize, bool)> =
            r.sketches.iter().map(|s| (s.id.clone(), s.profiles.regions.len(), s.profiles.open)).collect();
        assert_eq!(got, vec![("F1".into(), 1, false), ("F3".into(), 5, false), ("F5".into(), 2, false)]);
        assert_eq!(ck_json::to_string(&frame_json(&r.sketches[0].frame)), r#"{"o":[0,0,0],"x":[0,0,-1],"y":[0,1,0],"z":[1,0,0]}"#);
        // without geometry: F7's face plane is unresolved, so F8 loses its sketch
        let errs: Vec<(&str, &str)> = r.errors.iter().map(|e| (e.feature.as_str(), e.error.code.as_str())).collect();
        assert_eq!(errs, vec![("F7", "missingReference"), ("F8", "missingSketch")]);
    }

    /// With the B-rep exec: F2 builds the L body, F7's face plane (F2.C1)
    /// resolves to the oracle frame, the gusset F8 unites onto the body.
    /// (The slow ck-ops cuts F4/F6 are suppressed here; the whole bracket
    /// is pinned by the ignored `brep_golden::parts_match_oracle`.)
    #[test]
    fn bracket_brep() {
        let path = format!("{}/../../../bracket.ckpart", env!("CARGO_MANIFEST_DIR"));
        let Ok(src) = std::fs::read_to_string(path) else { return };
        let mut part = Part::parse(&src).unwrap();
        for f in part.features.iter_mut() {
            if f.id == "F4" || f.id == "F6" {
                f.suppressed = true;
            }
        }
        let mut ex = crate::features::Brep::new();
        let r = regen(&part, &RegenOpts { up_to: Some(8), ignore_rollback: false }, &mut ex);
        let f7 = r.sketches.iter().find(|s| s.id == "F7").unwrap();
        assert_eq!(ck_json::to_string(&frame_json(&f7.frame)), r#"{"o":[26,0,0],"x":[0,0,-1],"y":[0,1,0],"z":[1,0,0]}"#);
        assert!(r.errors.is_empty(), "{:?}", r.errors);
        let b = ex.body.unwrap();
        assert!(b.find_face("F8.C0").is_some() && b.find_face("F8.C1").is_none(), "F8.C1 merges into F2.C1");
    }
}
