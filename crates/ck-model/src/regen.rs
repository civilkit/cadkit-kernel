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
use crate::sketch::profile::{profiles_with, Profiles};
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
    /// `opts.sketchStatus: true`: add a solver `status` to every `sketches`
    /// entry (see `sketch::status`). Off by default: the oracle has no such field.
    pub sketch_status: bool,
    /// `opts.solveSketches: true`: solve every sketch from its supplied
    /// coordinates with all constraints and use the solved coordinates (reply
    /// and downstream profiles). Implies `sketch_status`. Off by default.
    pub solve_sketches: bool,
    /// `opts.featureTimeMs`: wall-clock budget of one feature in ms (`None`: the default,
    /// `ck_core::budget::DEFAULT_FEATURE_MS`; `<= 0`: unlimited). Only enforced where the host installed a clock.
    pub feature_time_ms: Option<f64>,
    /// `opts.regenTimeMs`: wall-clock budget of the whole regen in ms (`None` or `<= 0`: unlimited).
    pub regen_time_ms: Option<f64>,
}

impl RegenOpts {
    /// Parse `msg.opts` (`{"upTo": n}`; negative / non-numbers ignored).
    pub fn from_json(j: Option<&Json>) -> RegenOpts {
        let up_to = j.and_then(|o| o.get("upTo")).and_then(|v| v.as_f64()).filter(|v| *v >= 0.0).map(|v| v as u32);
        let sketch_status = j.and_then(|o| o.get("sketchStatus")).map_or(false, |v| matches!(v, Json::Bool(true)));
        let solve_sketches = j.and_then(|o| o.get("solveSketches")).map_or(false, |v| matches!(v, Json::Bool(true)));
        let ms = |k: &str| j.and_then(|o| o.get(k)).and_then(|v| v.as_f64()).filter(|v| v.is_finite());
        RegenOpts {
            up_to,
            ignore_rollback: false,
            sketch_status: sketch_status || solve_sketches,
            solve_sketches,
            feature_time_ms: ms("featureTimeMs"),
            regen_time_ms: ms("regenTimeMs"),
        }
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
    pub const PLANE_LINE_NORMAL: &str = "planeLineNormal";
    pub const MISSING_SKETCH: &str = "missingSketch";
    pub const BAD_EXPRESSION: &str = "badExpression";
    pub const INSERT_PART_MISSING: &str = "insertPartMissing";
    pub const NO_CLOSED_PROFILE: &str = "noClosedProfile";
    pub const NOTHING_TO_CUT: &str = "nothingToCut";
    pub const INVALID_VALUE: &str = "invalidValue";
    /// One feature used up `opts.featureTimeMs`.
    pub const FEATURE_BUDGET: &str = "featureBudgetExceeded";
    /// The whole regen used up `opts.regenTimeMs`; later features are not run.
    pub const REGEN_BUDGET: &str = "regenBudgetExceeded";
}

/// Feature types the kernel's regen knows (anything else -> `unknownFeature`).
/// Pinned by probing each type with an empty payload; e.g. `convert`,
/// `rebuildFace`, `matchEdges`, `refCurve` are separate messages, not features.
pub const KNOWN_TYPES: &[&str] = &[
    "sketch", "sketch3d", "plane", "axis", "extrude", "revolve", "sphere", "sweep", "loft", "fillet", "chamfer", "shell", "rib",
    "lip", "dome", "splitLine", "draft", "moveFace", "hole", "lpattern", "cpattern", "mirror", "spattern",
    "fillpattern", "curvepattern", "moveBody", "mate", "insertPart", "scale", "surfaceExtrude", "surfaceRevolve",
    "surfacePlanar", "surfaceOffset", "deleteFace", "fill", "knit", "extend", "freeform", "trimSurface", "thicken",
    "cutWithSurface", "pipeCut", "deleteBody", "editCurve", "projectCurve", "helix", "thread", "combine", "bodyBoolean",
];

/// The error the oracle reports for each known type when the payload is
/// empty (no references at all, no body yet) — a checklist for the geometry
/// implementations of [`FeatureExec`]. (`projectCurve`: with no body it is
/// `projectCurveNoFaces`; once a body exists an empty payload is
/// `projectCurveNoInput`, see tests/project_curve.rs.)
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
    ("combine", "nothingToCut"),
    ("bodyBoolean", "nothingToCut"),
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
        // a boolean that failed on two nearly coincident free-form walls: `booleanFailed;nearCoincident=<hostFace>|<toolFace>|<gap>`
        if let Some((c, d)) = code.split_once(";nearCoincident=") {
            let mut it = d.split('|');
            let (h, t, g) = (it.next().unwrap_or(""), it.next().unwrap_or(""), it.next().and_then(|x| x.parse::<f64>().ok()).unwrap_or(0.0));
            return FeatureError { code: c.to_string(), params: Vec::new() }
                .with("hostFace", Json::Str(h.to_string()))
                .with("toolFace", Json::Str(t.to_string()))
                .with("gap", Json::Num(g))
                .with(
                    "hint",
                    Json::Str(format!(
                        "the two faces are nearly coincident (gap {g:.3e} mm) but not the same surface, so the intersection cannot be closed; make them exactly coincident or move the tool by a small amount (0.01), or give the feature a fuzzy `tolerance` (1e-5 to 1e-3 mm) / `autoFuzzy: true`"
                    )),
                );
        }
        // a boolean whose assembled result had open / non-manifold edges: `booleanFailed;edgeFaces=<f1>|<f2>;issue=<kind>;at=<x>|<y>|<z>`
        if let Some((c, d)) = code.split_once(";edgeFaces=") {
            let (faces, rest) = d.split_once(";issue=").unwrap_or((d, ""));
            let (issue, at) = rest.split_once(";at=").unwrap_or((rest, ""));
            let at: Vec<Json> = at.split('|').filter_map(|x| x.parse::<f64>().ok()).map(Json::Num).collect();
            let mut e = FeatureError { code: c.to_string(), params: Vec::new() }
                .with("faces", Json::Arr(faces.split('|').filter(|s| !s.is_empty()).map(|s| Json::Str(s.to_string())).collect()))
                .with("issue", Json::Str(issue.to_string()));
            if at.len() == 3 {
                e = e.with("at", Json::Arr(at));
            }
            return e.with(
                "hint",
                Json::Str(
                    "the intersection of these faces could not be closed into a valid solid (touching tangentially, a rim lying on the other surface, or a sliver thinner than ~1e-3); move the tool by a small amount (0.01), unite the bodies that touch each other first, make the near-coincident faces exactly coincident, or give the feature a fuzzy `tolerance` (1e-5 to 1e-3 mm) / `autoFuzzy: true` so nearly coincident faces are treated as coincident".to_string(),
                ),
            );
        }
        // an exact tangency whose result has an odd Euler characteristic:
        // `booleanTangent;euler=<n>;hint=<text>`
        if let Some((c, d)) = code.split_once(";euler=") {
            let (n, hint) = d.split_once(";hint=").unwrap_or((d, ""));
            return FeatureError { code: c.to_string(), params: Vec::new() }
                .with("euler", Json::Num(n.parse::<f64>().unwrap_or(0.0)))
                .with("hint", Json::Str(hint.to_string()));
        }
        // a boolean that stopped retrying after a long time: `booleanBudgetExceeded;elapsedMs=<n>;budgetMs=<n>`
        if let Some((c, d)) = code.split_once(";elapsedMs=") {
            let (elapsed, budget) = d.split_once(";budgetMs=").unwrap_or((d, "0"));
            return FeatureError { code: c.to_string(), params: Vec::new() }
                .with("budgetMs", Json::Num(budget.parse::<f64>().unwrap_or(0.0)))
                .with("elapsedMs", Json::Num(elapsed.parse::<f64>().unwrap_or(0.0)))
                .with(
                    "hint",
                    Json::Str("the boolean did not assemble and the retries at looser tolerances were not started because it had already run long; the operands probably touch tangentially or nearly coincide over a large area. Move the tool by a small amount (0.01), unite the bodies that touch first, make near-coincident faces exactly coincident, or give the feature a fuzzy `tolerance`".to_string()),
                );
        }
        // a boolean that could not be assembled (no nearer cause diagnosed)
        if code == "booleanFailed" {
            return FeatureError { code: code.to_string(), params: Vec::new() }.with(
                "hint",
                Json::Str(
                    "the operands' intersection could not be closed into a valid solid; the faces usually touch tangentially, coincide over an area, or leave a sliver thinner than ~1e-3 mm. Move the tool by a small amount (0.01), unite bodies that touch each other first, split the tool, or give the feature a fuzzy `tolerance` (1e-5 to 1e-3 mm) / `autoFuzzy: true`".to_string(),
                ),
            );
        }
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
    /// Solver status; only computed when `opts.sketchStatus` is set.
    pub status: Option<crate::sketch::status::SketchStatus>,
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
    /// `thread` features: one entry per thread (id-keyed). Only on the wire when non-empty.
    pub threads: Vec<(String, Json)>,
    /// Non-fatal hints per feature (same shape as `errors`): the document still
    /// regenerates. Only on the wire when non-empty.
    pub warnings: Vec<RegenError>,
    /// Assembly degrees-of-freedom report (docs/spec-agent-interface.md,
    /// "Mate kinds"). Only on the wire when set.
    pub assembly: Option<Json>,
    /// Per-feature size limits, `[id, {"maxHeight": h}]` on the wire: a drafted extrude of one circle
    /// reports the extrude depth at which its taper closes to a point. Only on the wire when non-empty.
    pub limits: Vec<(String, Json)>,
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
    pub fn put_thread(&mut self, id: &str, j: Json) {
        upsert(&mut self.threads, |x| &x.0, (id.to_string(), j));
    }
    pub fn put_sketch3(&mut self, id: &str, j: Json) {
        upsert(&mut self.sketches3, |x| &x.0, (id.to_string(), j));
    }

    pub fn to_json(&self) -> Json {
        let mut out = self.to_json_base();
        if let Json::Obj(m) = &mut out {
            if !self.threads.is_empty() {
                m.push(("threads".into(), Json::Arr(sorted(&self.threads, |t| &t.0).into_iter().map(|t| t.1.clone()).collect())));
            }
            if !self.limits.is_empty() {
                m.push((
                    "limits".into(),
                    Json::Arr(
                        sorted(&self.limits, |t| &t.0)
                            .into_iter()
                            .map(|t| Json::Arr(vec![Json::Str(t.0.clone()), t.1.clone()]))
                            .collect(),
                    ),
                ));
            }
            if let Some(a) = &self.assembly {
                m.push(("assembly".into(), a.clone()));
            }
            if !self.warnings.is_empty() {
                m.push(("warnings".into(), Json::Arr(sorted(&self.warnings, |e| &e.feature).into_iter().map(|e| e.to_json()).collect())));
            }
        }
        out
    }

    fn to_json_base(&self) -> Json {
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
                                ].into_iter().chain(s.status.as_ref().map(|t| ("status".to_string(), t.to_json()))).collect()),
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
    /// `axis` features: id -> (point, unit direction).
    pub axes: Vec<(String, (Vec3, Vec3))>,
    /// `opts.sketchStatus`.
    pub sketch_status: bool,
    /// `opts.solveSketches`.
    pub solve_sketches: bool,
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
    /// Straight edge `(point, unit direction)` of an edge selection
    /// (`{kind:"edge", faces, at}`), for plane axes. `None` = unresolvable.
    fn edge_line(&mut self, _state: &RegenState, _faces: &[String], _at: Option<[f64; 3]>) -> Option<(Vec3, Vec3)> {
        None
    }
    /// Analytic surface of a face (tangent planes, cylindrical axes).
    fn face_surf(&mut self, _state: &RegenState, _face: &str) -> Option<FaceSurf> {
        None
    }
    /// Position of the vertex shared by `faces` nearest `at`.
    fn vertex_point(&mut self, _state: &RegenState, _faces: &[String], _at: Option<Vec3>) -> Option<Vec3> {
        None
    }
    /// The edge between `faces` nearest `at` as a path piece (line or circle arc).
    fn edge_seg(&mut self, _state: &RegenState, _faces: &[String], _at: Option<Vec3>) -> Option<ck_ops::sweep::PathSeg> {
        None
    }
    /// Called once before the feature loop with the features that will run
    /// (assembly pre-pass, see `features::mate`).
    fn prepare(&mut self, _state: &RegenState, _features: &[&Feature]) {}
    /// Execute a geometry feature (not sketch/sketch3d/plane).
    fn exec(&mut self, state: &mut RegenState, f: &Feature) -> Result<(), FeatureError>;
    /// Snapshot of the executor's accumulated state (the regen cache stores one
    /// after every feature). `prev` is the previous snapshot of the same run,
    /// whose unchanged parts the new one shares. `None` = not snapshottable.
    fn save(&self, _prev: Option<&crate::features::BrepSnap>) -> Option<crate::features::BrepSnap> {
        None
    }
    /// Replace the executor's state by a snapshot made by [`FeatureExec::save`].
    fn load(&mut self, _snap: &crate::features::BrepSnap) {}
}

/// Where a cached regen continues: features `..next` already ran and left
/// `result` / `axes` behind (the executor was [`FeatureExec::load`]ed).
pub struct Resume {
    pub next: usize,
    pub result: RegenResult,
    pub axes: Vec<(String, (Vec3, Vec3))>,
}

/// Called by [`regen_from`] after every feature of the run (suppressed ones
/// included, `ran = false`), with the state a continuation would need.
pub trait Checkpoint {
    fn after(&mut self, index: usize, ran: bool, state: &RegenState, exec: &mut dyn FeatureExec);
}

/// Analytic surface summary for [`FeatureExec::face_surf`].
#[derive(Debug, Clone, Copy)]
pub enum FaceSurf {
    Plane(Frame),
    /// `o` on the axis, `a` the unit axis.
    Cylinder { o: Vec3, a: Vec3, r: f64, x: Vec3 },
    /// `x` = the surface frame's x axis (degenerate-direction fallback).
    Sphere { c: Vec3, r: f64, x: Vec3 },
    Other,
}

/// Placeholder hook until the geometry crates are wired: geometry features
/// are skipped silently (no body, no error).
pub struct NoGeometry;

impl FeatureExec for NoGeometry {
    fn exec(&mut self, _state: &mut RegenState, _f: &Feature) -> Result<(), FeatureError> {
        Ok(())
    }
}

pub(crate) fn resolve_plane(state: &RegenState, exec: &mut dyn FeatureExec, r: &GeomRef) -> Option<Frame> {
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
    if state.solve_sketches && !changed {
        // GUI solve: skip when the supplied coordinates already satisfy every
        // constraint; on failure (no convergence / conflict) keep them as sent.
        if let Ok(sys) = crate::sketch::solver::build(&sk) {
            let res = crate::sketch::solver::residuals(&sys, &sys.x);
            let worst = res.iter().fold(0.0f64, |m, v| if v.is_nan() || m.is_nan() { f64::NAN } else { m.max(v.abs()) });
            if !(worst <= crate::sketch::status::SOLVED_TOL) {
                let _ = solve_sketch(&mut sk, &SolveOpts::default());
            }
        }
    }
    let texts = crate::sketch::text::layout_all(&sk);
    if let Some((_, e)) = crate::sketch::text::first_error(&texts) {
        return Err(FeatureError::new(e.code()).with("detail", Json::Str(e.message())));
    }
    for (eid, w) in crate::sketch::text::warnings(&texts) {
        state.result.warnings.push(RegenError {
            feature: f.id.clone(),
            error: FeatureError::new(w.code).with("entity", Json::Str(eid.to_string())).with("detail", Json::Str(w.detail.clone())),
        });
    }
    let profiles = profiles_with(&sk, &texts);
    let status = state.sketch_status.then(|| crate::sketch::status::sketch_status(&sk));
    state.result.put_sketch(SketchResult { id: sk.id.clone(), frame, profiles, sketch: sk, status });
    Ok(())
}

/// Solution of `z1.p = d1, z2.p = d2, x.p = d3` (numerator of Cramer's rule).
fn z_cross_scale(z1: Vec3, z2: Vec3, x: Vec3, d1: f64, d2: f64, d3: f64) -> Vec3 {
    z2.cross(x).scale(d1) + x.cross(z1).scale(d2) + z1.cross(z2).scale(d3)
}

fn rotate_about(v: Vec3, axis: Vec3, ang: f64) -> Vec3 {
    ck_core::linalg::Mat3::rotation(axis, ang).mul_vec(v)
}

fn base_ref(f: &Feature) -> Option<GeomRef> {
    match &f.kind {
        FeatureKind::RefPlane { base, .. } => base.clone(),
        _ => None,
    }
}

fn regen_plane(state: &mut RegenState, exec: &mut dyn FeatureExec, f: &Feature) -> Result<(), FeatureError> {
    let FeatureKind::RefPlane { mode, base, flip, axis, base2, .. } = &f.kind else { unreachable!() };
    use crate::planes as pl;
    // Point / curve / face driven modes (no base plane needed).
    let driven = match mode.as_str() {
        "threePoints" => Some(pl::three_points(state, exec, f, *flip)),
        "linePoint" => Some(pl::line_point(state, exec, f, axis, *flip)),
        "normal" => Some(pl::normal(state, f, *flip)),
        "normalCurve" => Some(pl::normal_curve(state, exec, f, *flip)),
        "curve" => Some(pl::curve_plane(state, exec, f, *flip)),
        "tangent" => Some(pl::tangent(state, exec, f, &base_ref(f), axis, *flip)),
        "parallel" => Some(pl::parallel(state, exec, f, &base_ref(f), *flip)),
        "frame" => Some(pl::frame_plane(f, *flip)),
        _ => None,
    };
    if let Some(r) = driven {
        state.result.put_plane(&f.id, r?);
        return Ok(());
    }
    let base = base
        .as_ref()
        .and_then(|b| resolve_plane(state, exec, b))
        .ok_or_else(|| FeatureError::new(codes::MISSING_REFERENCE))?;
    let missing = || FeatureError::new(codes::MISSING_REFERENCE);
    // Modes pinned against the original: offset, angle, perpendicular, mid
    // (the others live in `planes`). Unknown modes behave as "offset".
    let mut fr = base;
    match mode.as_str() {
        "angle" => {
            // Base frame rotated about the axis line by `angle` (degrees,
            // negated by `flip`); the plane passes through the axis. When
            // the axis lies in the base plane it becomes the new x.
            let (o, d) = axis.as_ref().and_then(|a| crate::planes::resolve_axis(state, exec, a)).ok_or_else(missing)?;
            let mut a = state.num(f, "angle", 0.0)?.to_radians();
            if *flip {
                a = -a;
            }
            let z = rotate_about(base.z, d, a);
            let mut x = if d.dot(base.z).abs() < 1e-9 { d } else { rotate_about(base.x, d, a) };
            // oblique axis: the oracle keeps x on the axis' positive side
            if x.dot(d) < -1e-12 {
                x = -x;
            }
            fr = Frame { o, x, y: z.cross(x), z };
        }
        "perpendicular" => {
            // Plane through the axis line, perpendicular to the base plane:
            // x = axis direction, y = base normal, z = x × y.
            let (o, d) = axis.as_ref().and_then(|a| crate::planes::resolve_axis(state, exec, a)).ok_or_else(missing)?;
            if d.cross(base.z).len() < 1e-9 {
                return Err(FeatureError::new(codes::PLANE_LINE_NORMAL));
            }
            // y = the base normal made perpendicular to the axis
            let y = base.z.reject(d).normalize();
            let (x, z) = if *flip { (-d, -d.cross(y)) } else { (d, d.cross(y)) };
            fr = Frame { o, x, y, z };
        }
        "mid" => {
            // Mid plane of two planes: midpoint of the origins, normal the
            // bisector (parallel planes keep the base frame).
            let b2 = base2.as_ref().and_then(|b| resolve_plane(state, exec, b)).ok_or_else(missing)?;
            let c = base.z.cross(b2.z);
            if c.len() < 1e-9 {
                // parallel: slide the origin along the normal only
                fr.o = base.o + base.z.scale((b2.o - base.o).dot(base.z) * 0.5);
            } else {
                // oracle: z = norm(z1 - z2), x = norm(z1 x z2); the origin is
                // the point of the planes' intersection line nearest the
                // midpoint of the two origins.
                let (z, x) = ((base.z - b2.z).normalize(), c.normalize());
                let m = (base.o + b2.o).scale(0.5);
                let num = z_cross_scale(base.z, b2.z, x, base.z.dot(base.o), b2.z.dot(b2.o), x.dot(m));
                let o = num.scale(1.0 / base.z.dot(b2.z.cross(x)));
                fr = Frame { o, x, y: z.cross(x), z };
            }
        }
        _ => {
            let offset = state.num(f, "offset", 0.0)?;
            if *flip {
                fr.x = -fr.x;
                fr.z = -fr.z;
            }
            fr.o = fr.o + fr.z.scale(offset);
        }
    }
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

/// A feature that failed while a time limit was running out reports the limit, whatever its own code was (the
/// loop that noticed the limit stopped early, so its own failure is a consequence). Adds the numbers and a hint.
fn budget_error(e: FeatureError) -> FeatureError {
    use ck_core::budget::{report, Which};
    let Some((which, limit, elapsed)) = report() else {
        return e;
    };
    let code = match which {
        Which::Regen => codes::REGEN_BUDGET,
        Which::Feature => {
            if e.code.ends_with("BudgetExceeded") {
                e.code.as_str()
            } else {
                codes::FEATURE_BUDGET
            }
        }
    };
    let mut out = FeatureError::new(code);
    if e.code != code && !e.code.ends_with("BudgetExceeded") {
        out = out.with("cause", Json::Str(e.code.clone()));
    }
    let hint = match which {
        Which::Regen => "the whole document used up its time budget (opts.regenTimeMs); this feature was not run. Simplify the model, reduce pattern counts, or raise regenTimeMs",
        Which::Feature => "this feature used up its time budget (opts.featureTimeMs) and was stopped; its result is not applied. Reduce the amount of geometry it works on (pattern count, number of edges or faces), split it into steps, or raise featureTimeMs",
    };
    out.with("budgetMs", Json::Num(limit.round())).with("elapsedMs", Json::Num(elapsed.round())).with("hint", Json::Str(hint.to_string()))
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
        "axis" => return crate::planes::exec_axis(state, exec, f),
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

/// Number of features a regen with these options runs (`upTo` / `rollback`).
pub fn run_len(part: &Part, opts: &RegenOpts) -> usize {
    let mut n = part.features.len();
    if let Some(u) = opts.up_to {
        n = n.min(u as usize);
    }
    if let Some(r) = part.rollback.filter(|_| !opts.ignore_rollback) {
        n = n.min(r);
    }
    n
}

/// Run the regen program.
pub fn regen(part: &Part, opts: &RegenOpts, exec: &mut dyn FeatureExec) -> RegenResult {
    regen_from(part, opts, exec, None, None)
}

/// [`regen`], optionally continuing from a cached state (`resume`: the first
/// `resume.next` features are not run) and reporting a [`Checkpoint`] after
/// every feature.
pub fn regen_from(part: &Part, opts: &RegenOpts, exec: &mut dyn FeatureExec, resume: Option<Resume>, mut ck: Option<&mut dyn Checkpoint>) -> RegenResult {
    let res = expr::resolve_variables(&part.variables);
    let mut state = RegenState { part, vars: res.values, result: RegenResult::default(), axes: vec![], sketch_status: opts.sketch_status, solve_sketches: opts.solve_sketches };
    let mut start = 0;
    if let Some(r) = resume {
        start = r.next;
        state.result = r.result;
        state.axes = r.axes;
    }
    let n = run_len(part, opts);
    let run: Vec<&Feature> = part.features[..n].iter().filter(|f| !f.suppressed).collect();
    exec.prepare(&state, &run);
    let _regen_budget = ck_core::budget::Scope::start(ck_core::budget::Which::Regen, opts.regen_time_ms);
    let feature_ms = match opts.feature_time_ms {
        None => Some(ck_core::budget::DEFAULT_FEATURE_MS),
        Some(ms) if ms > 0.0 => Some(ms),
        Some(_) => None,
    };
    for (i, f) in part.features[..n].iter().enumerate().skip(start) {
        if f.suppressed {
            if let Some(c) = ck.as_deref_mut() {
                c.after(i, false, &state, exec);
            }
            continue;
        }
        let label = format!("{} {}", f.id, f.type_name);
        let span = exec.tracer().map(|t| t.begin(&label));
        let r = {
            let _feature_budget = ck_core::budget::Scope::start(ck_core::budget::Which::Feature, feature_ms);
            // the regen budget ran out in an earlier feature: do not start this one
            let r = if ck_core::budget::expired() { Err(FeatureError::new(codes::REGEN_BUDGET)) } else { run_feature(&mut state, exec, f) };
            r.map_err(budget_error)
        };
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
        if let Some(c) = ck.as_deref_mut() {
            c.after(i, true, &state, exec);
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

    /// `opts.sketchStatus` is additive: without it the reply is unchanged; with it
    /// only a `status` key appears inside each `sketches` entry.
    #[test]
    fn sketch_status_is_opt_in_and_additive() {
        let src = r#"{"format":"ck-part","features":[{"type":"sketch","id":"F1","plane":{"kind":"std","id":"top"},"points":[{"id":"p1","x":1,"y":2}],"entities":[],"constraints":[]}]}"#;
        let plain = run(src, "{}");
        assert!(!plain.contains("\"status\""));
        let with = run(src, r#"{"sketchStatus":true}"#);
        let st = r#","status":{"dof":2,"fullyDefined":false,"conflicts":[],"redundant":[],"solved":true,"state":"under","residual":0}"#;
        assert!(with.contains(st), "{with}");
        assert_eq!(with.replacen(st, "", 1), plain);
        assert_eq!(run(src, r#"{"sketchStatus":1}"#), plain);
    }

    fn solve_reply(points: &str, ents: &str, cons: &str, opts: &str) -> (Vec<(f64, f64)>, Json, usize) {
        let src = format!(r#"{{"format":"ck-part","features":[{{"type":"sketch","id":"F1","plane":{{"kind":"std","id":"top"}},"points":[{points}],"entities":[{ents}],"constraints":[{cons}]}}]}}"#);
        let r = regen(&Part::parse(&src).unwrap(), &RegenOpts::from_json(Some(&ck_json::parse(opts).unwrap())), &mut NoGeometry).to_json();
        let e = &r.get("sketches").unwrap().as_arr().unwrap()[0].as_arr().unwrap()[1];
        let pts = e.get("sketch").unwrap().get("points").unwrap().as_arr().unwrap().iter().map(|p| (p.get("x").unwrap().as_f64().unwrap(), p.get("y").unwrap().as_f64().unwrap())).collect();
        (pts, e.get("status").cloned().unwrap_or(Json::Null), r.get("errors").unwrap().as_arr().unwrap().len())
    }
    const SOLVE: &str = r#"{"solveSketches":true}"#;
    const LINES4: &str = r#"{"type":"line","id":"e1","p1":"p1","p2":"p2"},{"type":"line","id":"e2","p1":"p2","p2":"p3"},{"type":"line","id":"e3","p1":"p3","p2":"p4"},{"type":"line","id":"e4","p1":"p4","p2":"p1"}"#;
    const HV4: &str = r#"{"type":"horizontal","id":"c1","refs":["e1"]},{"type":"vertical","id":"c2","refs":["e2"]},{"type":"horizontal","id":"c3","refs":["e3"]},{"type":"vertical","id":"c4","refs":["e4"]}"#;

    #[test]
    fn solve_sketches_perpendicular_pair() {
        let pts = r#"{"id":"a","x":0,"y":0},{"id":"b","x":10,"y":1},{"id":"c","x":9,"y":8}"#;
        let ents = r#"{"type":"line","id":"e1","p1":"a","p2":"b"},{"type":"line","id":"e2","p1":"b","p2":"c"}"#;
        let cons = r#"{"type":"perpendicular","id":"k1","refs":["e1","e2"]}"#;
        let (off, st, _) = solve_reply(pts, ents, cons, "{}");
        assert_eq!(off[1], (10.0, 1.0));
        assert_eq!(st, Json::Null);
        let (on, st, errs) = solve_reply(pts, ents, cons, SOLVE);
        assert_ne!(on, off);
        let d = ((on[1].0 - on[0].0) * (on[2].0 - on[1].0) + (on[1].1 - on[0].1) * (on[2].1 - on[1].1)).abs();
        assert!(d < 1e-6, "{on:?}");
        assert_eq!(errs, 0);
        assert_eq!(st.get("solved"), Some(&Json::Bool(true)));
    }

    #[test]
    fn solve_sketches_rect_distance_and_pin() {
        let pts = r#"{"id":"p1","x":0,"y":0},{"id":"p2","x":10,"y":1},{"id":"p3","x":11,"y":5},{"id":"p4","x":1,"y":6}"#;
        let cons = format!(r#"{HV4},{{"type":"distance","id":"c5","refs":["p1","p2"],"value":25,"offset":null}}"#);
        let (on, _, _) = solve_reply(pts, LINES4, &cons, SOLVE);
        assert!((on[1].0 - on[0].0 - 25.0).abs() < 1e-6 && (on[1].1 - on[0].1).abs() < 1e-6, "{on:?}");
        // drag: pin p3 -> corners follow
        let cons = format!(r#"{HV4},{{"type":"fix","id":"c9","refs":["p3"],"value":[14,9]}}"#);
        let (on, st, _) = solve_reply(pts, LINES4, &cons, SOLVE);
        assert!((on[2].0 - 14.0).abs() < 1e-6 && (on[2].1 - 9.0).abs() < 1e-6, "{on:?}");
        assert!((on[1].0 - 14.0).abs() < 1e-6 && (on[3].1 - 9.0).abs() < 1e-6, "{on:?}");
        assert_eq!(st.get("solved"), Some(&Json::Bool(true)));
        // free point pin
        let (on, _, _) = solve_reply(r#"{"id":"p","x":1,"y":2}"#, "", r#"{"type":"fix","id":"c1","refs":["p"],"value":[7,-3]}"#, SOLVE);
        assert!((on[0].0 - 7.0).abs() < 1e-9 && (on[0].1 + 3.0).abs() < 1e-9);
    }

    #[test]
    fn solve_sketches_conflict_keeps_coordinates() {
        let pts = r#"{"id":"p1","x":0,"y":0},{"id":"p2","x":10,"y":0},{"id":"p3","x":10,"y":5},{"id":"p4","x":0,"y":5}"#;
        let cons = format!(r#"{HV4},{{"type":"fix","id":"c5","refs":["p1"],"value":[0,0]}},{{"type":"distance","id":"c6","refs":["p1","p2"],"value":10,"offset":null}},{{"type":"distance","id":"c7","refs":["p2","p3"],"value":5,"offset":null}},{{"type":"fix","id":"pin","refs":["p3"],"value":[30,30]}}"#);
        let (on, st, errs) = solve_reply(pts, LINES4, &cons, SOLVE);
        assert_eq!(on, vec![(0.0, 0.0), (10.0, 0.0), (10.0, 5.0), (0.0, 5.0)]);
        assert_eq!(errs, 0);
        assert_eq!(st.get("solved"), Some(&Json::Bool(false)));
        assert!(ck_json::to_string(st.get("conflicts").unwrap()).contains("pin"), "{st:?}");
    }

    #[test]
    fn solve_sketches_off_is_unchanged() {
        let pts = r#"{"id":"p1","x":0,"y":0},{"id":"p2","x":10,"y":1},{"id":"p3","x":11,"y":5},{"id":"p4","x":1,"y":6}"#;
        let (a, _, _) = solve_reply(pts, LINES4, HV4, "{}");
        let (b, _, _) = solve_reply(pts, LINES4, HV4, r#"{"solveSketches":1}"#);
        assert_eq!(a, b);
        assert_eq!(a[1], (10.0, 1.0));
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

    /// Oracle-pinned plane modes: angle (rotation about a world axis, axis
    /// in-plane becomes x), perpendicular, mid; plus their error codes.
    #[test]
    fn plane_modes_match_oracle() {
        let planes = |feats: &str| {
            let src = format!(r#"{{"format":"ck-part","version":1,"name":"t","features":[{feats}]}}"#);
            let r = regen(&Part::parse(&src).unwrap(), &RegenOpts::default(), &mut NoGeometry);
            (r.planes.iter().map(|(i, f)| (i.clone(), *f)).collect::<Vec<_>>(), r.errors.iter().map(|e| e.error.code.clone()).collect::<Vec<_>>())
        };
        let near = |a: Vec3, b: [f64; 3]| (a - Vec3::new(b[0], b[1], b[2])).len() < 1e-9;
        let top = r#""base":{"kind":"std","id":"top"}"#;
        let ax = |a: &str| format!(r#""axis":{{"kind":"axis","axis":"{a}"}}"#);
        let one = |feats: String| planes(&feats).0.pop().unwrap().1;
        let (s, c) = (0.5, 0.75f64.sqrt());
        // angle about x (axis in the base plane: x = axis)
        let f = one(format!(r#"{{"type":"plane","id":"P","mode":"angle",{top},{},"angle":30}}"#, ax("x")));
        assert!(near(f.x, [1., 0., 0.]) && near(f.y, [0., s, -c]) && near(f.z, [0., c, s]), "{f:?}");
        // angle about y (axis along the base normal: x rotates)
        let f = one(format!(r#"{{"type":"plane","id":"P","mode":"angle",{top},{},"angle":30,"flip":true}}"#, ax("y")));
        assert!(near(f.x, [c, 0., s]) && near(f.y, [s, 0., -c]) && near(f.z, [0., 1., 0.]), "{f:?}");
        // angle about z
        let f = one(format!(r#"{{"type":"plane","id":"P","mode":"angle",{top},{},"angle":"=10+20"}}"#, ax("z")));
        assert!(near(f.x, [0., 0., 1.]) && near(f.y, [c, s, 0.]) && near(f.z, [-s, c, 0.]), "{f:?}");
        // the plane passes through the axis, not through the (offset) base
        let f = one(format!(r#"{{"type":"plane","id":"A","mode":"offset",{top},"offset":10}},{{"type":"plane","id":"P","mode":"angle","base":{{"kind":"feature","id":"A"}},{},"angle":30}}"#, ax("x")));
        assert!(near(f.o, [0., 0., 0.]));
        // perpendicular
        let f = one(format!(r#"{{"type":"plane","id":"P","mode":"perpendicular",{top},{}}}"#, ax("z")));
        assert!(near(f.x, [0., 0., 1.]) && near(f.y, [0., 1., 0.]) && near(f.z, [-1., 0., 0.]), "{f:?}");
        let f = one(format!(r#"{{"type":"plane","id":"P","mode":"perpendicular",{top},{},"flip":true}}"#, ax("x")));
        assert!(near(f.x, [-1., 0., 0.]) && near(f.z, [0., 0., -1.]), "{f:?}");
        assert_eq!(planes(&format!(r#"{{"type":"plane","id":"P","mode":"perpendicular",{top},{}}}"#, ax("y"))).1, vec!["planeLineNormal"]);
        // mid: parallel and bisecting
        let f = one(format!(r#"{{"type":"plane","id":"A","mode":"offset",{top},"offset":10}},{{"type":"plane","id":"P","mode":"mid",{top},"base2":{{"kind":"feature","id":"A"}}}}"#));
        assert!(near(f.o, [0., 5., 0.]) && near(f.z, [0., 1., 0.]));
        let f = one(format!(r#"{{"type":"plane","id":"P","mode":"mid",{top},"base2":{{"kind":"std","id":"front"}}}}"#));
        assert!(near(f.x, [1., 0., 0.]) && near(f.y, [0., -0.7071067811865476, -0.7071067811865476]) && near(f.z, [0., 0.7071067811865476, -0.7071067811865476]), "{f:?}");
        let f = one(format!(r#"{{"type":"plane","id":"P","mode":"mid","base":{{"kind":"std","id":"right"}},"base2":{{"kind":"std","id":"top"}}}}"#));
        assert!(near(f.x, [0., 0., 1.]) && near(f.z, [0.7071067811865476, -0.7071067811865476, 0.]), "{f:?}");
        // missing axis / base2
        assert_eq!(planes(&format!(r#"{{"type":"plane","id":"P","mode":"angle",{top},"angle":30}}"#)).1, vec!["missingReference"]);
        assert_eq!(planes(&format!(r#"{{"type":"plane","id":"P","mode":"mid",{top}}}"#)).1, vec!["missingReference"]);
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
        let r = regen(&part, &RegenOpts { up_to: Some(8), ..RegenOpts::default() }, &mut ex);
        let f7 = r.sketches.iter().find(|s| s.id == "F7").unwrap();
        assert_eq!(ck_json::to_string(&frame_json(&f7.frame)), r#"{"o":[26,0,0],"x":[0,0,-1],"y":[0,1,0],"z":[1,0,0]}"#);
        assert!(r.errors.is_empty(), "{:?}", r.errors);
        let b = ex.body.unwrap();
        // F8.C1 merged into F2.C1 during the union; its name survives as an
        // absorbed alias, so a later feature referring to it resolves to the
        // merged face instead of erroring (the Task 5 disjoint-lump regression).
        assert!(b.find_face("F8.C0").is_some(), "F8.C0 stays its own face");
        let merged = b.find_face("F8.C1").expect("F8.C1 resolves through absorbed");
        assert_eq!(Some(merged), b.find_face("F2.C1"), "F8.C1 resolves to the face it merged into");
    }
}
