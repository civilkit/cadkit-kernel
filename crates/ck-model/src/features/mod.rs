//! Feature definitions — the regen program of a part.
//!
//! A feature's payload is its `.ckpart` object; the `crate::Feature` envelope
//! holds `type`/`id`/`num`/`name`/`suppressed` and the raw DOM. Each module
//! implements the kernel-side application for one family; file layout
//! mirrors `rs_paths.txt`.
//!
//! Field tables below are pinned by the 4 sample .ckpart files (every field
//! of every feature they contain) plus the client's `newFeature()` defaults
//! table (app.pretty.js ~l.29884). Numeric fields are [`Expr`]s (JSON number
//! or `"=expr"` string). Parsing is lenient: missing fields take the client
//! defaults; types that are not typed here parse as [`FeatureKind::Other`]
//! (the raw DOM is always on the envelope).
//!
//! | type            | fields |
//! |-----------------|--------|
//! | sketch          | plane, points, entities, constraints, seq, visible |
//! | extrude         | op, sketch, end, depth, flip, draft, draftOut |
//! | revolve         | op, sketch, axis ("e14"), end, angle, flip |
//! | sweep           | op, sketch, path ({kind:curve}), surface |
//! | loft            | op, sketches, surface |
//! | fillet          | radius, edges[{faces,at}] |
//! | chamfer         | chamferType, distance, distance2, angle, flip, edges |
//! | draft           | neutral ({kind:face/std}), faces, angle, flip |
//! | shell           | faces, thickness, outward |
//! | hole            | sketch, standard, size, fit, kind, diameter, end, depth, tip, flip, cbore*/csk* |
//! | helix           | sketch, definedBy, pitch, revolutions, height, startAngle, leftHand, reverse, visible |
//! | rib             | sketch, thickness, flip |
//! | lip             | kind ("lip"), faces, edges, width, height |
//! | lpattern        | seeds, dir ({kind:axis}), spacing, count, reverse |
//! | cpattern        | seeds, bodies, axis ({kind:axis}), count, angle, equal, reverse |
//! | mirror          | seeds, plane ({kind:std}), body |
//! | trimSurface     | surface, surfaces, toolSurface, plane, mutual, flip, deleteTool |
//! | surfacePlanar   | sketch |
//! | surfaceRevolve  | sketch, axis, end, angle, flip, visible |
//! | knit            | surfaces, solid |
//! | fill            | edges, merge, continuity, fairing |
//! | insertPart      | part (nested .ckpart), source, place{o,x,y,z} or dx/dy/dz+axis+angle, fixed |
//! | mate            | kind, a, b, distance, flip |
//! | plane           | mode ("offset"/"angle"/"perpendicular"/"mid"; others fall back to offset), base, offset, flip, axis, angle, base2, curves, visible, userHidden |
//! | deleteBody      | mode, bodies |

pub mod blends;
pub mod convert;
pub mod curve_plane;
pub mod curve_ref;
pub mod delete_body;
pub mod draft;
pub mod edit_curve;
pub mod extrude;
pub mod freeform;
pub mod hole;
pub mod lip;
pub mod loft;
pub mod mate;
pub mod pattern;
pub mod pipecut;
pub mod project_curve;
pub mod ref_plane;
pub mod rib;
pub mod shell;
pub mod surface_extrude;
pub mod surfaces;
pub mod sweep;

use crate::sketch::jtruthy;
use crate::{EdgeRef, Expr, GeomRef, Part, Ref, Sketch};
use ck_json::Json;

/// Envelope shared by all features (alias kept for API compatibility).
pub type FeatureDef = crate::Feature;

/// Every typed feature kind in the part-document vocabulary.
#[derive(Debug, Clone)]
pub enum FeatureKind {
    Sketch(Sketch),
    Sketch3(crate::sketch3::Sketch3),
    Extrude { op: Op, sketch: Option<Ref>, end: End, depth: Expr, flip: bool, draft: Option<Expr>, draft_out: bool },
    Revolve { op: Op, sketch: Option<Ref>, axis: Option<GeomRef>, end: End, angle: Expr, flip: bool },
    Sweep { op: Op, sketch: Option<Ref>, path: Option<GeomRef>, surface: bool },
    Loft { op: Op, sketches: Vec<Ref>, surface: bool },
    Fillet { radius: Expr, edges: Vec<EdgeRef> },
    Chamfer {
        chamfer_type: ChamferType,
        distance: Expr,
        distance2: Option<Expr>,
        angle: Option<Expr>,
        flip: bool,
        edges: Vec<EdgeRef>,
    },
    Draft { neutral: Option<GeomRef>, faces: Vec<Ref>, angle: Expr, flip: bool },
    Shell { faces: Vec<Ref>, thickness: Expr, outward: bool },
    Hole {
        sketch: Option<Ref>,
        standard: Option<String>,
        size: Option<String>,
        fit: Option<HoleFit>,
        kind: HoleKind,
        diameter: Option<Expr>,
        end: End,
        depth: Expr,
        tip: bool,
        flip: bool,
        cbore_diameter: Option<Expr>,
        cbore_depth: Option<Expr>,
        csk_diameter: Option<Expr>,
        csk_angle: Option<Expr>,
    },
    Helix {
        sketch: Option<Ref>,
        defined_by: DefinedBy,
        pitch: Expr,
        revolutions: Expr,
        height: Expr,
        start_angle: Expr,
        left_hand: bool,
        reverse: bool,
    },
    Rib { sketch: Option<Ref>, thickness: Expr, flip: bool },
    /// `kind` is `"lip"` in the samples/defaults.
    Lip { kind: String, faces: Vec<Ref>, edges: Vec<EdgeRef>, width: Expr, height: Expr },
    LPattern { seeds: Vec<Ref>, dir: Option<GeomRef>, spacing: Expr, count: Expr, reverse: bool },
    /// `bodies` entries look like `{"face":"..."}` (kept raw).
    CPattern { seeds: Vec<Ref>, bodies: Vec<Json>, axis: Option<GeomRef>, count: Expr, angle: Expr, equal: bool, reverse: bool },
    Mirror { seeds: Vec<Ref>, plane: Option<GeomRef>, body: bool },
    TrimSurface {
        surface: Option<Ref>,
        surfaces: Vec<Ref>,
        tool_surface: Option<Ref>,
        plane: Option<GeomRef>,
        mutual: bool,
        flip: bool,
        delete_tool: bool,
    },
    SurfacePlanar { sketch: Option<Ref> },
    SurfaceRevolve { sketch: Option<Ref>, axis: Option<GeomRef>, end: End, angle: Expr, flip: bool },
    Knit { surfaces: Vec<Ref>, solid: bool },
    Fill { edges: Vec<EdgeRef>, merge: bool, continuity: Continuity, fairing: Option<String> },
    /// `part` is `None` when missing/invalid (regen error `insertPartMissing`).
    InsertPart { part: Option<Box<Part>>, source: Option<String>, place: Placement, fixed: bool },
    Mate { kind: MateKind, a: Option<Ref>, b: Option<Ref>, distance: Expr, flip: bool },
    /// Wire type `"plane"` (reference plane).
    RefPlane {
        mode: String,
        base: Option<GeomRef>,
        offset: Expr,
        flip: bool,
        axis: Option<GeomRef>,
        angle: Expr,
        base2: Option<GeomRef>,
        visible: bool,
    },
    DeleteBody { mode: String, bodies: Vec<Json> },
    /// Any other type (sketch3d, editCurve, projectCurve, freeform, dome,
    /// thicken, moveFace, splitLine, extend, pipeCut, ...): use `Feature::raw`.
    Other(String),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Op {
    Add,
    Cut,
}

/// End conditions (client error text lists: blind, through, throughBoth,
/// mid, upToNext, upToFace, offsetFromFace, upToPoint).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum End {
    Blind,
    Through,
    ThroughBoth,
    UpToNext,
    Mid,
    UpToFace,
    OffsetFromFace,
    UpToPoint,
    /// Unrecognised value (raw string on `Feature::raw`).
    Unknown,
}

impl End {
    pub fn from_wire(s: &str) -> End {
        match s {
            "blind" => End::Blind,
            "through" => End::Through,
            "throughBoth" => End::ThroughBoth,
            "upToNext" => End::UpToNext,
            "mid" => End::Mid,
            "upToFace" => End::UpToFace,
            "offsetFromFace" => End::OffsetFromFace,
            "upToPoint" => End::UpToPoint,
            _ => End::Unknown,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ChamferType {
    Equal,
    AngleDistance,
    DistanceDistance,
    Vertex,
    Unknown,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DefinedBy {
    PitchRevs,
    PitchHeight,
    RevsHeight,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HoleFit {
    Normal,
    Tap,
    Close,
    Loose,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HoleKind {
    Simple,
    Cbore,
    Csink,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LipKind {
    Material,
    Move,
    Offset,
    Linear,
    Turn,
    Tension,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Continuity {
    G0,
    G1,
    G2,
    G3,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MateKind {
    Coincident,
    Concentric,
    Distance,
    Angle,
    Parallel,
    Unknown,
}

/// insertPart placement: either an explicit frame (`place:{o,x,y,z}`) or a
/// translation `dx/dy/dz` plus rotation `axis` (`{kind:axis}`) + `angle` (deg).
#[derive(Debug, Clone, Default)]
pub struct Placement {
    pub dx: f64,
    pub dy: f64,
    pub dz: f64,
    pub axis: Option<GeomRef>,
    pub angle: Option<Expr>,
    pub frame: Option<ck_core::linalg::Frame>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Std {
    Iso,
}

// ---------------------------------------------------------------------------
// parsing helpers

fn s_opt(j: &Json, k: &str) -> Option<String> {
    j.get(k).and_then(|v| v.as_str()).map(|s| s.to_string())
}
fn r_opt(j: &Json, k: &str) -> Option<Ref> {
    s_opt(j, k).map(Ref)
}
fn refs(j: &Json, k: &str) -> Vec<Ref> {
    j.get(k)
        .and_then(|v| v.as_arr())
        .map(|a| a.iter().filter_map(|x| x.as_str().map(|s| Ref(s.to_string()))).collect())
        .unwrap_or_default()
}
fn edges(j: &Json, k: &str) -> Vec<EdgeRef> {
    j.get(k).and_then(|v| v.as_arr()).map(|a| a.iter().filter_map(EdgeRef::from_json).collect()).unwrap_or_default()
}
fn b(j: &Json, k: &str) -> bool {
    jtruthy(j.get(k))
}
fn b_or(j: &Json, k: &str, d: bool) -> bool {
    j.get(k).map_or(d, |v| jtruthy(Some(v)))
}
/// Numeric field; missing/non-number/non-string -> `Num(default)`.
fn e(j: &Json, k: &str, d: f64) -> Expr {
    j.get(k).and_then(Expr::from_json).unwrap_or(Expr::Num(d))
}
fn e_opt(j: &Json, k: &str) -> Option<Expr> {
    j.get(k).and_then(Expr::from_json)
}
fn g(j: &Json, k: &str) -> Option<GeomRef> {
    j.get(k).and_then(GeomRef::from_json)
}
fn op(j: &Json) -> Op {
    if j.get("op").and_then(|v| v.as_str()) == Some("cut") {
        Op::Cut
    } else {
        Op::Add
    }
}
fn end(j: &Json) -> End {
    End::from_wire(j.get("end").and_then(|v| v.as_str()).unwrap_or("blind"))
}
fn arr_raw(j: &Json, k: &str) -> Vec<Json> {
    j.get(k).and_then(|v| v.as_arr()).map(|a| a.to_vec()).unwrap_or_default()
}

fn frame_json(j: &Json) -> Option<ck_core::linalg::Frame> {
    use ck_core::linalg::{Frame, Vec3};
    let v = |k: &str| -> Option<Vec3> {
        let a = j.get(k)?.as_arr()?;
        Some(Vec3::new(a.first()?.as_f64()?, a.get(1)?.as_f64()?, a.get(2)?.as_f64()?))
    };
    Some(Frame { o: v("o")?, x: v("x")?, y: v("y")?, z: v("z")? })
}

impl FeatureKind {
    /// Parse a feature payload by wire `type`.
    pub fn from_json(type_name: &str, j: &Json) -> FeatureKind {
        match type_name {
            "sketch" => FeatureKind::Sketch(Sketch::from_json(j)),
            "sketch3d" => FeatureKind::Sketch3(crate::sketch3::Sketch3::from_json(j)),
            "extrude" => FeatureKind::Extrude {
                op: op(j),
                sketch: r_opt(j, "sketch"),
                end: end(j),
                depth: e(j, "depth", 0.0),
                flip: b(j, "flip"),
                draft: e_opt(j, "draft"),
                draft_out: b(j, "draftOut"),
            },
            "revolve" => FeatureKind::Revolve {
                op: op(j),
                sketch: r_opt(j, "sketch"),
                axis: g(j, "axis"),
                end: end(j),
                angle: e(j, "angle", 360.0),
                flip: b(j, "flip"),
            },
            "sweep" => FeatureKind::Sweep { op: op(j), sketch: r_opt(j, "sketch"), path: g(j, "path"), surface: b(j, "surface") },
            "loft" => FeatureKind::Loft { op: op(j), sketches: refs(j, "sketches"), surface: b(j, "surface") },
            "fillet" => FeatureKind::Fillet { radius: e(j, "radius", 0.0), edges: edges(j, "edges") },
            "chamfer" => FeatureKind::Chamfer {
                chamfer_type: match j.get("chamferType").and_then(|v| v.as_str()).unwrap_or("equal") {
                    "equal" => ChamferType::Equal,
                    "angleDistance" => ChamferType::AngleDistance,
                    "distanceDistance" => ChamferType::DistanceDistance,
                    "vertex" => ChamferType::Vertex,
                    _ => ChamferType::Unknown,
                },
                distance: e(j, "distance", 0.0),
                distance2: e_opt(j, "distance2"),
                angle: e_opt(j, "angle"),
                flip: b(j, "flip"),
                edges: edges(j, "edges"),
            },
            "draft" => FeatureKind::Draft { neutral: g(j, "neutral"), faces: refs(j, "faces"), angle: e(j, "angle", 0.0), flip: b(j, "flip") },
            "shell" => FeatureKind::Shell { faces: refs(j, "faces"), thickness: e(j, "thickness", 0.0), outward: b(j, "outward") },
            "hole" => FeatureKind::Hole {
                sketch: r_opt(j, "sketch"),
                standard: s_opt(j, "standard"),
                size: s_opt(j, "size"),
                fit: s_opt(j, "fit").and_then(|f| match f.as_str() {
                    "normal" => Some(HoleFit::Normal),
                    "tap" => Some(HoleFit::Tap),
                    "close" => Some(HoleFit::Close),
                    "loose" => Some(HoleFit::Loose),
                    _ => None,
                }),
                kind: match j.get("kind").and_then(|v| v.as_str()) {
                    Some("cbore") => HoleKind::Cbore,
                    Some("csink") => HoleKind::Csink,
                    _ => HoleKind::Simple,
                },
                diameter: e_opt(j, "diameter"),
                end: end(j),
                depth: e(j, "depth", 0.0),
                tip: b(j, "tip"),
                flip: b(j, "flip"),
                cbore_diameter: e_opt(j, "cboreDiameter"),
                cbore_depth: e_opt(j, "cboreDepth"),
                csk_diameter: e_opt(j, "cskDiameter"),
                csk_angle: e_opt(j, "cskAngle"),
            },
            "helix" => FeatureKind::Helix {
                sketch: r_opt(j, "sketch"),
                defined_by: match j.get("definedBy").and_then(|v| v.as_str()) {
                    Some("pitchHeight") => DefinedBy::PitchHeight,
                    Some("revsHeight") => DefinedBy::RevsHeight,
                    _ => DefinedBy::PitchRevs,
                },
                pitch: e(j, "pitch", 1.0),
                revolutions: e(j, "revolutions", 10.0),
                height: e(j, "height", 10.0),
                start_angle: e(j, "startAngle", 0.0),
                left_hand: b(j, "leftHand"),
                reverse: b(j, "reverse"),
            },
            "rib" => FeatureKind::Rib { sketch: r_opt(j, "sketch"), thickness: e(j, "thickness", 0.0), flip: b(j, "flip") },
            "lip" => FeatureKind::Lip {
                kind: s_opt(j, "kind").unwrap_or_else(|| "lip".into()),
                faces: refs(j, "faces"),
                edges: edges(j, "edges"),
                width: e(j, "width", 0.0),
                height: e(j, "height", 0.0),
            },
            "lpattern" => FeatureKind::LPattern {
                seeds: refs(j, "seeds"),
                dir: g(j, "dir"),
                spacing: e(j, "spacing", 0.0),
                count: e(j, "count", 0.0),
                reverse: b(j, "reverse"),
            },
            "cpattern" => FeatureKind::CPattern {
                seeds: refs(j, "seeds"),
                bodies: arr_raw(j, "bodies"),
                axis: g(j, "axis"),
                count: e(j, "count", 0.0),
                angle: e(j, "angle", 360.0),
                equal: b_or(j, "equal", true),
                reverse: b(j, "reverse"),
            },
            "mirror" => FeatureKind::Mirror { seeds: refs(j, "seeds"), plane: g(j, "plane"), body: b(j, "body") },
            "trimSurface" => FeatureKind::TrimSurface {
                surface: r_opt(j, "surface"),
                surfaces: refs(j, "surfaces"),
                tool_surface: r_opt(j, "toolSurface"),
                plane: g(j, "plane"),
                mutual: b(j, "mutual"),
                flip: b(j, "flip"),
                delete_tool: b_or(j, "deleteTool", true),
            },
            "surfacePlanar" => FeatureKind::SurfacePlanar { sketch: r_opt(j, "sketch") },
            "surfaceRevolve" => FeatureKind::SurfaceRevolve {
                sketch: r_opt(j, "sketch"),
                axis: g(j, "axis"),
                end: end(j),
                angle: e(j, "angle", 360.0),
                flip: b(j, "flip"),
            },
            "knit" => FeatureKind::Knit { surfaces: refs(j, "surfaces"), solid: b(j, "solid") },
            "fill" => FeatureKind::Fill {
                edges: edges(j, "edges"),
                merge: b(j, "merge"),
                continuity: match j.get("continuity").and_then(|v| v.as_str()) {
                    Some("G1") => Continuity::G1,
                    Some("G2") => Continuity::G2,
                    Some("G3") => Continuity::G3,
                    _ => Continuity::G0,
                },
                fairing: s_opt(j, "fairing"),
            },
            "insertPart" => FeatureKind::InsertPart {
                part: j.get("part").and_then(|p| Part::from_json(p).ok()).map(Box::new),
                source: s_opt(j, "source"),
                place: Placement {
                    dx: j.get("dx").and_then(|v| v.as_f64()).unwrap_or(0.0),
                    dy: j.get("dy").and_then(|v| v.as_f64()).unwrap_or(0.0),
                    dz: j.get("dz").and_then(|v| v.as_f64()).unwrap_or(0.0),
                    axis: g(j, "axis"),
                    angle: e_opt(j, "angle"),
                    frame: j.get("place").and_then(frame_json),
                },
                fixed: b(j, "fixed"),
            },
            "mate" => FeatureKind::Mate {
                kind: match j.get("kind").and_then(|v| v.as_str()).unwrap_or("coincident") {
                    "coincident" => MateKind::Coincident,
                    "concentric" => MateKind::Concentric,
                    "distance" => MateKind::Distance,
                    "angle" => MateKind::Angle,
                    "parallel" => MateKind::Parallel,
                    _ => MateKind::Unknown,
                },
                a: r_opt(j, "a"),
                b: r_opt(j, "b"),
                distance: e(j, "distance", 0.0),
                flip: b(j, "flip"),
            },
            "plane" => FeatureKind::RefPlane {
                mode: s_opt(j, "mode").unwrap_or_else(|| "offset".into()),
                base: g(j, "base"),
                offset: e(j, "offset", 0.0),
                flip: b(j, "flip"),
                axis: g(j, "axis"),
                angle: e(j, "angle", 0.0),
                base2: g(j, "base2"),
                visible: b_or(j, "visible", true),
            },
            "deleteBody" => FeatureKind::DeleteBody { mode: s_opt(j, "mode").unwrap_or_else(|| "delete".into()), bodies: arr_raw(j, "bodies") },
            other => FeatureKind::Other(other.to_string()),
        }
    }

    /// The sketch a sketch-driven feature consumes (`sketch` field).
    pub fn sketch_ref(&self) -> Option<&Ref> {
        match self {
            FeatureKind::Extrude { sketch, .. }
            | FeatureKind::Revolve { sketch, .. }
            | FeatureKind::Sweep { sketch, .. }
            | FeatureKind::Hole { sketch, .. }
            | FeatureKind::Helix { sketch, .. }
            | FeatureKind::Rib { sketch, .. }
            | FeatureKind::SurfacePlanar { sketch }
            | FeatureKind::SurfaceRevolve { sketch, .. } => sketch.as_ref(),
            _ => None,
        }
    }
}

// ---------------------------------------------------------------- B-rep exec

/// Modelling tolerance handed to ck-ops. GUESS: 1e-6 (ck-ops tests).
pub const TOL: f64 = 1e-6;

/// A debug-trace span produced inside a feature (drained into the regen
/// [`crate::regen::Tracer`] by [`Brep::flush_spans`], nested under the
/// feature's own span).
#[derive(Debug, Clone, PartialEq)]
pub struct Span {
    pub label: String,
    pub children: Vec<Span>,
}

impl Span {
    pub fn leaf(label: impl Into<String>) -> Span {
        Span { label: label.into(), children: vec![] }
    }
    fn emit(&self, t: &mut dyn crate::regen::Tracer) {
        let id = t.begin(&self.label);
        for c in &self.children {
            c.emit(t);
        }
        t.end(id);
    }
}

/// Oracle boolean span: `boolean Subtract (8 × 15 faces) → 13 faces` with
/// the stage children (kernel.strings
/// `locateimprintapplyclassifyassemblesimplifycheck`).
pub fn boolean_span(op: ck_ops::boolean::Op, na: usize, nb: usize, nout: usize) -> Span {
    let name = match op {
        ck_ops::boolean::Op::Union => "Unite",
        ck_ops::boolean::Op::Cut => "Subtract",
        ck_ops::boolean::Op::Intersect => "Intersect",
    };
    Span {
        label: format!("boolean {name} ({na} × {nb} faces) → {nout} faces"),
        children: ["locate", "imprint", "apply", "classify", "assemble", "simplify", "check"].iter().map(|s| Span::leaf(*s)).collect(),
    }
}

/// The geometry [`crate::regen::FeatureExec`]: owns the part's body during
/// a regen. Extrude / revolve tools are swept here (ck-topo) and combined
/// with the body by ck-ops booleans; mirror / patterns re-use the seed
/// features' tools ([`Brep::tools`]); fillets go to ck-ops blends.
#[derive(Default)]
pub struct Brep {
    pub body: Option<ck_topo::body::Body>,
    /// Tool of every body-making feature so far (feature id, tool).
    pub tools: Vec<(String, extrude::Tool)>,
    /// Trace spans of the feature being executed (see [`Brep::flush_spans`]).
    pub spans: Vec<Span>,
    /// Assembly components (insertPart / mate / component patterns).
    pub asm: mate::Assembly,
    /// Helix curves by feature id (sweep paths).
    pub helices: Vec<(String, ck_ops::helix::Helix)>,
    /// Surface bodies by feature id (see `surfaces.rs`).
    pub surfs: Vec<(String, ck_topo::body::Body)>,
}

impl Brep {
    pub fn new() -> Brep {
        Brep::default()
    }

    /// Emit (and clear) the spans recorded by the last `exec`.
    pub fn flush_spans(&mut self, t: &mut dyn crate::regen::Tracer) {
        for s in std::mem::take(&mut self.spans) {
            s.emit(t);
        }
    }

    pub fn tool(&self, feature: &str) -> Option<&extrude::Tool> {
        self.tools.iter().rev().find(|(id, _)| id == feature).map(|(_, t)| t)
    }

    /// Combine `tool` with the body: the first add becomes the body, later
    /// ones go through a boolean (pinned: also for disjoint tools, the
    /// result is re-ordered by the boolean).
    pub fn combine(&mut self, tool: &ck_topo::body::Body, combine: extrude::Combine, warn_no_effect: bool) -> Result<(), crate::regen::FeatureError> {
        use crate::regen::FeatureError;
        use ck_ops::boolean::Op as BOp;
        match (&self.body, combine) {
            (None, extrude::Combine::Add) => {
                self.body = Some(tool.clone());
                Ok(())
            }
            (None, extrude::Combine::Cut) => Err(FeatureError::new(crate::regen::codes::NOTHING_TO_CUT)),
            (Some(b), c) => {
                let op = if c == extrude::Combine::Cut { BOp::Cut } else { BOp::Union };
                let r = ck_ops::boolean::boolean(b, tool, op, TOL);
                match r {
                    Ok(out) => {
                        self.spans.push(boolean_span(op, b.num_faces(), tool.num_faces(), out.body.num_faces()));
                        if out.no_effect && op == BOp::Cut && warn_no_effect {
                            // pinned: body unchanged (original edge order)
                            return Err(FeatureError::new(CUT_NO_EFFECT));
                        }
                        if !out.no_effect {
                            let mut nb = out.body;
                            // a re-split piece keeps its old `~k`: re-rank the family
                            renumber_duplicates(&mut nb);
                            self.body = Some(nb);
                        }
                        Ok(())
                    }
                    Err(e) => {
                        // GUESS: failed boolean span label keeps the input counts
                        self.spans.push(boolean_span(op, b.num_faces(), tool.num_faces(), b.num_faces()));
                        Err(FeatureError::new(&e))
                    }
                }
            }
        }
    }

    /// `upToNext` (oracle probes): add -> `tool − body` (span `boolean
    /// Subtract (tool × body faces)`), cut -> `tool ∩ body` (`boolean
    /// Intersect`); the pieces touching the sketch plane become the tool,
    /// none -> `upToNextNone`. The tool is a `through` sweep.
    /// GUESS: "touching" = has a vertex on the sketch plane; for an add the
    /// piece must also be bounded by the body (a tool that misses -> none).
    fn up_to_next(&mut self, mut tool: extrude::Tool) -> Result<extrude::Tool, crate::regen::FeatureError> {
        use crate::regen::FeatureError;
        use ck_ops::boolean::Op as BOp;
        let body = self.body.as_ref().expect("upToNext with a body");
        let Some(extrude::Recipe::Extrude { profile, .. }) = &tool.recipe else { unreachable!() };
        let (o, n) = (profile.frame.o, profile.frame.z);
        let op = if tool.combine == extrude::Combine::Cut { BOp::Intersect } else { BOp::Cut };
        let none = || FeatureError::new(extrude::UP_TO_NEXT_NONE);
        let r = ck_ops::boolean::boolean(&tool.body, body, op, TOL).map_err(|e| FeatureError::new(&e))?;
        self.spans.push(boolean_span(op, tool.body.num_faces(), body.num_faces(), r.body.num_faces()));
        // pinned: an add whose tool runs off without meeting the body
        // (missing it, or flipped away) -> upToNextOpen
        let open = || FeatureError::new(extrude::UP_TO_NEXT_OPEN);
        if op == BOp::Cut && r.no_effect {
            return Err(open());
        }
        let b = &r.body;
        let on_plane = |f: ck_topo::body::FaceId| {
            b.face_edges(f).into_iter().any(|e| {
                let ed = b.edge(e);
                [ed.v0, ed.v1].iter().any(|&v| (b.vertex(v).p - o).dot(n).abs() < 1e-6)
            })
        };
        let keep: Vec<ck_topo::body::FaceId> =
            ck_topo::region::components(b).into_iter().filter(|c| c.iter().any(|&f| on_plane(f))).flatten().collect();
        if keep.is_empty() {
            return Err(none());
        }
        if op == BOp::Cut && keep.len() == b.num_faces() && ck_topo::region::components(b).len() == 1 {
            // GUESS: the whole tool survived -> nothing bounds it
            return Err(open());
        }
        let mut order: Vec<ck_topo::body::FaceId> = b.face_ids().collect();
        order.retain(|f| keep.contains(f));
        let mut piece = ck_ops::boolean::assemble::rebuild_in_order(b, &order);
        // The `~k` split suffixes belong to the intermediate boolean; a
        // face left whole in the kept piece gets its tool name back
        // (pinned: `F4.S.e1`, not `F4.S.e1~1`).
        let orig: Vec<String> = tool.body.face_ids().map(|f| tool.body.face_id_str(f)).collect();
        let base = |n: &ck_topo::body::FaceName| {
            let mut m = n.clone();
            m.region = None;
            m
        };
        let names: Vec<ck_topo::body::FaceName> = piece.face_ids().map(|f| piece.face(f).name.clone()).collect();
        for f in piece.faces.iter_mut().filter(|f| f.alive) {
            let id = f.name.id();
            let b0 = base(&f.name);
            if f.name.region.is_some() && !orig.contains(&id) && orig.contains(&b0.id()) && names.iter().filter(|n| base(n) == b0).count() == 1 {
                f.name = b0;
            }
        }
        tool.body = piece;
        Ok(tool)
    }

    fn apply(&mut self, id: &str, tool: extrude::Tool) -> Result<(), crate::regen::FeatureError> {
        let r = self.combine(&tool.body, tool.combine, true);
        // GUESS: the tool is remembered even when the combine fails.
        self.tools.push((id.to_string(), tool));
        r
    }
}

/// Feature warning: a cut that removes nothing (kernel.strings).
pub const CUT_NO_EFFECT: &str = "cutNoEffect";

/// Chord-height of the stand-in edge polylines relative to the body
/// diagonal (ck-topo README: oracle counts fit ≈ 3.74e-4 × diagonal).
pub const EDGE_SAG: f64 = 3.74e-4;

/// Bounds of vertices, edges and a uv grid over each face's pcurve box
/// (ck-topo's `bbox` only sees edges, which misses e.g. a revolved sphere).
pub fn body_box(b: &ck_topo::body::Body) -> (ck_core::linalg::Vec3, ck_core::linalg::Vec3) {
    use ck_geom::Vec2;
    let (mut lo, mut hi) = b.bbox();
    for f in b.face_ids() {
        let face = b.face(f);
        if matches!(face.surface, ck_geom::surface::Surface::Plane { .. }) {
            continue;
        }
        let (mut a, mut z) = (Vec2::new(f64::INFINITY, f64::INFINITY), Vec2::new(f64::NEG_INFINITY, f64::NEG_INFINITY));
        for &l in &face.loops {
            for &c in &b.lp(l).coedges {
                let (t0, t1) = b.coedge_params(c);
                for k in 0..=8 {
                    let uv = b.coedge_uv(c, t0 + (t1 - t0) * k as f64 / 8.0, None);
                    a = Vec2::new(a.x.min(uv.x), a.y.min(uv.y));
                    z = Vec2::new(z.x.max(uv.x), z.y.max(uv.y));
                }
            }
        }
        if !(a.x.is_finite() && z.x.is_finite()) {
            continue;
        }
        let n = 32;
        for i in 0..=n {
            for j in 0..=n {
                let (u, v) = (a.x + (z.x - a.x) * i as f64 / n as f64, a.y + (z.y - a.y) * j as f64 / n as f64);
                let p = face.surface.eval(u, v);
                lo = lo.min(p);
                hi = hi.max(p);
            }
        }
    }
    // Oracle (drafted box): the wire box is the union of the exact bounds
    // and the f32 display-mesh positions, e.g. min.z = f32(-20.87488663516974)
    // = -20.874887466430664 while min.x keeps the exact f64.
    let f = |x: f64| x as f32 as f64;
    let r = |p: ck_core::linalg::Vec3| ck_core::linalg::Vec3::new(f(p.x), f(p.y), f(p.z));
    (lo.min(r(lo)), hi.max(r(hi)))
}

/// `body.faces[]`: ck-topo's export with the planar `normal` re-normalised
/// the kernel's way (`sqrt(z² + (x² + y²))`, times `1/len`). Pinned by a
/// component pattern probe: a copy rotated 120° about x has frame z
/// `(0, −0.4999999999999999, 0.8660254037844389)` (dumped) and wire normal
/// `(0, 0.4999999999999998, −0.8660254037844387)` (sense flipped).
/// GUESS: applied only to `renorm` faces (assembly components, whose frames
/// went through the kernel's frame transform) — ck-topo's own frames already
/// print like the oracle (revolved cone cap) and would change.
pub fn faces_json(b: &ck_topo::body::Body, renorm: &dyn Fn(ck_topo::body::FaceId) -> bool) -> Json {
    let mut j = b.faces_json();
    if let Json::Arr(fs) = &mut j {
        for (f, o) in b.face_ids().zip(fs.iter_mut()) {
            if !renorm(f) {
                continue;
            }
            let Json::Obj(kv) = o else { continue };
            if let Some(slot) = kv.iter_mut().find(|(k, _)| k == "normal") {
                // pinned: the kernel prints normalize(x × y) of the frame
                let n = match &b.face(f).surface {
                    ck_geom::surface::Surface::Plane { frame } => mate::plane_normal(frame, b.face(f).sense),
                    _ => mate::knorm(b.planar_normal(f)),
                };
                slot.1 = Json::Arr(vec![Json::Num(n.x), Json::Num(n.y), Json::Num(n.z)]);
            }
        }
    }
    j
}

/// Kernel re-normalisation of a wire vector (see [`faces_json`]).
pub fn renorm(n: ck_core::linalg::Vec3) -> Option<Json> {
    let l = (n.z * n.z + (n.x * n.x + n.y * n.y)).sqrt();
    if !(l > 1e-300 && l.is_finite()) {
        return None;
    }
    let i = 1.0 / l;
    Some(Json::Arr(vec![Json::Num(n.x * i), Json::Num(n.y * i), Json::Num(n.z * i)]))
}

/// Re-normalise `circle.n` of the wire edges selected by `pick` (same rule
/// as the face normals: component pattern probe, circle normal
/// `(0, −0.4999999999999998, 0.8660254037844387)` on the oracle wire).
pub fn renorm_edges(b: &ck_topo::body::Body, edges: &mut Json, pick: &dyn Fn(ck_topo::body::EdgeId) -> bool) {
    let Json::Arr(es) = edges else { return };
    for (e, o) in b.wire_edges().into_iter().zip(es.iter_mut()) {
        if !pick(e) {
            continue;
        }
        let ck_geom::curve::Curve::Circle { frame, .. } = &b.edge(e).curve else { continue };
        let n = mate::plane_normal(frame, true);
        if let Json::Obj(kv) = o {
            if let Some((_, Json::Obj(c))) = kv.iter_mut().find(|(k, _)| k == "circle") {
                if let Some(slot) = c.iter_mut().find(|(k, _)| k == "n") {
                    slot.1 = Json::Arr(vec![Json::Num(n.x), Json::Num(n.y), Json::Num(n.z)]);
                }
            }
        }
    }
}

/// The geometry part of the wire `body`: `box`, `faces`, `edges` (in that
/// key order; the server appends `silhouette` and `mesh`). Edge `pts` use
/// ck-topo's `sample_edge` stand-in until ck-tess provides its own.
pub fn body_wire(b: &ck_topo::body::Body) -> Vec<(String, Json)> {
    body_wire_with(b, &|_| false)
}

/// [`body_wire`] with the faces whose normals are re-normalised.
pub fn body_wire_with(b: &ck_topo::body::Body, renorm: &dyn Fn(ck_topo::body::FaceId) -> bool) -> Vec<(String, Json)> {
    let (lo, hi) = body_box(b);
    let v = |p: ck_core::linalg::Vec3| Json::Arr(vec![Json::Num(p.x), Json::Num(p.y), Json::Num(p.z)]);
    // GUESS: the oracle box comes from the display mesh (curved bodies show
    // f32 mesh coordinates); exact bounds match it for planar bodies.
    let bx = Json::Obj(vec![("min".into(), v(lo)), ("max".into(), v(hi))]);
    let tol = EDGE_SAG * lo.dist(hi);
    vec![
        ("box".into(), bx),
        ("faces".into(), faces_json(b, renorm)),
        ("edges".into(), b.edges_json(&|b, e| b.sample_edge(e, tol))),
    ]
}

impl crate::regen::FeatureExec for Brep {
    fn prepare(&mut self, state: &crate::regen::RegenState, features: &[&crate::Feature]) {
        Brep::prepare(self, state, features)
    }
    fn face_frame(&mut self, _state: &crate::regen::RegenState, face: &str) -> Option<ck_core::linalg::Frame> {
        crate::refs::face_frame(self.body.as_ref()?, face)
    }
    fn face_count(&self) -> Option<usize> {
        self.body.as_ref().map(|b| b.num_faces())
    }
    fn edge_line(&mut self, _state: &crate::regen::RegenState, faces: &[String], at: Option<[f64; 3]>) -> Option<(ck_core::linalg::Vec3, ck_core::linalg::Vec3)> {
        use ck_core::linalg::Vec3;
        let b = self.body.as_ref()?;
        let at = at.map(|a| Vec3::new(a[0], a[1], a[2])).unwrap_or(Vec3::ZERO);
        let e = ck_ops::blend::build::find_edge(b, faces.first()?, faces.get(1)?, at)?;
        let ed = b.edge(e);
        match &ed.curve {
            // origin = the edge's start point (oracle-pinned)
            ck_geom::curve::Curve::Line { o, d } => Some((*o + *d * (b.vertex(ed.v0).p - *o).dot(*d), *d)),
            _ => None,
        }
    }
    fn edge_seg(&mut self, _state: &crate::regen::RegenState, faces: &[String], at: Option<ck_core::linalg::Vec3>) -> Option<ck_ops::sweep::PathSeg> {
        use ck_ops::sweep::PathSeg;
        let b = self.body.as_ref()?;
        let e = ck_ops::blend::build::find_edge(b, faces.first()?, faces.get(1)?, at.unwrap_or(ck_core::linalg::Vec3::ZERO))?;
        let ed = b.edge(e);
        match &ed.curve {
            ck_geom::curve::Curve::Line { .. } => Some(PathSeg::Line { a: b.vertex(ed.v0).p, b: b.vertex(ed.v1).p }),
            ck_geom::curve::Curve::Circle { frame, r } => Some(PathSeg::Arc { frame: *frame, r: *r, a0: ed.t0, a1: ed.t1 }),
            _ => None,
        }
    }
    fn vertex_point(&mut self, _state: &crate::regen::RegenState, faces: &[String], at: Option<ck_core::linalg::Vec3>) -> Option<ck_core::linalg::Vec3> {
        let b = self.body.as_ref()?;
        let ids: Vec<_> = faces.iter().map(|f| b.find_face(f)).collect::<Option<_>>()?;
        let at = at.unwrap_or(ck_core::linalg::Vec3::ZERO);
        let mut best: Option<(f64, ck_core::linalg::Vec3)> = None;
        // vertices that every named face touches
        let mut cands: Vec<ck_topo::body::VertexId> = vec![];
        for (i, f) in ids.iter().enumerate() {
            let mut vs = vec![];
            for e in b.face_edges(*f) {
                vs.push(b.edge(e).v0);
                vs.push(b.edge(e).v1);
            }
            if i == 0 {
                cands = vs;
            } else {
                cands.retain(|v| vs.contains(v));
            }
        }
        for v in cands {
            let p = b.vertex(v).p;
            let d = p.dist(at);
            if best.map_or(true, |(bd, _)| d < bd) {
                best = Some((d, p));
            }
        }
        best.map(|x| x.1)
    }
    fn face_surf(&mut self, _state: &crate::regen::RegenState, face: &str) -> Option<crate::regen::FaceSurf> {
        use crate::regen::FaceSurf;
        use ck_geom::surface::Surface;
        let b = self.body.as_ref()?;
        let f = b.find_face(face)?;
        Some(match &b.face(f).surface {
            Surface::Plane { frame } => FaceSurf::Plane(*frame),
            Surface::Cylinder { frame, r } => FaceSurf::Cylinder { o: frame.o, a: frame.z, r: *r, x: frame.x },
            Surface::Sphere { frame, r } => FaceSurf::Sphere { c: frame.o, r: *r, x: frame.x },
            _ => FaceSurf::Other,
        })
    }
    fn exec(&mut self, state: &mut crate::regen::RegenState, f: &crate::Feature) -> Result<(), crate::regen::FeatureError> {
        let tool = match &f.kind {
            FeatureKind::Extrude { end: End::UpToNext, .. } if self.body.is_some() => {
                let t = extrude::exec_extrude(state, f, self.body.as_ref())?;
                self.up_to_next(t)?
            }
            FeatureKind::Extrude { .. } => extrude::exec_extrude(state, f, self.body.as_ref())?,
            FeatureKind::Revolve { .. } => extrude::exec_revolve(state, f, self.body.is_some())?,
            FeatureKind::Mirror { .. } => return pattern::exec_mirror(self, state, f),
            FeatureKind::CPattern { seeds, bodies, .. } if seeds.is_empty() && !bodies.is_empty() && !self.asm.components.is_empty() => {
                return self.exec_component_pattern(state, f)
            }
            FeatureKind::LPattern { .. } | FeatureKind::CPattern { .. } => return pattern::exec_pattern(self, state, f),
            FeatureKind::Fillet { .. } => return blends::exec_fillet(self, state, f),
            FeatureKind::Chamfer { .. } => return blends::exec_chamfer(self, state, f),
            FeatureKind::Shell { .. } => return shell::exec_shell(self, state, f),
            FeatureKind::InsertPart { .. } => return self.exec_insert(state, f),
            FeatureKind::Hole { .. } => return hole::exec_hole(self, state, f),
            FeatureKind::Sweep { .. } => sweep::exec_sweep(self, state, f)?,
            FeatureKind::Helix { .. } => return sweep::exec_helix(self, state, f),
            FeatureKind::Mate { .. } => return mate::exec_mate(self, state, f),
            _ if surfaces::handles(f) => return surfaces::dispatch(self, state, f),
            FeatureKind::Rib { .. } => return rib::exec_rib(self, state, f),
            FeatureKind::Lip { .. } => return lip::exec_lip(self, state, f),
            FeatureKind::Draft { .. } => return draft::exec_draft(self, state, f),
            FeatureKind::Loft { .. } => return loft::exec_loft(self, state, f),
            _ => return Err(crate::regen::FeatureError::new(&extrude::todo(&f.type_name))),
        };
        self.apply(&f.id, tool)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Part;

    const SAMPLES: [&str; 4] = ["bracket", "coupling-sample", "stepper-sample", "large-sample"];

    fn load(name: &str) -> Option<Part> {
        let p = format!("{}/../../../{}.ckpart", env!("CARGO_MANIFEST_DIR"), name);
        let src = std::fs::read_to_string(p).ok()?;
        Some(Part::parse(&src).expect("sample parses"))
    }

    fn walk(p: &Part, f: &mut dyn FnMut(&crate::Feature)) {
        for x in &p.features {
            f(x);
            if let FeatureKind::InsertPart { part: Some(sub), .. } = &x.kind {
                walk(sub, f);
            }
        }
    }

    /// Every feature in every sample parses to a typed kind (no `Other`), and
    /// the document round-trips through `to_json` unchanged.
    #[test]
    fn samples_parse_fully() {
        for name in SAMPLES {
            let Some(p) = load(name) else { continue };
            let mut n = 0;
            walk(&p, &mut |f| {
                n += 1;
                assert!(!matches!(f.kind, FeatureKind::Other(_)), "{name}: {} {}", f.id, f.type_name);
                if let FeatureKind::InsertPart { part, .. } = &f.kind {
                    assert!(part.is_some(), "{name}: {} nested part", f.id);
                }
            });
            assert!(n > 5, "{name}");
            assert_eq!(ck_json::to_string(&p.to_json()), ck_json::to_string(&p.raw), "{name} round-trip");
        }
    }

    #[test]
    fn bracket_fields() {
        let Some(p) = load("bracket") else { return };
        assert_eq!(p.name, "NEMA 17 motor bracket");
        assert_eq!(p.seq, 13);
        assert_eq!(p.rollback, None);
        assert_eq!(p.material, "aluminium");
        assert_eq!(p.counters[0], ("sketch".to_string(), 4));
        let f2 = p.feature("F2").unwrap();
        match &f2.kind {
            FeatureKind::Extrude { op, sketch, end, depth, .. } => {
                assert_eq!(*op, Op::Add);
                assert_eq!(sketch.as_ref().unwrap().as_str(), "F1");
                assert_eq!(*end, End::Mid);
                assert_eq!(depth.as_num(), Some(52.0));
            }
            k => panic!("{k:?}"),
        }
        match &p.feature("F11").unwrap().kind {
            FeatureKind::Fillet { radius, edges } => {
                assert_eq!(radius.as_num(), Some(5.0));
                assert_eq!(edges.len(), 4);
                assert_eq!(edges[0].faces, vec!["F2.C0", "F2.S.e10"]);
                assert_eq!(edges[0].at, Some([-26.0, 58.0, 2.0]));
            }
            k => panic!("{k:?}"),
        }
        let s7 = p.feature("F7").unwrap().as_sketch().unwrap();
        assert_eq!(s7.plane, Some(GeomRef::Face("F2.C1".into())));
        match &p.feature("F10").unwrap().kind {
            FeatureKind::Mirror { plane, body, seeds } => {
                assert_eq!(*plane, Some(GeomRef::Std(crate::StdPlane::Right)));
                assert!(!body);
                assert_eq!(seeds[0].as_str(), "F8");
            }
            k => panic!("{k:?}"),
        }
    }

    #[test]
    fn assembly_fields() {
        let Some(p) = load("coupling-sample") else { return };
        let f2 = p.feature("F2").unwrap();
        match &f2.kind {
            FeatureKind::InsertPart { part, place, source, .. } => {
                let sub = part.as_ref().unwrap();
                assert_eq!(sub.name, "Flange coupling half");
                assert_eq!(place.dx, -40.0);
                assert_eq!(place.axis, Some(GeomRef::Axis("y".into())));
                assert_eq!(place.angle, Some(Expr::Num(150.0)));
                assert_eq!(source.as_deref(), Some("tests/real-parts/coupling-half.ckpart"));
                match &sub.feature("F12").unwrap().kind {
                    FeatureKind::RefPlane { mode, base, offset, visible, .. } => {
                        assert_eq!(mode, "offset");
                        assert_eq!(*base, Some(GeomRef::Std(crate::StdPlane::Top)));
                        assert_eq!(offset.as_num(), Some(20.0));
                        assert!(!visible);
                    }
                    k => panic!("{k:?}"),
                }
                match &sub.feature("F2").unwrap().kind {
                    FeatureKind::Revolve { axis, angle, .. } => {
                        assert_eq!(*axis, Some(GeomRef::Name("e14".into())));
                        assert_eq!(angle.as_num(), Some(360.0));
                    }
                    k => panic!("{k:?}"),
                }
            }
            k => panic!("{k:?}"),
        }
        let Some(n) = load("stepper-sample") else { return };
        assert!(n.is_assembly());
        match &n.feature("F30").unwrap().kind {
            FeatureKind::InsertPart { place, .. } => {
                let fr = place.frame.unwrap();
                assert_eq!(fr.o.x, 0.6);
                assert_eq!(fr.y.x, -1.0);
            }
            k => panic!("{k:?}"),
        }
    }

    #[test]
    fn format_errors() {
        for bad in [
            r#"{}"#,
            r#"{"format":"ck-part"}"#,
            r#"{"features":[]}"#,
            r#"{"format":"x","version":1,"features":[]}"#,
            r#"{"format":"ck-part","features":{}}"#,
            r#"[]"#,
        ] {
            assert_eq!(Part::parse(bad).unwrap_err(), "Format", "{bad}");
        }
        // version is not validated; junk in other fields is tolerated.
        let p = Part::parse(r#"{"format":"ck-part","version":2,"features":[3,{"type":"bogus"}],"name":5,"seq":"x"}"#).unwrap();
        assert_eq!(p.features.len(), 2);
        assert_eq!(p.features[1].type_name, "bogus");
    }
}

/// A face split by the union (the shell floor cut by the rib) keeps its old
/// `~k` on both pieces; pinned: the whole family is re-ranked by control-box
/// centre (`F2.C0~1` outer, `~2`/`~3` the floor pieces).
pub(crate) fn renumber_duplicates(b: &mut ck_topo::body::Body) {
    use std::collections::BTreeMap;
    let mut fam: BTreeMap<(String, String, String), Vec<ck_topo::body::FaceId>> = BTreeMap::new();
    for fid in b.face_ids() {
        let n = &b.face(fid).name;
        let base = ck_topo::body::FaceName { region: None, ..n.clone() };
        fam.entry((base.id(), String::new(), String::new())).or_default().push(fid);
    }
    for (_, fs) in fam {
        let ids: Vec<String> = fs.iter().map(|&x| b.face_id_str(x)).collect();
        let dup = ids.iter().enumerate().any(|(i, a)| ids[..i].contains(a));
        if dup {
            for &x in &fs {
                b.faces[x.idx()].name.region = None;
            }
            b.number_split_faces();
        }
    }
}
