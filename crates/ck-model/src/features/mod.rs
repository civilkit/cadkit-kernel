//! Feature definitions — the regen program of a part.
//!
//! A feature's payload is exactly its `.ckpart` object minus `type`/`id`/`num`
//! (those live in the `crate::Feature` envelope). Discriminator is lowercase
//! `type`. Each module implements the kernel-side application for one family;
//! file layout mirrors `rs_paths.txt`.
//!
//! Confirmed payload fields (from .ckpart samples + app.pretty.js):
//!
//! | type            | fields |
//! |-----------------|--------|
//! | sketch          | plane, points, entities, constraints, seq, visible |
//! | extrude         | op, sketch, end, depth, flip, draft, draftOut |
//! | revolve         | op, sketch, axis, end, angle, flip |
//! | sweep           | op, sketch, path (+guides/surface in UI) |
//! | loft            | op, sketches, surface, centreLine/guides |
//! | fillet          | radius, edges |
//! | chamfer         | chamferType, distance, (angle / distance2), edges |
//! | draft           | neutral, faces, angle, flip |
//! | shell           | faces, thickness |
//! | hole            | sketch, standard, size, fit, kind, end, depth, tip, flip |
//! | helix           | sketch, definedBy, pitch, revolutions, height, startAngle, leftHand, reverse |
//! | rib             | sketch, thickness |
//! | lip             | kind, faces, edges, width, height |
//! | lpattern        | seeds, dir, spacing, count, reverse |
//! | cpattern        | seeds, axis, count, angle, equal, reverse |
//! | mirror          | seeds, plane, body |
//! | trimSurface     | surface, toolSurface, mutual, flip |
//! | surfacePlanar   | sketch |
//! | surfaceRevolve  | sketch, axis, end, angle |
//! | knit            | surfaces, solid |
//! | fill            | edges, merge, continuity |
//! | insertPart      | part, place/source, fixed, dx/dy/dz, axis, angle |
//! | mate            | kind, a, b, distance, flip |
//! | refPlane        | mode, base, offset, flip |
//! | deleteBody      | bodies |

pub mod blends;
pub mod convert;
pub mod curve_plane;
pub mod curve_ref;
pub mod delete_body;
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

use crate::{Expr, Part, Plane, Ref, Sketch};

/// Envelope shared by all features.
#[derive(Debug, Clone)]
pub struct FeatureDef {
    pub id: String,
    /// Display ordinal.
    pub num: u32,
    pub name: String,
    pub kind: FeatureKind,
}

/// Every feature kind in the part-document vocabulary.
#[derive(Debug, Clone)]
pub enum FeatureKind {
    Sketch(Sketch),
    Extrude { op: Op, sketch: Ref, end: End, depth: Expr, flip: bool, draft: Option<Expr>, draft_out: bool },
    Revolve { op: Op, sketch: Ref, axis: Ref, end: End, angle: Expr, flip: bool },
    Sweep { op: Op, sketch: Ref, path: Ref, guides: Vec<Ref>, surface: bool },
    Loft { op: Op, sketches: Vec<Ref>, centre_line: Option<Ref>, guides: Vec<Ref>, surface: bool },
    Fillet { radius: Expr, edges: Vec<Ref> },
    Chamfer { chamfer_type: ChamferType, distance: Expr, angle: Option<Expr>, distance2: Option<Expr>, edges: Vec<Ref> },
    Draft { neutral: Ref, faces: Vec<Ref>, angle: Expr, flip: bool },
    Shell { faces: Vec<Ref>, thickness: Expr },
    Hole { sketch: Ref, standard: Std, size: String, fit: HoleFit, kind: HoleKind, end: End, depth: Option<Expr>, tip: Option<Expr>, flip: bool },
    Helix { sketch: Ref, defined_by: DefinedBy, pitch: Expr, revolutions: Expr, height: Expr, start_angle: Expr, left_hand: bool, reverse: bool },
    Rib { sketch: Ref, thickness: Expr },
    Lip { kind: LipKind, faces: Vec<Ref>, edges: Vec<Ref>, width: Expr, height: Expr },
    LPattern { seeds: Vec<Ref>, dir: Ref, spacing: Expr, count: u32, reverse: bool },
    CPattern { seeds: Vec<Ref>, axis: Ref, count: u32, angle: Expr, equal: bool, reverse: bool },
    Mirror { seeds: Vec<Ref>, plane: Ref, body: Ref },
    TrimSurface { surface: Ref, tool_surface: Ref, mutual: bool, flip: bool },
    SurfacePlanar { sketch: Ref },
    SurfaceRevolve { sketch: Ref, axis: Ref, end: End, angle: Expr },
    Knit { surfaces: Vec<Ref>, solid: bool },
    Fill { edges: Vec<Ref>, merge: bool, continuity: Continuity },
    InsertPart { part: Box<Part>, place: Placement, fixed: bool },
    Mate { kind: MateKind, a: Ref, b: Ref, distance: Option<Expr>, flip: bool },
    RefPlane { plane: Plane },
    DeleteBody { bodies: Vec<Ref> },
    /// editCurve, projectCurve, curvePlane, curveRef, freeform, convert,
    /// surfaces, surfaceExtrude, pipecut, dome, …
    Other(String),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Op {
    Add,
    Cut,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum End {
    Blind,
    Through,
    ThroughBoth,
    UpToNext,
    Mid,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ChamferType {
    Equal,
    AngleDistance,
    DistanceDistance,
    Vertex,
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
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MateKind {
    Coincident,
    Concentric,
    Distance,
    Angle,
    Parallel,
}

#[derive(Debug, Clone, Default)]
pub struct Placement {
    /// translation
    pub dx: f64,
    pub dy: f64,
    pub dz: f64,
    /// rotation
    pub axis: Option<Ref>,
    pub angle: Option<Expr>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Std {
    Iso,
}
