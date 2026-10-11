//! Sketch documents: plane, points, entities, constraints, solver submodules.

pub mod offset;
pub mod profile;
pub mod smooth;
pub mod solver;
pub mod spline;

use crate::{Expr, Plane, Ref};

/// A 2D sketch: plane + points + entities + constraints.
#[derive(Debug, Clone)]
pub struct Sketch {
    pub id: String,
    pub name: String,
    pub plane: Plane,
    pub points: Vec<SketchPoint>,
    pub entities: Vec<SketchEntity>,
    pub constraints: Vec<Constraint>,
    pub seq: u32,
    pub visible: bool,
}

/// A 2D point in a sketch. JSON: `{"id":"p1","x":0,"y":10,"fixed":true}`
#[derive(Debug, Clone, Default)]
pub struct SketchPoint {
    pub id: String,
    pub x: f64,
    pub y: f64,
    /// Omitted unless true.
    pub fixed: bool,
}

/// Sketch curve entities. JSON: `{"type":"line","id":"e7","p1":"p1","p2":"p2"}`
#[derive(Debug, Clone)]
pub enum SketchEntity {
    Line { id: String, p1: String, p2: String, construction: bool },
    Circle { id: String, c: String, r: f64, construction: bool },
    Arc { id: String, c: String, p1: String, p2: String },
    Spline { id: String, pts: Vec<String> },
}

impl SketchEntity {
    pub fn id(&self) -> &str {
        match self {
            SketchEntity::Line { id, .. }
            | SketchEntity::Circle { id, .. }
            | SketchEntity::Arc { id, .. }
            | SketchEntity::Spline { id, .. } => id,
        }
    }
}

/// Constraint on sketch geometry. Wire: `{"type":"horizontal","id":"c1","refs":[..]}`,
/// dimensional ones add `"value":"20","offset":0`.
#[derive(Debug, Clone)]
pub enum Constraint {
    Horizontal { id: String, refs: Vec<Ref> },
    Vertical { id: String, refs: Vec<Ref> },
    Coincident { id: String, refs: Vec<Ref> },
    Concentric { id: String, refs: Vec<Ref> },
    Collinear { id: String, refs: Vec<Ref> },
    Midpoint { id: String, refs: Vec<Ref> },
    Perpendicular { id: String, refs: Vec<Ref> },
    Tangent { id: String, refs: Vec<Ref> },
    Equal { id: String, refs: Vec<Ref> },
    Symmetric { id: String, refs: Vec<Ref> },
    PointOnCircle { id: String, refs: Vec<Ref> },
    Angle { id: String, refs: Vec<Ref>, value: Expr, offset: f64 },
    Distance { id: String, refs: Vec<Ref>, value: Expr, offset: f64 },
    Hdist { id: String, refs: Vec<Ref>, value: Expr, offset: f64 },
    Vdist { id: String, refs: Vec<Ref>, value: Expr, offset: f64 },
    Pldist { id: String, refs: Vec<Ref>, value: Expr, offset: f64 },
    Diameter { id: String, refs: Vec<Ref>, value: Expr, offset: f64 },
    Radius { id: String, refs: Vec<Ref>, value: Expr, offset: f64 },
}
