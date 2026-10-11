//! Data model: Part documents, features, sketches, constraints, references.
//!
//! Everything here is the wire format of `.ckpart` files and kernel messages.
//! Field names MUST match the JSON exactly — the schema is pinned by the
//! .ckpart samples and oracle goldens. JSON mapping lives in `ck-json`
//! (the original kernel has no serde; it uses a hand-rolled DOM).

pub mod expr;
pub mod features;
pub mod interference;
pub mod refs;
pub mod regen;
pub mod sketch;
pub mod sketch3;

pub use sketch::{Constraint, Sketch, SketchEntity, SketchPoint};

/// Standard sketch planes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StdPlane {
    Front,
    Top,
    Right,
}

/// Sketch/assembly plane. JSON: `{"kind":"std","id":"front"}` or
/// `{"kind":"offset","base":"...","offset":10,"flip":false}`.
#[derive(Debug, Clone)]
pub enum Plane {
    Std { id: StdPlane },
    Offset { base: Ref, offset: f64, flip: bool },
}

/// Reference selector string: `"F3.e2"` (sketch entity), `"F5.p1"` (sketch
/// point), `"F4.C0@F1.1"` (derived face), `"F8.B0"` (blend ball), etc.
/// Kept as an opaque string on the wire; resolved by `ck-model::refs`.
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub struct Ref(pub String);

impl Ref {
    pub fn as_str(&self) -> &str {
        &self.0
    }
}

impl std::fmt::Display for Ref {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(&self.0)
    }
}

impl From<&str> for Ref {
    fn from(s: &str) -> Self {
        Ref(s.to_string())
    }
}

/// Dimensional constraint values are expression strings, e.g. `"20"`, `"d/2"`.
/// Evaluated by `ck-model::expr`.
#[derive(Debug, Clone, PartialEq)]
pub struct Expr(pub String);

impl Expr {
    pub fn as_str(&self) -> &str {
        &self.0
    }
}

impl From<&str> for Expr {
    fn from(s: &str) -> Self {
        Expr(s.to_string())
    }
}

/// Part document — the `.ckpart` file format and kernel input.
/// JSON: `{"format":"ck-part","version":1,"name":...,"features":[...],
/// "seq":1,"rollback":null,"material":"steel","counters":{...}}`
#[derive(Debug, Clone)]
pub struct Part {
    /// Always `"ck-part"`.
    pub format: String,
    /// Always 1.
    pub version: u32,
    pub name: String,
    /// The regen program, in order.
    pub features: Vec<Feature>,
    /// Monotonic counter used to mint feature ids (`"F" + seq`).
    pub seq: u32,
    pub rollback: Option<Ref>,
    pub material: String,
    /// Per-collection counters (point/entity/constraint id minting).
    pub counters: Counters,
}

#[derive(Debug, Clone, Default)]
pub struct Counters {
    pub sketch: u32,
    pub point: u32,
    pub entity: u32,
    pub constraint: u32,
}

/// Feature envelope shared by all kinds.
#[derive(Debug, Clone)]
pub struct Feature {
    pub id: String,
    /// Display ordinal.
    pub num: u32,
    pub name: String,
    pub kind: features::FeatureKind,
}

impl Part {
    /// Find a feature by id (`"F4"`).
    pub fn feature(&self, id: &str) -> Option<&Feature> {
        self.features.iter().find(|f| f.id == id)
    }
}
