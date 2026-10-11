//! ACIS SAT import: text -> ck-topo [`Body`] with ck-geom surfaces/curves.
//!
//! Origin: the record parser is adapted from the author's own
//! `sat_import.rs` in SGRebuild (`fem-core`, dependency-free ACIS 7.00 text
//! parser; reused freely). That seed tessellated planar/cone faces; this
//! module instead builds the real B-rep.
//!
//! Supported
//! - surfaces: `plane-surface`, `cone-surface` (cylinder when sine = 0, cone
//!   otherwise; circular sections only), `sphere-surface`, `torus-surface`
//!   (major > |minor|), `spline-surface` with an exact `nubs` / `nurbs` net.
//! - curves: `straight-curve`, `ellipse-curve` (circle when ratio = 1),
//!   `intcurve-curve` with an exact `nubs` / `nurbs` block.
//! - sense flags: face sense, coedge sense, edge sense (resolved
//!   geometrically against the vertices), face `single` / `double` sided
//!   (all-double = sheet body) and containment, header unit scale.
//! - periodic faces without seam edges (ACIS style) get a seam inserted on
//!   cylinders / cones whose two boundary loops wrap the axis.
//!
//! Anything else is a structured [`SatError`] naming the record (`$N`) and
//! entity type; the importer never panics on malformed input.

mod build;
pub mod parse;

use ck_topo::body::Body;

/// Structured import failure.
#[derive(Debug, Clone, PartialEq)]
pub struct SatError {
    /// Stable machine code: `sat.parse`, `sat.topology`, `sat.unsupported`,
    /// `sat.geometry`, `sat.orientation`.
    pub code: &'static str,
    /// Offending record ordinal (`$N`), if any.
    pub entity: Option<usize>,
    /// Entity type of that record (`cone-surface`, ...), empty if unknown.
    pub kind: String,
    /// 1-based file line of the record (0 = unknown).
    pub line: usize,
    pub detail: String,
}

impl SatError {
    pub(crate) fn parse(line: usize, detail: impl Into<String>) -> SatError {
        SatError { code: "sat.parse", entity: None, kind: String::new(), line, detail: detail.into() }
    }
    pub(crate) fn at(code: &'static str, r: &parse::Rec, detail: impl Into<String>) -> SatError {
        SatError { code, entity: Some(r.idx), kind: r.ty().to_string(), line: r.line, detail: detail.into() }
    }
    /// `$N` label of the offending record.
    pub fn entity_id(&self) -> Option<String> {
        self.entity.map(|i| format!("${i}"))
    }
}

impl std::fmt::Display for SatError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self.entity {
            Some(i) => write!(f, "{}: ${} ({}) line {}: {}", self.code, i, self.kind, self.line, self.detail),
            None => write!(f, "{}: line {}: {}", self.code, self.line, self.detail),
        }
    }
}

impl std::error::Error for SatError {}

#[derive(Debug)]
pub struct SatImport {
    pub body: Body,
    /// All faces were double-sided: an open sheet body, not a solid.
    pub sheet: bool,
    /// Millimetres per file unit (already applied to the geometry).
    pub units: f64,
}

/// Import the first/all `body` records of a SAT text into one [`Body`]
/// (millimetres).
pub fn import_sat(text: &str) -> Result<SatImport, SatError> {
    let table = parse::parse_table(text)?;
    build::build(&table)
}
