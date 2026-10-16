//! ck-sldprt: a SolidWorks part (`.SLDPRT`) reader that builds an `ck-topo` [`Body`] directly (no STEP hop).
//!
//! Pipeline: [`container`] (phase A: modern block container or OLE2/CFB -> named streams, CRC checked, size
//! limited) -> [`xt`] (phase B: Parasolid XT neutral-binary transmit with embedded schema edits) -> [`ir`]
//! (bodies / regions / shells / faces / loops / fins / edges / vertices plus carriers) -> [`build`]
//! (phase C: kernel body in millimetres, void region skipped, seams and loop order fixed up, approximated carriers
//! reported).
//!
//! The reader never panics on malformed input; failures are [`SldprtError`] values with stable codes.
//! `SolidWorks` and `Parasolid` are trademarks of their owners; they are used here only to name the file formats read.

#![forbid(unsafe_code)]

pub mod build;
pub mod container;
pub mod error;
pub mod inflate;
pub mod ir;
pub mod xt;

pub use build::Approx;
pub use container::Kind;
pub use error::{Limits, SldprtError};

use container::TransmitKind;
use ck_topo::body::Body;

#[derive(Debug)]
pub struct SldprtImport {
    pub body: Body,
    pub solids: usize,
    pub container: Kind,
    /// Parasolid schema name of the transmit that was read.
    pub schema: String,
    /// Stream the body came from.
    pub stream: String,
    /// Millimetres per file unit (applied).
    pub unit_scale: f64,
    pub approx: Vec<Approx>,
    pub notes: Vec<String>,
    /// XT node count of the transmit that was read.
    pub nodes: usize,
}

/// Read a part file with the default limits.
pub fn import_sldprt(bytes: &[u8]) -> Result<SldprtImport, SldprtError> {
    import_sldprt_with(bytes, &Limits::default())
}

pub fn import_sldprt_with(bytes: &[u8], limits: &Limits) -> Result<SldprtImport, SldprtError> {
    let c = container::open(bytes, limits)?;
    let mut ts = c.transmits(limits)?;
    if ts.is_empty() {
        return Err(SldprtError::new("container.no_parasolid", "no Parasolid transmit found in the file"));
    }
    // partitions first (largest first), then free-standing parts, ghost partitions last
    let rank = |t: &container::Transmit| match (t.kind, t.stream.contains("Ghost")) {
        (TransmitKind::Partition, false) => 0,
        (TransmitKind::Part, _) => 1,
        (TransmitKind::Partition, true) => 2,
        _ => 9,
    };
    ts.retain(|t| rank(t) < 9);
    ts.sort_by(|a, b| rank(a).cmp(&rank(b)).then(b.data.len().cmp(&a.data.len())));
    let mut first_err: Option<SldprtError> = None;
    for t in &ts {
        let attempt = (|| {
            let x = xt::read_xt(&t.data, limits)?;
            let ir = ir::extract(&x, limits)?;
            let built = build::build(&ir, limits)?;
            Ok::<_, SldprtError>((built, ir.schema.clone(), x.nodes.len()))
        })();
        match attempt {
            Ok((built, schema, nodes)) => {
                return Ok(SldprtImport { body: built.body, solids: built.solids, container: c.kind, schema, stream: t.stream.clone(), unit_scale: build::SCALE, approx: built.approx, notes: built.notes, nodes });
            }
            Err(e) => {
                // an empty partition (no solid) is not an error worth reporting if a later transmit works
                if first_err.is_none() || (first_err.as_ref().map_or(false, |f| f.code == "unsupported.body" || f.code == "xt.empty") && e.code != "unsupported.body" && e.code != "xt.empty") {
                    first_err = Some(e);
                }
            }
        }
    }
    Err(first_err.unwrap_or_else(|| SldprtError::new("container.no_parasolid", "no usable Parasolid transmit")))
}
