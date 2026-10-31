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

/// The mass properties SolidWorks stored in the file (property `SW-MassProp-Config-0` of
/// `docProps/ISolidWorksInformation.xml`, modern container only), in the SI units as stored: centre of mass (m), volume
/// (m^3, six decimals), surface area (m^2, six decimals, i.e. 1 mm^2 resolution) and mass (kg). An independent check of the
/// imported body.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SwMassProps {
    pub com_m: [f64; 3],
    pub volume_m3: f64,
    pub area_m2: f64,
    pub mass_kg: f64,
}

fn sw_mass_props(c: &container::Container) -> Option<SwMassProps> {
    let s = c.stream("docProps/ISolidWorksInformation.xml")?;
    let text = String::from_utf8_lossy(&s.data);
    let at = text.find("SW-MassProp-Config-0\"")?;
    let rest = &text[at..];
    let a = rest.find("<vt:lpstr>")? + 10;
    let b = rest[a..].find("</vt:lpstr>")? + a;
    let v: Vec<f64> = rest[a..b].split(',').filter_map(|x| x.trim().parse().ok()).collect();
    if v.len() < 6 || v.iter().any(|x| !x.is_finite()) {
        return None;
    }
    Some(SwMassProps { com_m: [v[0], v[1], v[2]], volume_m3: v[3], area_m2: v[4], mass_kg: v[5] })
}

#[derive(Debug)]
pub struct SldprtImport {
    /// Mass properties stored by SolidWorks in the file, if present.
    pub sw_mass: Option<SwMassProps>,
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
    /// What became of each solid region (faces built, faces left out with the reason).
    pub parts: Vec<build::PartRead>,
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
    let note_err = |e: SldprtError, first_err: &mut Option<SldprtError>| {
        if first_err.is_none() {
            *first_err = Some(e);
        }
    };
    // 1. the live partition (most faces wins; the others only if it fails)
    let mut parts: Vec<(ir::Ir, usize, String)> = vec![];
    let mut nonsolid = 0usize;
    for t in &ts {
        let attempt = (|| {
            let x = xt::read_xt(&t.data, limits)?;
            let ir = ir::extract(&x, limits)?;
            Ok::<_, SldprtError>((ir, x.nodes.len()))
        })();
        match attempt {
            Ok((ir, n)) => {
                nonsolid += ir.bodies.iter().filter(|b| b.kind != ir::BodyKind::Solid).count();
                if t.kind == TransmitKind::Part {
                    parts.push((ir, n, t.stream.clone()));
                    continue;
                }
                if ir.solid_faces() == 0 {
                    continue;
                }
                match build::build(&[&ir], limits) {
                    Ok(built) => {
                        return Ok(SldprtImport { sw_mass: sw_mass_props(&c), body: built.body, solids: built.solids, container: c.kind, schema: ir.schema.clone(), stream: t.stream.clone(), unit_scale: build::SCALE, approx: built.approx, notes: built.notes, nodes: n, parts: built.parts });
                    }
                    Err(e) => note_err(e, &mut first_err),
                }
            }
            Err(e) => note_err(e, &mut first_err),
        }
    }
    // 2. an empty partition: solid bodies of the free-standing part transmits (imported bodies)
    let irs: Vec<&ir::Ir> = parts.iter().filter(|p| p.0.solid_faces() > 0).map(|p| &p.0).collect();
    if !irs.is_empty() {
        match build::build(&irs, limits) {
            Ok(mut built) => {
                built.notes.push(format!("live partition is empty; {} solid feature body transmit(s) taken", irs.len()));
                let n = parts.iter().map(|p| p.1).sum();
                return Ok(SldprtImport { sw_mass: sw_mass_props(&c), body: built.body, solids: built.solids, container: c.kind, schema: irs[0].schema.clone(), stream: parts[0].2.clone(), unit_scale: build::SCALE, approx: built.approx, notes: built.notes, nodes: n, parts: built.parts });
            }
            Err(e) => note_err(e, &mut first_err),
        }
    }
    if nonsolid > 0 && first_err.as_ref().map_or(true, |e| e.code == "xt.empty" || e.code == "unsupported.body") {
        return Err(SldprtError::new("unsupported.body", format!("no solid body in the file: it holds {nonsolid} sheet / wire body(ies), which are not imported")));
    }
    Err(first_err.unwrap_or_else(|| SldprtError::new("container.no_parasolid", "no usable Parasolid transmit")))
}
