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
pub mod meta;
pub mod xt;

pub use build::Approx;
pub use container::Kind;
pub use error::{Limits, SldprtError};
pub use meta::{SwMassProps, SwMeta};

use container::TransmitKind;
use ck_topo::body::Body;

#[derive(Debug)]
pub struct SldprtImport {
    /// Mass properties stored by SolidWorks in the file, if present (the same as `meta.mass`).
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
    /// Feature list, configurations, material, properties, units, versions and preview (see [`meta`]).
    pub meta: SwMeta,
    /// Colour and SolidWorks feature of each face built, in body order.
    pub face_tags: Vec<FaceTag>,
    /// Attributes of each solid XT body read.
    pub body_tags: Vec<BodyTag>,
}

/// What the file says about one built face. The kernel body has no per-face colour, so it is reported here.
#[derive(Debug, Clone, PartialEq)]
pub struct FaceTag {
    /// Kernel face id (`sldprt.F<xt face>`).
    pub face: String,
    pub xt_face: i64,
    /// RGB, 0..1.
    pub colour: Option<[f64; 3]>,
    pub sw_id: Option<ir::SwFaceId>,
    /// Name of the feature `sw_id.feature` in the feature list.
    pub feature: Option<String>,
    /// Every feature tag on the face (see [`ir::IrFace::sw_ids`]).
    pub sw_ids: Vec<ir::SwFaceId>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct BodyTag {
    pub xt_body: i64,
    /// Density stored on the body (unit not established, see [`ir::IrBody::density`]).
    pub density: Option<f64>,
    pub last_feature: Option<i64>,
}

impl SldprtImport {
    /// Parasolid modeller version of the transmit read, from the schema name `SCH_<modeller>_<schema>_<base>`: the number
    /// (e.g. 3301247) and its `major.minor` form (`33.01`).
    pub fn parasolid_version(&self) -> Option<(u32, String)> {
        let n: u32 = self.schema.strip_prefix("SCH_")?.split('_').next()?.parse().ok()?;
        Some((n, format!("{}.{:02}", n / 100_000, (n / 1000) % 100)))
    }
}

/// Only the metadata of a part file (no geometry is read): for a file whose body cannot be imported.
pub fn read_meta(bytes: &[u8]) -> Result<SwMeta, SldprtError> {
    let limits = Limits::default();
    Ok(meta::read(&container::open(bytes, &limits)?, &limits))
}

/// One tag per built face, in the order of `built.parts` and their `faces` (XT face ids repeat across the bodies of a
/// multi-body part, so faces are looked up by body and face id).
fn tags(irs: &[&ir::Ir], built: &build::Built, meta: &SwMeta) -> (Vec<FaceTag>, Vec<BodyTag>) {
    let mut faces: std::collections::HashMap<(i64, i64), &ir::IrFace> = Default::default();
    let mut bodies = vec![];
    for b in irs.iter().flat_map(|ir| &ir.bodies).filter(|b| b.kind == ir::BodyKind::Solid) {
        bodies.push(BodyTag { xt_body: b.id, density: b.density, last_feature: b.last_feature });
        for f in b.regions.iter().flat_map(|r| &r.shells).flat_map(|s| &s.faces) {
            faces.entry((b.id, f.id)).or_insert(f);
        }
    }
    let mut out = vec![];
    for p in &built.parts {
        for (fid, xt) in p.faces.iter().zip(&p.face_xt) {
            let Some(f) = faces.get(&(p.body, *xt)) else { continue };
            let feature = f.sw_id.as_ref().and_then(|s| meta.feature(s.feature)).map(|x| x.name.clone());
            out.push(FaceTag { face: built.body.face(*fid).name.id(), xt_face: *xt, colour: f.colour, sw_id: f.sw_id.clone(), feature, sw_ids: f.sw_ids.clone() });
        }
    }
    (out, bodies)
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
                        let meta = meta::read(&c, limits);
                        let (face_tags, body_tags) = tags(&[&ir], &built, &meta);
                        return Ok(SldprtImport { sw_mass: meta.mass, body: built.body, solids: built.solids, container: c.kind, schema: ir.schema.clone(), stream: t.stream.clone(), unit_scale: build::SCALE, approx: built.approx, notes: built.notes, nodes: n, parts: built.parts, meta, face_tags, body_tags });
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
                let meta = meta::read(&c, limits);
                let (face_tags, body_tags) = tags(&irs, &built, &meta);
                return Ok(SldprtImport { sw_mass: meta.mass, body: built.body, solids: built.solids, container: c.kind, schema: irs[0].schema.clone(), stream: parts[0].2.clone(), unit_scale: build::SCALE, approx: built.approx, notes: built.notes, nodes: n, parts: built.parts, meta, face_tags, body_tags });
            }
            Err(e) => note_err(e, &mut first_err),
        }
    }
    if nonsolid > 0 && first_err.as_ref().map_or(true, |e| e.code == "xt.empty" || e.code == "unsupported.body") {
        return Err(SldprtError::new("unsupported.body", format!("no solid body in the file: it holds {nonsolid} sheet / wire body(ies), which are not imported")));
    }
    Err(first_err.unwrap_or_else(|| SldprtError::new("container.no_parasolid", "no usable Parasolid transmit")))
}
