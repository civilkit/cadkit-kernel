//! The whole path from the bytes of an `.f3d` / `.f3z` file, or of one `.smb` / `.smbh` body blob,
//! to `ck-topo` bodies and a report in plain English.
//!
//! Every ASM body of a blob is converted ([`crate::convert`]) and built ([`crate::build`]) into one
//! [`Import::body`], a lump (or several) per ASM body. What became of each is told by
//! [`Import::sort`], with the same honesty as the STEP and SolidWorks imports (`ck_heal::honest`): a
//! body whose faces make a valid closed solid is a solid; a body that lost a face the reader could
//! not make, or that does not close, is a surface body of the faces that came in, with the
//! reasons; a body with no face at all is refused. Nothing is ever kept as a solid that is not one.
//!
//! Units: Fusion stores centimetres, the bodies come out in millimetres.
//!
//! Only Windows-written files (ASM binary version 4, `BinaryFile4`) have been seen; the macOS
//! `BinaryFile8` writer is untested (see `docs/f3d.md`).

use crate::asm::AsmError;
use crate::build::{SkippedFace, build_body};
use crate::container::{self, Blob};
use crate::convert::Options as ConvertOptions;
use ck_heal::honest::{self, PartIn, Reason};
use ck_topo::body::{Body, LumpId};

/// Options of an import.
#[derive(Clone, Copy, Debug)]
pub struct ImportOptions {
    /// The conversion's options (centimetres to millimetres by default).
    pub convert: ConvertOptions,
    /// Read every body of the history blobs (`.smbh`). A history blob holds the bodies of the
    /// design in their stored state, and some of them are also in the plain blob (`.smb`): by
    /// default a history body that has an identical twin in a plain blob is left out (read
    /// once), the others are read.
    pub history: bool,
    /// Read every body of every blob. By default only the design's own bodies are read: the
    /// top-level bodies of the document's history blob (`.smbh`, the design as it stands), or,
    /// for a document without one, the top-level bodies of its plain blobs (`.smb`). A document
    /// with a history blob keeps in its plain blobs the bodies of earlier timeline steps, which
    /// are not parts of the design, and bodies that only own a face or an edge the design refers
    /// to (not top-level) are not parts either.
    pub all_bodies: bool,
}

impl Default for ImportOptions {
    fn default() -> Self {
        ImportOptions {
            convert: ConvertOptions::default(),
            history: false,
            all_bodies: false,
        }
    }
}

/// A file or blob that could not be read at all: a stable code and what went wrong.
#[derive(Clone, Debug, PartialEq)]
pub struct ImportError {
    pub code: &'static str,
    pub message: String,
}

impl std::fmt::Display for ImportError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}: {}", self.code, self.message)
    }
}

impl std::error::Error for ImportError {}

impl From<crate::zip::ZipError> for ImportError {
    fn from(e: crate::zip::ZipError) -> ImportError {
        ImportError {
            code: "f3d.container",
            message: e.to_string(),
        }
    }
}

fn asm_error(entry: &str, e: AsmError) -> ImportError {
    ImportError {
        code: "f3d.asm",
        message: format!("{entry}: {e}"),
    }
}

/// One body blob that was read.
#[derive(Clone, Debug)]
pub struct BlobInfo {
    pub entry: String,
    pub guid: String,
    pub history: bool,
    /// The ASM header's version text.
    pub asm_version: String,
    pub bodies: usize,
    /// Tokenising stopped early (an unknown tag): the message.
    pub truncated: Option<String>,
}

/// One ASM body and what became of it.
#[derive(Clone, Debug)]
pub struct ImportedBody {
    /// Index into [`Import::blobs`].
    pub blob: usize,
    /// The `body` record in the ASM file.
    pub record: usize,
    /// The body's place in the face names: its faces are `f3d.<tag>f<n>` (`n` = the face's index
    /// in the reader's body).
    pub tag: String,
    /// A top-level entity of the file (otherwise the body owns a face or an edge the design
    /// refers to).
    pub top_level: bool,
    pub label: String,
    /// The lumps in [`Import::body`] (none when the body was refused outright).
    pub lumps: Vec<LumpId>,
    /// The faces the file gives the body.
    pub faces_total: usize,
    pub faces_built: usize,
    /// Faces left out: those the reader could not convert and those the builder could not make.
    pub skipped: Vec<SkippedFace>,
    /// Unsupported or undecodable geometry and topology the reader met, one line each.
    pub issues: Vec<String>,
    /// Edges without a usable curve (kept in the loops of the faces that were left out).
    pub unknown_edges: usize,
    /// Edges and vertices that were given a tolerance (the file's geometry deviates from itself
    /// by more than the strict tolerance there).
    pub tolerances_set: usize,
    /// Edges and vertices that deviate by more than the largest tolerance that may be set.
    pub over_tolerance: usize,
    /// Edges split so that the loops of a periodic face could be joined.
    pub splits: usize,
    /// Why no part of the body could be built, if so.
    pub error: Option<String>,
}

/// The result of an import.
#[derive(Debug, Default)]
pub struct Import {
    /// Every body that was built (lumps, in file order), in millimetres.
    pub body: Body,
    pub blobs: Vec<BlobInfo>,
    pub bodies: Vec<ImportedBody>,
    /// Things worth telling about the file as a whole, in plain words.
    pub warnings: Vec<String>,
}

/// Whether the bytes are a zip file (`.f3d` / `.f3z`) rather than a bare ASM blob.
pub fn is_container(bytes: &[u8]) -> bool {
    bytes.starts_with(b"PK")
}

/// Imports an `.f3d` or `.f3z` file, or a bare `.smb` / `.smbh` body blob.
pub fn import(bytes: &[u8], opts: &ImportOptions) -> Result<Import, ImportError> {
    if is_container(bytes) {
        import_f3d(bytes, opts)
    } else {
        import_smb(bytes, opts)
    }
}

/// Imports an `.f3d` or `.f3z` file: the body blobs of its document, or of each document of a
/// package.
pub fn import_f3d(bytes: &[u8], opts: &ImportOptions) -> Result<Import, ImportError> {
    let blobs = container::read_blobs(bytes)?;
    if blobs.is_empty() {
        return Err(ImportError {
            code: "f3d.empty",
            message: "the file holds no body blob (Breps.BlobParts/BREP.*.smb): it may be a design with no solid geometry, or not a Fusion file".into(),
        });
    }
    import_blobs(&blobs, opts)
}

/// Imports one `.smb` / `.smbh` body blob.
pub fn import_smb(bytes: &[u8], opts: &ImportOptions) -> Result<Import, ImportError> {
    let blob = Blob {
        entry: "body.smb".into(),
        guid: String::new(),
        history: false,
        data: bytes.to_vec(),
    };
    import_blobs(&[blob], opts)
}

/// What tells two bodies from different blobs apart cheaply: the counts and the box.
fn signature(b: &crate::brep::Body) -> (usize, usize, usize, [f64; 6]) {
    let mut bb = [f64::INFINITY, f64::INFINITY, f64::INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY];
    for v in &b.vertices {
        for k in 0..3 {
            bb[k] = bb[k].min(v.point[k]);
            bb[k + 3] = bb[k + 3].max(v.point[k]);
        }
    }
    (b.faces.len(), b.edges.len(), b.vertices.len(), bb)
}

fn same_signature(a: &(usize, usize, usize, [f64; 6]), b: &(usize, usize, usize, [f64; 6])) -> bool {
    a.0 == b.0 && a.1 == b.1 && a.2 == b.2 && a.3.iter().zip(&b.3).all(|(x, y)| (x - y).abs() <= 1e-6)
}

/// Imports blobs already read from a container.
pub fn import_blobs(blobs: &[Blob], opts: &ImportOptions) -> Result<Import, ImportError> {
    let mut imp = Import::default();
    let mut read = Vec::new();
    for blob in blobs {
        let r = crate::read_blob(&blob.entry, &blob.data, &opts.convert)
            .map_err(|e| asm_error(&blob.entry, e))?;
        read.push(r);
    }
    // the bodies of the plain blobs, to find the history bodies that are twins of them
    let plain: Vec<_> = blobs
        .iter()
        .zip(&read)
        .filter(|(b, _)| !b.history)
        .flat_map(|(_, r)| r.bodies.iter().map(|cb| signature(&cb.body)))
        .collect();
    // the documents whose history blob holds bodies of the design
    let doc_of = |b: &Blob| b.entry.rsplit_once("/Breps.BlobParts/").map(|x| x.0.to_string()).unwrap_or_default();
    let history_docs: std::collections::HashSet<String> = blobs
        .iter()
        .zip(&read)
        .filter(|(b, r)| b.history && r.bodies.iter().any(|cb| cb.top_level))
        .map(|(b, _)| doc_of(b))
        .collect();
    let (mut twins, mut earlier, mut internal) = (0usize, 0usize, 0usize);
    for (blob, r) in blobs.iter().zip(&read) {
        let bi = imp.blobs.len();
        imp.blobs.push(BlobInfo {
            entry: blob.entry.clone(),
            guid: blob.guid.clone(),
            history: blob.history,
            asm_version: format!("{} (BinaryFile{})", r.header.asm_version, r.header.int_size),
            bodies: r.bodies.len(),
            truncated: r.truncated.clone(),
        });
        if let Some(t) = &r.truncated {
            imp.warnings.push(format!(
                "{}: the ASM data stops early ({t}); the bodies after that point are missing",
                blob.entry
            ));
        }
        let short = blob_label(blob);
        for cb in &r.bodies {
            if !opts.all_bodies {
                if !cb.top_level {
                    internal += 1;
                    continue;
                }
                if !blob.history && history_docs.contains(&doc_of(blob)) {
                    earlier += 1;
                    continue;
                }
            } else if blob.history && !opts.history {
                let sig = signature(&cb.body);
                if plain.iter().any(|p| same_signature(p, &sig)) {
                    twins += 1;
                    continue;
                }
            }
            let tag = format!("b{bi}r{}", cb.record);
            let mut ib = ImportedBody {
                blob: bi,
                record: cb.record,
                tag: tag.clone(),
                top_level: cb.top_level,
                label: format!("{short} body {}", cb.record),
                lumps: vec![],
                faces_total: cb.body.faces.len(),
                faces_built: 0,
                skipped: vec![],
                issues: cb.issues.clone(),
                unknown_edges: cb.unknown_edges,
                tolerances_set: 0,
                over_tolerance: 0,
                splits: 0,
                error: None,
            };
            match build_body(&mut imp.body, &cb.body, &tag) {
                Ok(b) => {
                    ib.lumps = b.lumps;
                    ib.faces_built = b.faces_built;
                    ib.skipped = b.skipped;
                    ib.tolerances_set = b.tolerances_set;
                    ib.over_tolerance = b.over_tolerance;
                    ib.splits = b.splits;
                }
                Err(e) => ib.error = Some(e),
            }
            imp.bodies.push(ib);
        }
    }
    if earlier + internal > 0 {
        let mut parts = Vec::new();
        if earlier > 0 {
            parts.push(format!("{earlier} bod{} of earlier timeline steps (kept in the plain blobs beside the design's history blob)", if earlier == 1 { "y" } else { "ies" }));
        }
        if internal > 0 {
            parts.push(format!("{internal} bod{} that only own a face or an edge the design refers to", if internal == 1 { "y" } else { "ies" }));
        }
        imp.warnings.push(format!("Not read as parts of the design: {} (opts.allBodies reads them).", parts.join(" and ")));
    }
    if twins > 0 {
        imp.warnings.push(format!(
            "{twins} bod{} of a history blob (.smbh) {} identical to {} in a plain blob (.smb) and {} read once (opts.history reads them again).",
            if twins == 1 { "y" } else { "ies" },
            if twins == 1 { "is" } else { "are" },
            if twins == 1 { "one" } else { "ones" },
            if twins == 1 { "was" } else { "were" },
        ));
    }
    Ok(imp)
}

fn blob_label(b: &Blob) -> String {
    let file = b.entry.rsplit('/').next().unwrap_or(&b.entry);
    let doc = b.entry.split('/').next().filter(|d| d.ends_with(".f3d") && *d != file);
    let short: String = b.guid.chars().take(8).collect();
    let base = if short.is_empty() { file.to_string() } else { format!("BREP.{short}") };
    match doc {
        Some(d) => format!("{} {base}", d.trim_end_matches(".f3d").chars().take(8).collect::<String>()),
        None => base,
    }
}

impl ImportedBody {
    /// The faces left out, grouped by what was wrong, as the reasons a part carries.
    pub fn reasons(&self) -> Vec<Reason> {
        let mut groups: Vec<(&'static str, &str, Vec<String>)> = Vec::new();
        for s in &self.skipped {
            let name = format!("f3d.{}f{}", self.tag, s.face);
            match groups.iter_mut().find(|g| g.0 == s.code && g.1 == s.message) {
                Some(g) => g.2.push(name),
                None => groups.push((s.code, &s.message, vec![name])),
            }
        }
        groups
            .into_iter()
            .map(|(code, msg, faces)| {
                let message = match faces.len() {
                    1 => format!("face {} was left out: {msg}", faces[0]),
                    n => format!("{n} faces were left out ({}): {msg}", faces.join(", ")),
                };
                Reason {
                    code: code.to_string(),
                    faces,
                    message,
                }
            })
            .collect()
    }
}

impl Import {
    /// What became of each body, as the other imports tell it: the valid solids, the surface
    /// bodies (index into [`Import::bodies`] and body) and a part per body, in order.
    pub fn sort(&self) -> honest::Sorted {
        let parts: Vec<PartIn> = self
            .bodies
            .iter()
            .map(|b| {
                let mut faces = Vec::new();
                for &l in &b.lumps {
                    faces.extend(ck_topo::mass::lump_faces(&self.body, l));
                }
                let mut reasons = b.reasons();
                if let Some(e) = &b.error {
                    reasons.push(Reason {
                        code: "buildFailed".into(),
                        faces: vec![],
                        message: format!("the body could not be built: {e}"),
                    });
                }
                for i in &b.issues {
                    reasons.push(Reason {
                        code: "readerIssue".into(),
                        faces: vec![],
                        message: i.clone(),
                    });
                }
                PartIn {
                    label: b.label.clone(),
                    faces,
                    faces_total: b.faces_total,
                    reasons,
                }
            })
            .collect();
        honest::sort(&self.body, &parts, ck_check::DEFAULT_TOL_CAP)
    }

    /// What was read and what was left out, in plain words (one line each), given the outcome of
    /// [`Import::sort`]. The first line is the summary.
    pub fn report(&self, sorted: &honest::Sorted) -> Vec<String> {
        use honest::Outcome;
        let n = |o: Outcome| sorted.parts.iter().filter(|p| p.outcome == o).count();
        let (solids, surfaces, refused) = (n(Outcome::Solid), n(Outcome::Surface), n(Outcome::Refused));
        let built: usize = self.bodies.iter().map(|b| b.faces_built).sum();
        let total: usize = self.bodies.iter().map(|b| b.faces_total).sum();
        let mut lines = vec![format!(
            "Read {} body blob(s) holding {} ASM bod{}: {solids} valid solid(s), {surfaces} surface bod{} (sheets, or bodies that are not closed solids), {refused} refused; {built} of {total} faces were built.",
            self.blobs.len(),
            self.bodies.len(),
            if self.bodies.len() == 1 { "y" } else { "ies" },
            if surfaces == 1 { "y" } else { "ies" },
        )];
        let mut kinds: std::collections::BTreeMap<&str, usize> = std::collections::BTreeMap::new();
        for b in &self.bodies {
            for s in b.skipped.iter().filter(|s| !s.what.is_empty()) {
                *kinds.entry(s.what.as_str()).or_default() += 1;
            }
        }
        if !kinds.is_empty() {
            let list: Vec<String> = kinds.iter().map(|(k, c)| format!("{k} ({c})")).collect();
            lines.push(format!(
                "Faces left out because their surface or an edge's curve is not imported: {}.",
                list.join(", ")
            ));
        }
        for p in sorted.parts.iter().filter(|p| p.outcome != Outcome::Solid) {
            if lines.len() > 40 {
                lines.push("(more bodies that are not solids are not listed)".into());
                break;
            }
            let why: Vec<&str> = p.reasons.iter().map(|r| r.message.as_str()).collect();
            let what = match p.outcome {
                Outcome::Refused => "nothing of it could be read".to_string(),
                _ => format!("it came in as a surface body of {} of its {} faces", p.faces, p.faces_total),
            };
            lines.push(if why.is_empty() {
                format!("{}: {what}.", p.label)
            } else {
                format!("{}: {what}: {}.", p.label, why.join("; "))
            });
        }
        let set: usize = self.bodies.iter().map(|b| b.tolerances_set).sum();
        if set > 0 {
            lines.push(format!(
                "{set} edge(s) and vertex(es) were given a tolerance because the file's geometry deviates from itself by more than a micrometre there."
            ));
        }
        let splits: usize = self.bodies.iter().map(|b| b.splits).sum();
        if splits > 0 {
            lines.push(format!(
                "{splits} edge(s) were split so that the two loops of a cylindrical, conical, spherical or toroidal face could be joined by a seam."
            ));
        }
        for bl in &self.blobs {
            if bl.asm_version.contains("BinaryFile8") {
                lines.push(format!(
                    "{}: written by the BinaryFile8 variant of the ASM format (macOS Fusion), which this reader has not been tested on; check the result.",
                    bl.entry
                ));
            }
        }
        lines.extend(self.warnings.iter().cloned());
        lines
    }
}
