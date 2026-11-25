//! The safety guard: regenerate the written features with the kernel and compare the result with the body the ASM
//! reader builds from the same file. A history whose volume or box differs beyond the tolerance is marked
//! [`Verdict::Unverified`] with the delta, so the app never shows a history as the design's when it is not.
//!
//! The idea is Dutchman's `guard.rs` (Apache-2.0: a carried history that cannot be the stored solid is not trusted),
//! done here with the real kernel instead of an estimate from the sketch areas.

use ck_model::Part;
use ck_model::features::Brep;
use ck_model::regen::{self, FeatureExec, RegenOpts};
use ck_topo::body::Body;
use serde_json::Value;

use crate::FeatureImport;

/// A body's volume (mm^3) and box (mm), the whole of its lumps together.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Measure {
    pub volume: f64,
    pub min: [f64; 3],
    pub max: [f64; 3],
    /// How many solids (lumps).
    pub solids: usize,
}

/// The volume may differ by this fraction of the reference (tessellation-free integrals of both bodies).
pub const VOLUME_TOLERANCE: f64 = 2e-3;
/// Each side of the box may differ by this many mm plus [`BOX_TOLERANCE_REL`] of the box diagonal.
pub const BOX_TOLERANCE_MM: f64 = 0.01;
pub const BOX_TOLERANCE_REL: f64 = 1e-4;

/// The result of the guard.
#[derive(Clone, Debug, PartialEq)]
pub enum Verdict {
    /// The regenerated history has the volume and box of the imported body.
    Verified,
    /// It does not (or could not be compared): the history is shown for reference only. The reason is in the check.
    Unverified,
}

/// What the guard found.
#[derive(Clone, Debug)]
pub struct Check {
    pub verdict: Verdict,
    /// The imported body (the reference), when it could be built.
    pub reference: Option<Measure>,
    /// The regenerated history's body, when it built one.
    pub history: Option<Measure>,
    /// `(history - reference) / reference` of the volume.
    pub volume_delta: Option<f64>,
    /// The largest difference of a box side (mm).
    pub box_delta: Option<f64>,
    /// Why it is unverified, in one line.
    pub reason: Option<String>,
    /// The regeneration errors (`feature id: code`).
    pub errors: Vec<String>,
}

impl Check {
    fn unverified(reason: impl Into<String>) -> Check {
        Check {
            verdict: Verdict::Unverified,
            reference: None,
            history: None,
            volume_delta: None,
            box_delta: None,
            reason: Some(reason.into()),
            errors: Vec::new(),
        }
    }
}

/// The volume and box of the design bodies the ASM reader builds from the file (the app's default import).
pub fn measure_import(bytes: &[u8]) -> Result<Measure, String> {
    let imp = std::panic::catch_unwind(|| {
        let imp = ck_asm::import::import_f3d(bytes, &ck_asm::import::ImportOptions::default()).map_err(|e| e.to_string())?;
        let sorted = imp.sort();
        let m = measure(&sorted.solid);
        if m.solids == 0 {
            return Err("the file's design has no closed solid to compare with".to_string());
        }
        Ok::<Measure, String>(m)
    })
    .map_err(|_| "reading the body panicked".to_string())??;
    Ok(imp)
}

/// The reference of each design of a file, in the order `ck_f3d_design::decode_bytes` gives the designs: the documents of an
/// `.f3z` package are each read on their own (a package's designs are separate documents with body blobs of their own), so a
/// package design is compared with its own body; any other file is one document.
pub fn measure_documents(bytes: &[u8]) -> Vec<Result<Measure, String>> {
    if bytes.starts_with(b"PK")
        && let Ok(archive) = ck_asm::zip::Archive::new(bytes)
    {
        let docs: Vec<&ck_asm::zip::Entry> = archive.entries().iter().filter(|e| e.name.to_lowercase().ends_with(".f3d")).collect();
        if !docs.is_empty() {
            return docs
                .into_iter()
                .map(|e| match archive.read(e) {
                    Ok(doc) => measure_import(&doc),
                    Err(err) => Err(err.to_string()),
                })
                .collect();
        }
    }
    vec![measure_import(bytes)]
}

fn measure(body: &Body) -> Measure {
    let volume = ck_topo::mass::mass_props(body).volume;
    let (lo, hi) = body.bbox();
    Measure { volume, min: [lo.x, lo.y, lo.z], max: [hi.x, hi.y, hi.z], solids: body.lump_ids().count() }
}

struct Exec(Brep);

impl FeatureExec for Exec {
    fn face_count(&self) -> Option<usize> {
        self.0.face_count()
    }
    fn face_frame(&mut self, s: &regen::RegenState, face: &str) -> Option<ck_core::linalg::Frame> {
        self.0.face_frame(s, face)
    }
    fn exec(&mut self, s: &mut regen::RegenState, f: &ck_model::Feature) -> Result<(), regen::FeatureError> {
        self.0.exec(s, f)
    }
}

/// Regenerates a part document and returns the body it ends with (the solid slot), for a mapping step that has to find
/// an edge of the history built so far.
pub(crate) fn regenerate_body(part: &Value) -> Option<Body> {
    let text = serde_json::to_string(part).ok()?;
    std::panic::catch_unwind(|| {
        let part = Part::parse(&text).ok()?;
        let mut ex = Exec(Brep::new());
        let _ = regen::regen(&part, &RegenOpts::default(), &mut ex);
        ex.0.body.take().filter(|b| b.lump_ids().count() > 0)
    })
    .ok()
    .flatten()
}

/// Regenerates a part document: the volume and box of its body (`None` when it built none), and the errors.
pub fn regenerate(part: &Value) -> Result<(Option<Measure>, Vec<String>), String> {
    let text = serde_json::to_string(part).map_err(|e| e.to_string())?;
    std::panic::catch_unwind(|| {
        let part = Part::parse(&text).map_err(|e| format!("the part does not parse: {e:?}"))?;
        let mut ex = Exec(Brep::new());
        let r = regen::regen(&part, &RegenOpts::default(), &mut ex);
        let errors = r
            .errors
            .iter()
            .map(|e| {
                let detail = e.error.params.iter().find(|(k, _)| k == "detail").and_then(|(_, v)| v.as_str().map(str::to_owned));
                match detail {
                    Some(d) => format!("{}: {} ({d})", e.feature, e.error.code),
                    None => format!("{}: {}", e.feature, e.error.code),
                }
            })
            .collect();
        Ok::<_, String>((ex.0.body.as_ref().filter(|b| b.lump_ids().count() > 0).map(measure), errors))
    })
    .map_err(|_| "regenerating the history panicked".to_string())?
}

/// Whether two measures agree within the tolerances; the deltas are returned either way.
pub fn compare(reference: &Measure, history: &Measure) -> (bool, f64, f64) {
    let volume_delta = (history.volume - reference.volume) / reference.volume.abs().max(1e-12);
    let diagonal = reference.min.iter().zip(&reference.max).map(|(l, h)| (h - l).powi(2)).sum::<f64>().sqrt();
    let box_delta = (0..3)
        .flat_map(|i| [(history.min[i] - reference.min[i]).abs(), (history.max[i] - reference.max[i]).abs()])
        .fold(0.0, f64::max);
    let ok = volume_delta.abs() <= VOLUME_TOLERANCE && box_delta <= BOX_TOLERANCE_MM + BOX_TOLERANCE_REL * diagonal;
    (ok, volume_delta, box_delta)
}

/// Regenerates the import's features and compares them with the reference body.
pub fn verify(import: &FeatureImport, reference: Option<&Measure>) -> Check {
    if import.features.is_empty() {
        return Check::unverified("the history has no features");
    }
    let regenerated = match regenerate(&import.part()) {
        Ok(r) => r,
        Err(e) => return Check::unverified(e),
    };
    let (history, errors) = regenerated;
    let mut check = Check {
        verdict: Verdict::Unverified,
        reference: reference.copied(),
        history,
        volume_delta: None,
        box_delta: None,
        reason: None,
        errors,
    };
    let Some(reference) = reference else {
        check.reason = Some("the imported body could not be built to compare with".into());
        return check;
    };
    let Some(history) = check.history else {
        check.reason = Some("the history builds no body".into());
        return check;
    };
    let (ok, dv, db) = compare(reference, &history);
    check.volume_delta = Some(dv);
    check.box_delta = Some(db);
    if !check.errors.is_empty() {
        check.reason = Some(format!("the history does not regenerate cleanly: {}", check.errors[0]));
    } else if !ok {
        check.reason = Some(format!(
            "the history's body differs from the imported one: volume {:+.3} %, box {:.3} mm",
            dv * 100.0,
            db
        ));
    } else {
        check.verdict = Verdict::Verified;
    }
    check
}
