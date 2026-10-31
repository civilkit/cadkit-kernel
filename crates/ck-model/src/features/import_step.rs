//! `importStep`: a STEP body held by the part. On regeneration the STEP data is read with the kernel's STEP reader
//! (`ck-io`) and its solids join the history as bodies, so later features can cut, join or fillet them.
//!
//! ```text
//! {"type":"importStep","id":"F1","blob":"<sha-256 hex>","body":"Case"}
//! {"type":"importStep","id":"F1","step":"ISO-10303-21; ..."}
//! {"type":"importStep","id":"F1","blob":"<sha-256 hex>","heal":true}
//! ```
//!
//! * The data is either a `blob` key into the part's top-level `blobs` table (`{"<sha-256 hex>": "<STEP text>"}`, the key being
//!   the SHA-256 of the UTF-8 text) or, for small files, inline `step` text. The table form is the one to use: the regen cache key
//!   of a feature is a hash of the feature's JSON, so a feature that only names a blob adds a few bytes to the key instead of the
//!   whole file, and the same file used twice is stored once. The key is checked against the text on every run of the feature, so a
//!   cached state can never belong to other data than its key says.
//! * `heal: true` repairs the solids after reading, as the `importStep` message does by default (`ck-heal`: faces turned the other way
//!   are made consistent, free edges sewn). Without it the reader's result is used as it is. A converter that restates the geometry of a
//!   part as STEP can write some faces with the wrong sense; read as it is, the solid is open or has a wrong volume, healed it is the
//!   solid of the source. Leave it off for the STEP of the program that made the part (nothing to repair, and the body stays what the
//!   file says). The flag is part of the feature, so it is part of the regen cache key.
//! * Units come from the file (millimetre, centimetre, metre, inch, ... and degree or radian); the body is in millimetres.
//! * Each solid of the file is checked on its own (`ck_heal::honest`). A solid whose faces make a valid solid joins the history
//!   as a body. Any other (a face the reader could not read, faces that do not close up, ...) is NOT kept as a solid: it becomes
//!   the surface body `<id>.P<k>` (k = the solid's place in the file) of exactly the faces that were read, and the feature
//!   carries the warning `importStepParts {detail, solids, surfaces, refused, parts:[{label, outcome, faces, facesTotal, surface?,
//!   reasons:[{code, faces, message}]}]}` (the "needs repair" mark; `repairPart` repairs them). A file of surface models
//!   (SHELL_BASED_SURFACE_MODEL) comes in the same way, as surface bodies.
//! * Every solid in the file becomes its own body (lump). With `body`, they are named `<body>`, `<body> 2`, `<body> 3`, ...
//!   A feature after existing bodies adds the solids beside them without uniting.
//! * Faces are named `<feature id>.I<solid>F<n>` (for example `F1.I1F7`): the solid and face numbers follow the order of the
//!   file, so the names are the same on every run and after the part is saved and opened again.
//! * Error: `importStepInvalid {detail, reason?, entity?}` (no data, the key does not match the text, the file cannot be read,
//!   the file holds no solid). The feature never panics on bad data.

use super::{extrude::Combine, Brep};
use crate::regen::{FeatureError, RegenState};
use crate::Feature;
use ck_json::Json;
use ck_topo::body::FaceName;

pub const IMPORT_STEP_INVALID: &str = "importStepInvalid";

fn invalid(detail: impl Into<String>) -> FeatureError {
    FeatureError::new(IMPORT_STEP_INVALID).with("detail", Json::Str(detail.into())).with(
        "hint",
        Json::Str("give the feature a `blob` key that names a STEP text in the part's `blobs` table (the key is the SHA-256 of the text), or a small `step` text".into()),
    )
}

/// Lower-case hex SHA-256 of `text`, the key of a blob.
pub fn blob_key(text: &str) -> String {
    ck_text::sha256_hex(&ck_text::sha256(&[text.as_bytes()]))
}

/// The STEP text the feature names, checked against its key.
pub(crate) fn data<'a>(state: &'a RegenState, f: &'a Feature) -> Result<&'a str, FeatureError> {
    if let Some(key) = f.raw.get("blob") {
        let key = key.as_str().ok_or_else(|| invalid("`blob` must be a string (the key of an entry of the part's `blobs`)"))?;
        let text = state
            .part
            .raw
            .get("blobs")
            .and_then(|b| b.get(key))
            .ok_or_else(|| invalid(format!("the part has no blob `{key}` in its `blobs` table")))?
            .as_str()
            .ok_or_else(|| invalid(format!("blob `{key}` is not text")))?;
        if blob_key(text) != key.to_ascii_lowercase() {
            return Err(invalid(format!("blob `{key}` does not match its key: the key must be the SHA-256 of the text")));
        }
        return Ok(text);
    }
    match f.raw.get("step") {
        Some(Json::Str(s)) => Ok(s),
        Some(_) => Err(invalid("`step` must be the STEP text")),
        None => Err(invalid("no STEP data: the feature has neither `blob` nor `step`")),
    }
}

/// What reading the STEP of an `importStep` feature gives: the parts sorted into solids and surface bodies, and for each
/// part the faces the reader left out (with their surfaces, for `repairPart`).
pub struct ReadOut {
    pub sorted: ck_heal::honest::Sorted,
    pub left_out: Vec<Vec<ck_heal::repair::LeftOut>>,
    pub solids: usize,
}

/// The id of the surface body of part `k` (0-based) of import feature `id`: `<id>.P<k+1>`.
pub fn surface_id(id: &str, k: usize) -> String {
    format!("{id}.P{}", k + 1)
}

/// Read `text` into a body whose faces are named `<id>.I<solid>F<n>`; the number of solids with it. Only the parts that
/// make valid solids are in the body (see [`read_parts`]).
pub fn read(id: &str, text: &str, heal: bool) -> Result<(ck_topo::body::Body, usize), FeatureError> {
    let r = read_parts(id, text, heal)?;
    Ok((r.sorted.solid, r.solids))
}

/// Read `text` and sort its parts (`ck_heal::honest`): a part whose faces make a valid solid joins the solid body, any
/// other part with faces is a surface body of exactly those faces, and every part says what became of it and why.
pub fn read_parts(id: &str, text: &str, heal: bool) -> Result<ReadOut, FeatureError> {
    if !text.contains("ISO-10303-21") {
        return Err(invalid("the data is not a STEP file (no ISO-10303-21 header)"));
    }
    let imp = ck_io::step::read_step(text).map_err(|e| {
        // the reader's own words; the hint about where the data goes would not help here
        let mut fe = FeatureError::new(IMPORT_STEP_INVALID).with("detail", Json::Str(e.message.clone())).with("reason", Json::Str(e.code.to_string()));
        if let Some(n) = e.entity {
            fe = fe.with("entity", Json::Num(n as f64));
        }
        fe
    })?;
    // the options of the `importStep` message: every fixer, and edges that deviate from their faces get a tolerance
    let opts = heal.then(|| {
        let mut o = ck_heal::HealOpts::new(1e-4);
        o.tolerant = true;
        o
    });
    let r = sort_step(imp, Some(id), opts.as_ref());
    if r.sorted.solid.num_faces() == 0 && r.sorted.surfaces.is_empty() {
        return Err(invalid("the file holds no solid (no MANIFOLD_SOLID_BREP)"));
    }
    Ok(r)
}

/// Sort what the STEP reader gave (`ck_heal::honest`), healing first with `heal` when given. With `rename`, faces are
/// renamed `<rename>.I<solid>F<n>` (the part's naming); without, they keep the reader's `I<solid>.F<n>`.
pub fn sort_step(imp: ck_io::step::Imported, rename: Option<&str>, heal: Option<&ck_heal::HealOpts>) -> ReadOut {
    use ck_heal::honest::{self, PartIn};
    let mut body = imp.body;
    ck_topo::region::rebuild_lumps(&mut body);
    let mut cap = ck_check::DEFAULT_TOL_CAP;
    if let Some(o) = heal {
        cap = o.cap;
        ck_heal::heal_with(&mut body, o);
        ck_topo::region::rebuild_lumps(&mut body);
    }
    // the reader names a face `I<solid>.F<n>`; the part's naming puts the feature first
    for fid in body.face_ids().collect::<Vec<_>>() {
        let Some(id) = rename else { break };
        let n = &body.face(fid).name;
        let solid = n.feature.strip_prefix('I').unwrap_or("1").to_string();
        let role = match &n.role {
            ck_topo::body::Role::Other(r) => r.clone(),
            _ => continue,
        };
        body.faces[fid.idx()].name = FaceName::other(id, &format!("I{solid}{role}"));
    }
    let parts: Vec<PartIn> = imp
        .parts
        .iter()
        .map(|p| {
            let mut reasons: Vec<honest::Reason> = p.refused.iter().map(|e| honest::refused_face(e.code, format!("#{}", e.entity.unwrap_or(0)), &e.message)).collect();
            if p.surface_model {
                reasons.push(honest::Reason { code: ck_heal::honest::SURFACE_MODEL.into(), faces: vec![], message: "the file gives this part as surfaces (a surface model), not as a solid".into() });
            }
            if let Some(e) = &p.error {
                reasons.push(honest::Reason { code: e.code.to_string(), faces: vec![], message: format!("the solid could not be read: {}", e.message) });
            }
            let label = if p.name.trim().is_empty() { format!("solid #{}", p.entity) } else { p.name.clone() };
            PartIn { label, faces: p.faces.clone(), faces_total: p.faces_total, reasons }
        })
        .collect();
    let left_out = imp
        .parts
        .iter()
        .map(|p| p.refused_surfaces.iter().map(|(f, s, sense)| ck_heal::repair::LeftOut { id: format!("#{f}"), surface: s.clone(), sense: *sense }).collect())
        .collect();
    ReadOut { sorted: honest::sort(&body, &parts, cap), left_out, solids: imp.solids }
}

/// What became of each part, as warning params: `[{label, outcome, faces, facesTotal, surface?, reasons:[{code, faces,
/// message}]}]`.
pub fn parts_json(id: &str, parts: &[ck_heal::honest::Part]) -> Json {
    Json::Arr(
        parts
            .iter()
            .enumerate()
            .map(|(k, p)| {
                let reasons = p
                    .reasons
                    .iter()
                    .map(|r| {
                        Json::Obj(vec![
                            ("code".into(), Json::Str(r.code.clone())),
                            ("faces".into(), Json::Arr(r.faces.iter().map(|f| Json::Str(f.clone())).collect())),
                            ("message".into(), Json::Str(r.message.clone())),
                        ])
                    })
                    .collect();
                let mut o = vec![
                    ("label".into(), Json::Str(p.label.clone())),
                    ("outcome".into(), Json::Str(p.outcome.as_str().into())),
                    ("faces".into(), Json::Num(p.faces as f64)),
                    ("facesTotal".into(), Json::Num(p.faces_total as f64)),
                ];
                if p.outcome == ck_heal::honest::Outcome::Surface {
                    o.push(("surface".into(), Json::Str(surface_id(id, k))));
                }
                o.push(("reasons".into(), Json::Arr(reasons)));
                Json::Obj(o)
            })
            .collect(),
    )
}

/// The warning an import with parts that are not solids carries (the "needs repair" mark).
pub const IMPORT_STEP_PARTS: &str = "importStepParts";

pub fn exec_import_step(br: &mut Brep, state: &mut RegenState, f: &Feature) -> Result<(), FeatureError> {
    let name = super::bodies::body_name(&f.raw)?;
    if let Some(n) = &name {
        super::bodies::check_free(br.body.as_ref(), n, &f.id, &[])?;
    }
    let heal = match f.raw.get("heal") {
        None => false,
        Some(Json::Bool(b)) => *b,
        Some(_) => return Err(invalid("`heal` must be true or false")),
    };
    let r = read_parts(&f.id, data(state, f)?, heal)?;
    let sorted = r.sorted;
    if sorted.solid.num_faces() > 0 {
        br.combine(&sorted.solid, Combine::Separate, false)?;
    }
    if let (Some(n), Some(b)) = (name, br.body.as_mut()) {
        let mut k = 0;
        let ls: Vec<_> = b.lump_ids().collect();
        for l in ls {
            if b.lump(l).name.is_none() && super::bodies::origin(b, l) == Some(f.id.as_str()) {
                k += 1;
                b.lumps[l.idx()].name = Some(if k == 1 { n.clone() } else { format!("{n} {k}") });
            }
        }
    }
    for (k, sb) in sorted.surfaces {
        br.set_surface(&surface_id(&f.id, k), sb);
    }
    if sorted.parts.iter().any(|p| p.outcome != ck_heal::honest::Outcome::Solid) {
        let n = |o| sorted.parts.iter().filter(|p| p.outcome == o).count() as f64;
        let (ns, total) = (sorted.parts.iter().filter(|p| p.outcome == ck_heal::honest::Outcome::Surface).count(), sorted.parts.len());
        let detail = format!("needs repair: {ns} of {total} part(s) of the file are not valid solids and came in as surface bodies of the faces that could be read; Repair Part (repairPart) can try to make them solids");
        let w = FeatureError::new(IMPORT_STEP_PARTS)
            .with("detail", Json::Str(detail))
            .with("solids", Json::Num(n(ck_heal::honest::Outcome::Solid)))
            .with("surfaces", Json::Num(n(ck_heal::honest::Outcome::Surface)))
            .with("refused", Json::Num(n(ck_heal::honest::Outcome::Refused)))
            .with("parts", parts_json(&f.id, &sorted.parts));
        state.result.warnings.push(crate::regen::RegenError { feature: f.id.clone(), error: w });
    }
    Ok(())
}
