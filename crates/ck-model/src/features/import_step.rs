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
fn data<'a>(state: &'a RegenState, f: &'a Feature) -> Result<&'a str, FeatureError> {
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

/// Read `text` into a body whose faces are named `<id>.I<solid>F<n>`; the number of solids with it.
pub fn read(id: &str, text: &str, heal: bool) -> Result<(ck_topo::body::Body, usize), FeatureError> {
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
    let mut body = imp.body;
    ck_topo::region::rebuild_lumps(&mut body);
    if heal {
        // the options of the `importStep` message: every fixer, and edges that deviate from their faces get a tolerance
        let mut opts = ck_heal::HealOpts::new(1e-4);
        opts.tolerant = true;
        ck_heal::heal_with(&mut body, &opts);
        ck_topo::region::rebuild_lumps(&mut body);
    }
    if body.num_faces() == 0 || imp.solids == 0 {
        return Err(invalid("the file holds no solid (no MANIFOLD_SOLID_BREP)"));
    }
    // the reader names a face `I<solid>.F<n>`; the part's naming puts the feature first
    for fid in body.face_ids().collect::<Vec<_>>() {
        let n = &body.face(fid).name;
        let solid = n.feature.strip_prefix('I').unwrap_or("1").to_string();
        let role = match &n.role {
            ck_topo::body::Role::Other(r) => r.clone(),
            _ => continue,
        };
        body.faces[fid.idx()].name = FaceName::other(id, &format!("I{solid}{role}"));
    }
    Ok((body, imp.solids))
}

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
    let (body, _) = read(&f.id, data(state, f)?, heal)?;
    br.combine(&body, Combine::Separate, false)?;
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
    Ok(())
}
