//! `.f3d` bytes -> the bodies and the `import` report of the `importF3d` reply. Shared by the lazily loaded wasm
//! (`decodeF3d`) and by the inline `importF3d` of builds that link the decoder (ck-wasm feature `f3d`, the native
//! service), so both give the same reply.

use ck_json::{Json, Obj};
use ck_topo::body::Body;

pub struct Options {
    pub history: bool,
    pub all_bodies: bool,
}

/// A file that cannot be read: `code` is `f3d.container`, `f3d.asm`, `f3d.empty`.
pub struct DecodeError {
    pub code: String,
    pub message: String,
}

pub struct Decoded {
    /// The valid closed solids (a lump each).
    pub solid: Body,
    /// `("import.P<k>", surface body)` of every other ASM body.
    pub surfaces: Vec<(String, Body)>,
    /// The reply's `import` object (docs/f3d.md).
    pub info: Json,
}

pub fn decode(bytes: &[u8], name: &str, o: &Options) -> Result<Decoded, DecodeError> {
    let opts = ck_asm::import::ImportOptions { history: o.history, all_bodies: o.all_bodies, ..Default::default() };
    let imp = ck_asm::import::import(bytes, &opts).map_err(|e| DecodeError { code: e.code.to_string(), message: e.message })?;
    let sorted = imp.sort();
    let report = imp.report(&sorted);
    let parts = ck_model::features::import_step::parts_json("import", &sorted.parts);
    let count = |o: ck_heal::honest::Outcome| sorted.parts.iter().filter(|p| p.outcome == o).count() as f64;
    let (solids, surface_parts, refused) = (count(ck_heal::honest::Outcome::Solid), count(ck_heal::honest::Outcome::Surface), count(ck_heal::honest::Outcome::Refused));
    let blobs = imp
        .blobs
        .iter()
        .map(|b| {
            Obj::new()
                .put("entry", b.entry.as_str())
                .put("guid", b.guid.as_str())
                .put("history", b.history)
                .put("asmVersion", b.asm_version.as_str())
                .put("bodies", b.bodies as f64)
                .build()
        })
        .collect();
    // the faces the reader could not make, at most 200 (the reasons of each part carry them all, grouped)
    let skipped: Vec<Json> = imp
        .bodies
        .iter()
        .flat_map(|b| b.skipped.iter().map(move |s| (b, s)))
        .take(200)
        .map(|(b, s)| Obj::new().put("body", b.label.as_str()).put("face", format!("f3d.{}f{}", b.tag, s.face).as_str()).put("code", s.code).put("message", s.message.as_str()).build())
        .collect();
    let faces_total: usize = imp.bodies.iter().map(|b| b.faces_total).sum();
    let faces: usize = imp.bodies.iter().map(|b| b.faces_built).sum();
    let surfaces: Vec<(String, Body)> = sorted.surfaces.into_iter().map(|(k, b)| (ck_model::features::import_step::surface_id("import", k), b)).collect();
    let info = Obj::new()
        .put("format", "f3d")
        .put("name", name)
        .put("container", if ck_asm::import::is_container(bytes) { "zip" } else { "blob" })
        .put("unitScale", 10.0)
        .put("blobs", Json::Arr(blobs))
        .put("bodies", imp.bodies.len() as f64)
        .put("solids", solids)
        .put("surfaces", surface_parts)
        .put("refused", refused)
        .put("faces", faces as f64)
        .put("facesTotal", faces_total as f64)
        .put("parts", parts)
        .put("skippedFaces", Json::Arr(skipped))
        .put("report", Json::Arr(report.into_iter().map(Json::Str).collect()))
        .build();
    Ok(Decoded { solid: sorted.solid, surfaces, info })
}

/// The wire form of a [`Decoded`]: `{solid, surfaces:[[id, b64]], info}` with the bodies as base64 of the lossless
/// `ck_topo::codec` form (what `importF3d`'s `decoded` takes).
pub fn to_wire(d: &Decoded) -> Json {
    let surfaces = d.surfaces.iter().map(|(id, b)| Json::Arr(vec![Json::Str(id.clone()), Json::Str(crate::b64::encode(&ck_topo::codec::encode(b)))])).collect();
    Obj::new().put("solid", crate::b64::encode(&ck_topo::codec::encode(&d.solid)).as_str()).put("surfaces", Json::Arr(surfaces)).put("info", d.info.clone()).build()
}
