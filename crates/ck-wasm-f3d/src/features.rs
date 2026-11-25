//! `f3dDump`: the design dump (the `ck-f3d-dump` IR, as JSON) of each design of the file, with the volume and box of the body
//! the ASM reader builds for it. The timeline mapping itself (`ck-f3d-features`, which links the kernel's regen to find the
//! edges and faces of the history so far) runs in the MAIN kernel wasm as `f3dFeatures` (docs/f3d-lazy.md), not here.

use ck_json::{Json, Obj};
use ck_topo::body::Body;

fn array3(v: [f64; 3]) -> Json {
    Json::Arr(v.iter().map(|x| Json::Num(*x)).collect())
}

/// The volume (mm^3), box (mm) and lump count of the design bodies the ASM reader builds from a document.
fn measure_import(bytes: &[u8]) -> Result<Json, String> {
    std::panic::catch_unwind(|| {
        let imp = ck_asm::import::import_f3d(bytes, &ck_asm::import::ImportOptions::default()).map_err(|e| e.to_string())?;
        let sorted = imp.sort();
        let b: &Body = &sorted.solid;
        let solids = b.lump_ids().count();
        if solids == 0 {
            return Err("the file's design has no closed solid to compare with".to_string());
        }
        let volume = ck_topo::mass::mass_props(b).volume;
        let (lo, hi) = b.bbox();
        Ok(Obj::new().put("volume", volume).put("min", array3([lo.x, lo.y, lo.z])).put("max", array3([hi.x, hi.y, hi.z])).put("solids", solids as f64).build())
    })
    .map_err(|_| "reading the body panicked".to_string())?
}

/// The reference of each design, in the order `ck_f3d_design::decode_bytes` gives the designs: the documents of an `.f3z`
/// package are each read on their own, any other file is one document (the same rule as `ck_f3d_features::guard`).
fn measure_documents(bytes: &[u8]) -> Vec<Result<Json, String>> {
    let archive = if bytes.starts_with(b"PK") { ck_asm::zip::Archive::new(bytes).ok() } else { None };
    if let Some(archive) = archive {
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

/// `{designs: [{label, dump, reference, referenceError}]}`; `dump` is the design dump as JSON (null for a design without a
/// timeline segment), `reference` the measure of its imported body (null with `referenceError` when none could be built).
/// The caller sends these to the main kernel's `f3dFeatures`.
pub fn request(bytes: &[u8], name: &str) -> Result<Json, String> {
    let designs = match ck_f3d_design::decode_bytes(bytes.to_vec(), name) {
        Ok(d) => d,
        Err(e) => return Ok(crate::error_result("f3d.container", &e)),
    };
    let references = measure_documents(bytes);
    let mut out = vec![];
    for (i, d) in designs.into_iter().enumerate() {
        let dump = match d.dump {
            Some(dump) => ck_json::parse(&serde_json::to_string(&dump).map_err(|e| e.to_string())?).map_err(|e| format!("{e:?}"))?,
            None => Json::Null,
        };
        let (reference, error) = match references.get(i) {
            Some(Ok(m)) => (m.clone(), Json::Null),
            Some(Err(e)) => (Json::Null, Json::Str(e.clone())),
            None => (Json::Null, Json::Str("no imported body to compare with".into())),
        };
        out.push(Obj::new().put("label", d.label.as_str()).put("dump", dump).put("reference", reference).put("referenceError", error).build());
    }
    Ok(Obj::new().put("designs", Json::Arr(out)).build())
}
