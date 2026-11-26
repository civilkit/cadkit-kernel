//! Geometry queries over a regenerated state: `measure`, `massProps`,
//! `interference`, `step`.
//!
//! Frontend messages (app.pretty.js):
//! - `{type:"measure", tag, refs:[{kind:"face",id}|{kind:"edge",faces,at}]}`
//!   -> `{items:[{area|wedge|...}]}`
//! - `{type:"massProps", tag:"main", bodies:true, densities:[...]}` — with
//!   `densities` every `bodies[]` entry also carries `mass`; `densities`
//!   alone implies `bodies:true` and must have one entry per lump.
//! - `{type:"interference", tag:"main"}` -> `{bodies:n, pairs:[...]}`
//! - `{type:"step", tag:"main", name}` -> STEP text (ck-io); no body: refused `stepNothingToExport`
//!
//! Pinned with an empty state (body null): `measure`, `massProps`, `interference` answer `null`; `step` is refused.
//! `massProps` / `interference` run inside the spans "mass properties" /
//! "interference between the bodies", opened BEFORE the state lookup.

use crate::session::Cx;
use ck_json::Json;

pub const MASS_LABEL: &str = "mass properties";
pub const INTERFERENCE_LABEL: &str = "interference between the bodies";

fn null_or_query(cx: &mut Cx, ty: &str) -> Result<Json, String> {
    if !cx.state()?.has_body {
        return Ok(Json::Null);
    }
    cx.query(ty)
}

pub fn measure(cx: &mut Cx) -> Result<Json, String> {
    null_or_query(cx, "measure")
}

/// `{type:"faceRun", tag, faces:[names]}`: area centre, outward normal and reach of flat faces, and the feature end each is
/// (see ck-model `press_pull::face_run`). Read-only. No body: null.
pub fn face_run(cx: &mut Cx) -> Result<Json, String> {
    null_or_query(cx, "faceRun")
}

pub fn mass_props(cx: &mut Cx) -> Result<Json, String> {
    cx.span(MASS_LABEL, |cx| null_or_query(cx, "massProps"))
}

pub fn interference(cx: &mut Cx) -> Result<Json, String> {
    cx.span(INTERFERENCE_LABEL, |cx| null_or_query(cx, "interference"))
}

pub fn validate(cx: &mut Cx) -> Result<Json, String> {
    null_or_query(cx, "validate")
}

/// `{type:"bodyBoolean", tag, op, target, tools, keepTools?, tolerance?, autoFuzzy?}`: the `combine` feature's
/// operation on a copy of the current body; the reply says what the result would be, nothing is committed.
pub fn body_boolean(cx: &mut Cx) -> Result<Json, String> {
    null_or_query(cx, "bodyBoolean")
}

/// `{type:"exportMesh", tag, quality?, chord_mm?}`: the triangle mesh of the current body at the tolerance an export asks for
/// (docs/spec-agent-interface.md "Export mesh"). Null when the tag holds no body.
pub fn export_mesh(cx: &mut Cx) -> Result<Json, String> {
    null_or_query(cx, "exportMesh")
}

pub fn heal(cx: &mut Cx) -> Result<Json, String> {
    null_or_query(cx, "heal")
}

/// `{"type":"importSat","tag":t,"name":n?,"data":"<SAT text>"}` -> regen-shaped
/// result (`errors`, `body`, ...). A failed import answers `body:null` with a
/// structured error and leaves the tag's state alone.
pub fn import_sat(cx: &mut Cx) -> Result<Json, String> {
    if cx.msg.get_str("data").is_none() {
        return Err("importSat: data (the SAT file text) is required".to_string());
    }
    let tag = cx.tag();
    let Some(h) = cx.hooks.import_sat else { return Err(crate::session::todo("importSat")) };
    let (reply, state) = h(cx)?;
    if let Some(s) = state {
        cx.session.put(&tag, s);
    }
    Ok(reply)
}

/// `{"type":"importSldprt","tag":t,"name":n?,"data":"<base64 of a .SLDPRT file>"}` -> regen-shaped result like
/// `importStep` (`body`, `errors`, plus `import`). A failed import answers `body:null` with a structured error and
/// leaves the tag's state alone.
pub fn import_sldprt(cx: &mut Cx) -> Result<Json, String> {
    if cx.msg.get_str("data").is_none() {
        return Err("importSldprt: data (the .SLDPRT file as base64) is required".to_string());
    }
    let tag = cx.tag();
    let Some(h) = cx.hooks.import_sldprt else { return Err(crate::session::todo("importSldprt")) };
    let (reply, state) = h(cx)?;
    if let Some(s) = state {
        cx.session.put(&tag, s);
    }
    Ok(reply)
}

/// `{"type":"sldprtHistory","name":n?,"data":"<base64 of a .SLDPRT file>"}` -> `{ok, part?, complete?, reason?, outcome?}`: the
/// part file's feature history (sketches, extrudes, fillets, ...) as a part document that regenerates to the body
/// `importSldprt` reads, or `ok:false` with the reason (a file that cannot be read back is not an error of the request:
/// the caller keeps the plain import). Read only: no tag, no state.
pub fn sldprt_history(cx: &mut Cx) -> Result<Json, String> {
    if cx.msg.get_str("data").is_none() {
        return Err("sldprtHistory: data (the .SLDPRT file as base64) is required".to_string());
    }
    let Some(h) = cx.hooks.sldprt_history else { return Err(crate::session::todo("sldprtHistory")) };
    h(cx)
}

/// `{"type":"sldasmTree","data":"<base64 of a .SLDASM file>","configuration":name?,"asPart":bool?,"mates":bool?}` -> `{ok, name, configuration,
/// configurations:[{name, active, id}], count, components:[{name, model, part, kind, configuration, transform, suppressed, hidden,
/// virtual, excludeFromBom, flexible, bbox, children}], notes, mates, matesStream, mateSummary, mateNotes, part?, mateMapping?}`: the assembly's component tree (nested sub-assemblies,
/// per-component placement in millimetres relative to the parent, referenced file names). With `asPart` also `part`, a
/// part document of `insertPart` features; `mates` (default on) lists the decoded mates with a geometric check (see book/src/io/sldprt.md). A file that is not a readable assembly answers
/// `{ok:false, code, reason}` (stable `asm.*` / `container.*` / `limit.*` codes). Read only: no tag, no state.
pub fn sldasm_tree(cx: &mut Cx) -> Result<Json, String> {
    if cx.msg.get_str("data").is_none() {
        return Err("sldasmTree: data (the .SLDASM file as base64) is required".to_string());
    }
    let Some(h) = cx.hooks.sldasm_tree else { return Err(crate::session::todo("sldasmTree")) };
    h(cx)
}

/// `{"type":"f3dFeatures","designs":[{label, dump, reference?, referenceError?}]}` (the `designs` of ck_f3d.wasm's `f3dDump`) ->
/// `{designs:[{label, part, mapped, placeholders, items, notes, check}]}`: each design's timeline as a ck-part feature list and the
/// regenerate-and-compare verdict against `reference` (docs/f3d-lazy.md). Read only: no tag, no state.
pub fn f3d_features(cx: &mut Cx) -> Result<Json, String> {
    let Some(h) = cx.hooks.f3d_features else { return Err(crate::session::todo("f3dFeatures")) };
    h(cx)
}

/// `{"type":"builtExport","feature":id?}` -> `{ok, entries:[{feature, key, source, rejected?, data}]}`: the saved results of the
/// import features (`importBody`) that the last regens built from their file or read from the document's `built` table,
/// oldest first (at most the last few). `data` is the base64 text to store as `built[key]` of the part document (or beside
/// it), compressed by the app if it likes before keeping and expanded again before it goes back in; `source` is `built` or
/// `saved`; `rejected` says why an entry under the key was present but not used. With `feature`, only that feature's.
/// Read only: no tag, no state. The entry is a cache: a document without it, or with a damaged or foreign one, rebuilds.
pub fn built_export(cx: &mut Cx) -> Result<Json, String> {
    let Some(h) = cx.hooks.built_export else { return Err(crate::session::todo("builtExport")) };
    h(cx)
}

/// `{"type":"importSkp","tag":t,"name":n?,"data":"<base64 of a .skp file>","opts":{"hidden":bool?,"maxFaces":n?}?}` ->
/// regen-shaped result like `importStep` (`body`, `errors`, plus `import`). The body is a surface model: one planar face per
/// SketchUp face (holes kept), vertices welded, edges shared; `import.faceMaterials` lists the colour index of each
/// `skp.F<n>` face. A failed import answers `body:null` with a structured error and leaves the tag's state alone.
pub fn import_skp(cx: &mut Cx) -> Result<Json, String> {
    if cx.msg.get_str("data").is_none() {
        return Err("importSkp: data (the .skp file as base64) is required".to_string());
    }
    let tag = cx.tag();
    let Some(h) = cx.hooks.import_skp else { return Err(crate::session::todo("importSkp")) };
    let (reply, state) = h(cx)?;
    if let Some(s) = state {
        cx.session.put(&tag, s);
    }
    Ok(reply)
}

/// `{"type":"importF3d","tag":t,"name":n?,"data":"<base64 of a .f3d / .f3z file or of one .smb / .smbh body blob>","opts":{"history":bool?}?}` ->
/// regen-shaped result like `importStep` (`body`, `surfaces`, `errors`, `warnings`, plus `import`). The valid closed solids of the file are the
/// tag's body (a lump each); every other ASM body (sheets, bodies with a face the reader cannot make) comes in as the surface body of the faces
/// that were read, with the reasons in `import.parts` and the `importStepParts` warning. `import.report` says in plain words what was read and
/// what was left out. A file that cannot be read answers `body:null` with a structured error and leaves the tag's state alone.
pub fn import_f3d(cx: &mut Cx) -> Result<Json, String> {
    if cx.msg.get_str("data").is_none() && cx.msg.get("decoded").is_none() {
        return Err("importF3d: data (the .f3d / .f3z file or .smb blob as base64) or decoded (the bodies ck_f3d.wasm decoded) is required".to_string());
    }
    let tag = cx.tag();
    let Some(h) = cx.hooks.import_f3d else { return Err(crate::session::todo("importF3d")) };
    let (reply, state) = h(cx)?;
    if let Some(s) = state {
        cx.session.put(&tag, s);
    }
    Ok(reply)
}

/// `{"type":"meshToSolid","tag":t,"name":n?,"positions":[..] | "positionsF32"|"positionsF64":"<base64>",
/// "indices":[..]? | "indicesU32":"<base64>"?, "opts":{..}?}` -> regen-shaped result like `importSkp` (`body`, `errors`,
/// plus `import` with the conversion report; docs/agent-queries.md). A refused mesh answers `body:null` with a structured
/// error (`meshNotWatertight`, `meshNonManifold`, `meshDegenerate`, `meshTooLarge`, `meshBadInput`) and leaves the tag alone.
pub fn mesh_to_solid(cx: &mut Cx) -> Result<Json, String> {
    let tag = cx.tag();
    let Some(h) = cx.hooks.mesh_to_solid else { return Err(crate::session::todo("meshToSolid")) };
    let (reply, state) = h(cx)?;
    if let Some(s) = state {
        cx.session.put(&tag, s);
    }
    Ok(reply)
}

/// `{"type":"meshDecimate","positions":[..] | "positionsF32"|"positionsF64":"<base64>","indices":[..]? | "indicesU32":"<base64>"?,
/// "opts":{"tol":mm,"targetTriangles":n?,"keepAngleDeg":a?,"weldTol":mm?}}` -> `{triangles, vertices, ..., deviation, positions,
/// indices}` (docs/agent-queries.md "meshDecimate"). Stateless: no tag. A refused mesh answers `errors: [["import", {code,
/// params:{message, hint, ..}}]]`.
pub fn mesh_decimate(cx: &mut Cx) -> Result<Json, String> {
    cx.query("meshDecimate")
}

/// The refusal of `step` when the document has no body at all (no solid, no surface).
pub const STEP_NOTHING_TO_EXPORT: &str = "stepNothingToExport: the document has no bodies to export. Hint: make a solid or a surface first (an extrude, a revolve, a surfaceExtrude, ...); suppressed features and an empty rollback give nothing to write";

/// `{type:"step", tag, name?}` -> the STEP text of the solid body and the surface bodies (each surface body a named
/// SHELL_BASED_SURFACE_MODEL). A document with no body is refused with `stepNothingToExport` (not `null`).
pub fn step(cx: &mut Cx) -> Result<Json, String> {
    if !cx.state()?.has_body {
        return Err(STEP_NOTHING_TO_EXPORT.to_string());
    }
    cx.query("step")
}

/// Extension `{type:"importStep", tag?, name?, data:"<STEP text>"}`: parse the
/// B-rep solids of a STEP file into the tag's state (same body tagging as
/// regen, so massProps / inspect / validate / mesh / step work on it).
pub fn import_step(cx: &mut Cx) -> Result<Json, String> {
    let data = cx.msg.get_str("data").ok_or_else(|| "importStep: data must be a STEP file as a string".to_string())?;
    let name = cx.msg.get_str("name").unwrap_or("part");
    let tag = cx.tag();
    let Some(h) = cx.hooks.import_step else { return Err(crate::session::todo("importStep")) };
    let (result, state) = h(cx, name, data)?;
    if let Some(st) = state {
        cx.session.put(&tag, st);
    }
    Ok(result)
}
