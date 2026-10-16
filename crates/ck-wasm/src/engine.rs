//! Composition root: installs the ck-model engine into ck-server's hooks.
//! ck-server stays geometry-free; everything engine-side is reached from here.

use ck_json::{Json, Obj};
use ck_model::features::{body_wire_with, Brep};
use ck_model::regen::{self, RegenOpts, SketchResult, Tracer};
use ck_model::sketch::offset::sketch_offset_json;
use ck_model::{Part, Sketch};
use ck_server::session::{regen_label, Cx, Hooks, State, SNAPSHOT_LABEL};
use ck_server::trace::{SpanId, Trace};
use crate::regen_cache;
use crate::view_cache;
use ck_topo::body::Body;

pub fn install() {
    // wall-clock budget of the measurement requests (`elapsedMs`, `truncated`)
    ck_ops::analysis::set_clock(ck_server::clock::now);
    ck_server::set_hooks(Hooks {
        import_sat: Some(import_sat_hook),
        import_sldprt: Some(import_sldprt_hook),
        regen: Some(regen_hook),
        import_step: Some(import_hook),
        query: Some(query_hook),
        sketch_offset: Some(sketch_offset_hook),
        match_edges: Some(match_edges_hook),
        ..Hooks::default()
    });
}

/// What a regen leaves behind for later requests on its tag.
pub struct Snapshot {
    pub body: Body,
    pub sketches: Vec<SketchResult>,
    /// Assembly state (components, mates) for `drag`.
    pub asm: ck_model::features::mate::Assembly,
    /// `heal` self-test memo: (tol, healthy at tol, volume) of `body`.
    pub heal_base: Option<(f64, bool, f64)>,
}

/// ck-server trace as a ck-model [`Tracer`].
pub(crate) struct TraceSink<'a> {
    trace: &'a mut Trace,
    ids: Vec<Option<SpanId>>,
}

impl<'a> TraceSink<'a> {
    pub(crate) fn new(trace: &'a mut Trace) -> Self {
        TraceSink { trace, ids: vec![] }
    }
}

impl Tracer for TraceSink<'_> {
    fn begin(&mut self, label: &str) -> usize {
        self.ids.push(Some(self.trace.begin(label)));
        self.ids.len() - 1
    }
    fn relabel(&mut self, id: usize, label: &str) {
        if let Some(s) = &self.ids[id] {
            self.trace.relabel(s, label);
        }
    }
    fn end(&mut self, id: usize) {
        if let Some(s) = self.ids[id].take() {
            self.trace.end(s);
        }
    }
}

/// [`Brep`] + the request's trace.
pub(crate) struct Exec<'a> {
    pub(crate) brep: Brep,
    pub(crate) sink: TraceSink<'a>,
    /// `opts.autoHeal`: tolerance to heal with after each feature that leaves
    /// validator issues (None = off, the default).
    pub(crate) auto_heal: Option<f64>,
}

impl regen::FeatureExec for Exec<'_> {
    fn tracer(&mut self) -> Option<&mut dyn Tracer> {
        Some(&mut self.sink)
    }
    fn face_count(&self) -> Option<usize> {
        self.brep.face_count()
    }
    fn face_frame(&mut self, state: &regen::RegenState, face: &str) -> Option<ck_core::linalg::Frame> {
        self.brep.face_frame(state, face)
    }
    fn edge_line(&mut self, state: &regen::RegenState, faces: &[String], at: Option<[f64; 3]>) -> Option<(ck_core::linalg::Vec3, ck_core::linalg::Vec3)> {
        self.brep.edge_line(state, faces, at)
    }
    fn edge_seg(&mut self, state: &regen::RegenState, faces: &[String], at: Option<ck_core::linalg::Vec3>) -> Option<ck_ops::sweep::PathSeg> {
        self.brep.edge_seg(state, faces, at)
    }
    fn vertex_point(&mut self, state: &regen::RegenState, faces: &[String], at: Option<ck_core::linalg::Vec3>) -> Option<ck_core::linalg::Vec3> {
        self.brep.vertex_point(state, faces, at)
    }
    fn face_surf(&mut self, state: &regen::RegenState, face: &str) -> Option<regen::FaceSurf> {
        self.brep.face_surf(state, face)
    }
    fn prepare(&mut self, state: &regen::RegenState, features: &[&ck_model::Feature]) {
        self.brep.prepare(state, features);
        self.brep.flush_spans(&mut self.sink);
    }
    fn save(&self, prev: Option<&ck_model::features::BrepSnap>) -> Option<ck_model::features::BrepSnap> {
        Some(self.brep.save(prev))
    }
    fn load(&mut self, snap: &ck_model::features::BrepSnap) {
        self.brep.load(snap)
    }
    fn exec(&mut self, state: &mut regen::RegenState, f: &ck_model::Feature) -> Result<(), regen::FeatureError> {
        let r = self.brep.exec(state, f);
        self.brep.flush_spans(&mut self.sink);
        if let (Ok(()), Some(tol)) = (&r, self.auto_heal) {
            if let Some(body) = self.brep.body.as_mut() {
                auto_heal(body, tol, f, state);
            }
        }
        r
    }
}

/// `regen`: parse, run the feature program (continuing from the longest
/// cached prefix, see regen_cache.rs), snapshot.
fn regen_hook(part: &Json, cx: &mut Cx) -> Result<(Json, State), String> {
    let part_json = part;
    let part = Part::from_json(part)?;
    let opts = RegenOpts::from_json(cx.msg.get("opts"));
    let n = opts.up_to.map_or(part.features.len(), |u| part.features.len().min(u as usize));
    let o = cx.msg.get("opts");
    let flag = |k: &str| o.map_or(false, |o| o.get_bool(k) == Some(true));
    let auto_heal = o.filter(|o| o.get_bool("autoHeal") == Some(true)).map(|o| o.get_f64("healTol").unwrap_or(1e-5));
    if let Some(cap) = o.and_then(|o| o.get_f64("cacheCap")).filter(|c| *c >= 0.0) {
        regen_cache::with(|c| c.set_cap(cap as usize));
        // the display caches scale with it (defaults 128 MB faces + 96 MB snapshots next to the 256 MB of regen states)
        view_cache::with(|c| c.set_cap(cap as usize / 8 * 3));
        ck_tess::mesh::set_face_cache_cap(cap as usize / 2);
    }
    // debug requests want the real per-feature spans, so they never replay
    let use_cache = !flag("noCache") && !cx.trace.enabled();
    ck_ops::intersect::ss::GRID_SEEDS.store(flag("ssiGridSeeds"), std::sync::atomic::Ordering::Relaxed);
    ck_ops::intersect::ss::ORIG_STEP.store(flag("ssiOrigStep"), std::sync::atomic::Ordering::Relaxed);
    let want_stats = flag("cacheStats");
    let run_n = regen::run_len(&part, &opts);
    let mut stats = (0usize, run_n); // (features reused, features in the run)
    let mut final_key: Option<regen_cache::Key> = None;
    let (mut result, body) = cx.span(regen_label(n), |cx| {
        let mut ex = Exec { brep: Brep::new(), sink: TraceSink { trace: &mut *cx.trace, ids: vec![] }, auto_heal };
        let mut resume = None;
        let mut rec = None;
        if use_cache && run_n > 0 {
            let ko = regen_cache::KeyOpts { sketch_status: opts.sketch_status, solve_sketches: opts.solve_sketches, auto_heal };
            let keys = regen_cache::chain_keys(part_json, &ko, run_n);
            final_key = keys.last().copied();
            let hit = regen_cache::with(|c| c.lookup(&keys));
            let mut chained = None;
            if let Some(h) = hit {
                ex.brep.load(&h.snap);
                stats.0 = h.next;
                chained = Some((h.result.clone(), h.snap));
                resume = Some(regen::Resume { next: h.next, result: h.result, axes: h.axes });
            }
            rec = Some(regen_cache::Recorder::new(keys, chained));
        }
        let ck = rec.as_mut().map(|r| r as &mut dyn regen::Checkpoint);
        let mut r = regen::regen_from(&part, &opts, &mut ex, resume, ck);
        r.assembly = ex.brep.finalize_assembly(flag("dofReport"));
        r.components = ex.brep.components_json();
        let rn = ex.brep.renorm_faces();
        let asm = ex.brep.asm.clone();
        let surfs = std::mem::take(&mut ex.brep.surfs);
        (r, (ex.brep.body, rn, asm, surfs))
    });
    let (body, renorm, asm, surfs) = body;
    let mut out = result.to_json();
    // the snapshot depends on the document only (see view_cache.rs), so the final chain key finds it again
    let view_key = if use_cache && run_n > 0 { final_key } else { None };
    let tess_before = ck_tess::mesh::face_cache_stats();
    let mut view_hit = false;
    // `noView`: the caller already holds the view of this very document (a preview it is now committing) and only wants the
    // kernel's tag state and the errors, so the snapshot (meshes, edges) is skipped and `body` / `surfaces` stay empty
    let lite = flag("noView");
    cx.span(SNAPSHOT_LABEL, |cx| {
        if lite {
            return;
        }
        let cached = view_key.and_then(|k| view_cache::with(|c| c.get(&k)));
        if let Some(v) = cached {
            view_hit = true;
            if let Some(bv) = &v.body {
                set_body(&mut out, attach_mesh(bv.json.clone(), &bv.mesh, cx));
            }
            let sj = v.surfaces.iter().map(|(id, bv)| Json::Arr(vec![Json::Str(id.clone()), attach_mesh(bv.json.clone(), &bv.mesh, cx)])).collect();
            set_key(&mut out, "surfaces", Json::Arr(sj));
            return;
        }
        let tess_of = |b: &Body| if use_cache { ck_tess::mesh::tessellate_memo(b) } else { ck_tess::mesh::tessellate(b) };
        let mut keep = view_cache::View { body: None, surfaces: vec![] };
        if let Some(b) = &body {
            let t = cx.trace.begin(String::new());
            let tess = tess_of(b);
            cx.trace.relabel(&t, format!("tessellate {} faces → {} triangles", b.num_faces(), tess.mesh.tri_face.len()));
            cx.trace.end(t);
            let json = body_static(b, &tess, &renorm);
            set_body(&mut out, attach_mesh(json.clone(), &tess.mesh, cx));
            keep.body = Some(view_cache::BodyView { json, mesh: tess.mesh });
        }
        // surface bodies follow the solid body (buffer order is pinned)
        let mut sj = vec![];
        for (id, sb) in &surfs {
            let t = tess_of(sb);
            let mut json = body_static(sb, &t, &(vec![(0, sb.faces.len())], vec![]));
            mark_surface_faces(&mut json);
            sj.push(Json::Arr(vec![Json::Str(id.clone()), attach_mesh(json.clone(), &t.mesh, cx)]));
            keep.surfaces.push((id.clone(), view_cache::BodyView { json, mesh: t.mesh }));
        }
        set_key(&mut out, "surfaces", Json::Arr(sj));
        if let Some(k) = view_key {
            view_cache::with(|c| c.put(k, keep));
        }
    });
    let tess_after = ck_tess::mesh::face_cache_stats();
    if want_stats {
        let (reused, total) = stats;
        let (entries, bytes, cap, st) = regen_cache::with(|c| (c.len(), c.bytes(), c.cap(), c.stats));
        let j = Obj::new()
            .put("enabled", use_cache)
            .put("reusedFeatures", reused as f64)
            .put("ranFeatures", (total - reused) as f64)
            .put("fullHit", use_cache && total > 0 && reused == total)
            .put("entries", entries as f64)
            .put("bytes", bytes as f64)
            .put("cap", cap as f64)
            .put("lookups", st.lookups as f64)
            .put("hits", st.hits as f64)
            .put("stored", st.stored as f64)
            .put("evicted", st.evicted as f64)
            .put("viewHit", view_hit)
            .put("tessFaceLookups", (tess_after.lookups - tess_before.lookups) as f64)
            .put("tessFaceHits", (tess_after.hits - tess_before.hits) as f64)
            .put("tessEntries", tess_after.entries as f64)
            .put("tessBytes", tess_after.bytes as f64)
            .put("viewEntries", view_cache::with(|c| c.len()) as f64)
            .put("viewBytes", view_cache::with(|c| c.bytes()) as f64)
            .build();
        if let Json::Obj(kv) = &mut out {
            kv.push(("cacheStats".into(), j));
        }
    }
    let state = match body {
        Some(body) => {
            let sketches = std::mem::take(&mut result.sketches);
            State { has_body: true, data: Some(Box::new(Snapshot { body, sketches, asm, heal_base: None })) }
        }
        None => State::empty(),
    };
    Ok((out, state))
}

/// Healing options of an import / heal request: `heal: "off"` disables it;
/// `heal: {tol, sew, fixSmall, fixOrientation, tolerant, cap}` or the flat
/// `healTolerance` (cap on edge / vertex tolerances, mm) tune it. `None` = off.
fn heal_opts(msg: &Json) -> Option<ck_heal::HealOpts> {
    let h = msg.get("heal");
    if matches!(h, Some(Json::Bool(false))) || h.and_then(Json::as_str) == Some("off") {
        return None;
    }
    let o = h.filter(|h| h.is_obj());
    let f = |k: &str| o.and_then(|o| o.get_f64(k));
    let b = |k: &str| o.and_then(|o| o.get_bool(k)).unwrap_or(true);
    let mut ho = ck_heal::HealOpts::new(f("tol").unwrap_or(1e-4));
    ho.sew = b("sew");
    ho.fix_small = b("fixSmall");
    ho.fix_orientation = b("fixOrientation");
    ho.tolerant = b("tolerant");
    ho.cap = f("cap").or_else(|| msg.get_f64("healTolerance")).unwrap_or(ck_check::DEFAULT_TOL_CAP);
    Some(ho)
}

/// `importStep`: STEP text -> the tag's body (regen-style result: `body`,
/// `errors` as `["import", {code, params:{entity, message}}]`, plus
/// `import:{solids, faces, unitScale, warnings}`).
fn import_hook(cx: &mut Cx, _name: &str, data: &str) -> Result<(Json, Option<State>), String> {
    let mut out = ck_server::session::empty_regen_result();
    let err_json = |e: &ck_io::step::ImportError| {
        let mut params = Obj::new().put("message", e.message.as_str());
        if let Some(id) = e.entity {
            params = params.put("entity", id as f64);
        }
        Json::Arr(vec![Json::Str("import".into()), Obj::new().put("code", e.code).put("params", params.build()).build()])
    };
    match ck_io::step::read_step(data) {
        Err(e) => {
            set_key(&mut out, "errors", Json::Arr(vec![err_json(&e)]));
            Ok((out, None))
        }
        Ok(imp) => {
            let mut body = imp.body;
            ck_topo::region::rebuild_lumps(&mut body);
            let heal_info = heal_opts(cx.msg).map(|ho| {
                let rep = ck_heal::heal_with(&mut body, &ho);
                ck_topo::region::rebuild_lumps(&mut body);
                Obj::new()
                    .put("fixed", rep.fixed.len() as f64)
                    .put("remaining", rep.remaining.len() as f64)
                    .put("toleranceUsed", ck_check::tolerance_used(&body))
                    .build()
            });
            let tess = ck_tess::mesh::tessellate(&body);
            set_body(&mut out, body_json(&body, &tess, cx, &(vec![], vec![])));
            let info = Obj::new()
                .put("solids", imp.solids as f64)
                .put("faces", body.num_faces() as f64)
                .put("unitScale", imp.length_scale)
                .put("warnings", Json::Arr(imp.warnings.iter().map(err_json).collect()))
                .build();
            let mut info = info;
            if let (Json::Obj(kv), Some(h)) = (&mut info, heal_info) {
                kv.push(("heal".into(), h));
            }
            if let Json::Obj(kv) = &mut out {
                kv.push(("import".into(), info));
            }
            let snap = Snapshot { body, sketches: vec![], asm: Default::default(), heal_base: None };
            Ok((out, Some(State { has_body: true, data: Some(Box::new(snap)) })))
        }
    }
}

/// Standard base64 (also accepts url-safe letters, whitespace and a `data:...;base64,` prefix).
fn base64_decode(s: &str) -> Result<Vec<u8>, String> {
    let s = s.find("base64,").map_or(s, |i| &s[i + 7..]);
    let mut out = Vec::with_capacity(s.len() / 4 * 3);
    let (mut acc, mut nb) = (0u32, 0u32);
    for c in s.bytes() {
        let v = match c {
            b'A'..=b'Z' => c - b'A',
            b'a'..=b'z' => c - b'a' + 26,
            b'0'..=b'9' => c - b'0' + 52,
            b'+' | b'-' => 62,
            b'/' | b'_' => 63,
            b'=' | b' ' | b'\n' | b'\r' | b'\t' => continue,
            _ => return Err(format!("importSldprt: data is not base64 (byte 0x{c:02x})")),
        };
        acc = (acc << 6) | v as u32;
        nb += 6;
        if nb >= 8 {
            nb -= 8;
            out.push((acc >> nb) as u8);
            acc &= (1 << nb) - 1;
        }
    }
    Ok(out)
}

/// `importSldprt`: base64 of a SolidWorks part file -> the tag's body (result shaped like `importStep`: `body`,
/// `errors` as `["import", {code, params:{message}}]`, plus `import:{format, solids, faces, unitScale, container,
/// schema, approximations, notes}`).
fn import_sldprt_hook(cx: &mut Cx) -> Result<(Json, Option<State>), String> {
    let data = cx.msg.get_str("data").unwrap_or("").to_string();
    let name = cx.msg.get_str("name").unwrap_or("part").to_string();
    let mut out = ck_server::session::empty_regen_result();
    let bytes = base64_decode(&data)?;
    match ck_sldprt::import_sldprt(&bytes) {
        Err(e) => {
            let err = Json::Arr(vec![Json::Str("import".into()), Obj::new().put("code", e.code).put("params", Obj::new().put("message", e.message.as_str()).build()).build()]);
            set_key(&mut out, "errors", Json::Arr(vec![err]));
            Ok((out, None))
        }
        Ok(imp) => {
            let mut body = imp.body;
            ck_topo::region::rebuild_lumps(&mut body);
            let tess = ck_tess::mesh::tessellate(&body);
            set_body(&mut out, body_json(&body, &tess, cx, &(vec![], vec![])));
            let approx = imp.approx.iter().map(|a| Obj::new().put("what", a.what.as_str()).put("maxErrorMm", a.max_err_mm).build()).collect();
            // the mass properties SolidWorks stored in the file (mm^3, mm^2, kg), an independent check of the body
            let sw = match &imp.sw_mass {
                Some(m) => Obj::new()
                    .put("volumeMm3", m.volume_m3 * 1e9)
                    .put("areaMm2", m.area_m2 * 1e6)
                    .put("massKg", m.mass_kg)
                    .put("comMm", Json::Arr(m.com_m.iter().map(|x| Json::Num(x * 1e3)).collect()))
                    .build(),
                None => Json::Null,
            };
            let info = Obj::new()
                .put("swMassProps", sw)
                .put("format", "sldprt")
                .put("name", name.as_str())
                .put("solids", imp.solids as f64)
                .put("faces", body.num_faces() as f64)
                .put("unitScale", imp.unit_scale)
                .put("container", if imp.container == ck_sldprt::Kind::Modern { "modern" } else { "ole2" })
                .put("schema", imp.schema.as_str())
                .put("stream", imp.stream.as_str())
                .put("nodes", imp.nodes as f64)
                .put("approximations", Json::Arr(approx))
                .put("notes", Json::Arr(imp.notes.iter().map(|n| Json::Str(n.clone())).collect()))
                .build();
            if let Json::Obj(kv) = &mut out {
                kv.push(("import".into(), info));
            }
            let snap = Snapshot { body, sketches: vec![], asm: Default::default(), heal_base: None };
            Ok((out, Some(State { has_body: true, data: Some(Box::new(snap)) })))
        }
    }
}

/// `importSat`: ACIS SAT text -> body snapshot. Reply = regen-shaped result
/// (`errors`, `body`, ...) plus `import` {format, name, units, sheet, faces}.
fn import_sat_hook(cx: &mut Cx) -> Result<(Json, Option<State>), String> {
    let data = cx.msg.get_str("data").unwrap_or("").to_string();
    let name = cx.msg.get_str("name").unwrap_or("part").to_string();
    let mut out = ck_server::session::empty_regen_result();
    match ck_io::sat::import_sat(&data) {
        Err(e) => {
            let err = Obj::new()
                .put("code", e.code)
                .put("entity", e.entity_id().map_or(Json::Null, Json::Str))
                .put("kind", e.kind.as_str())
                .put("line", e.line as f64)
                .put("message", e.detail.as_str())
                .build();
            set_key(&mut out, "errors", Json::Arr(vec![err]));
            Ok((out, None))
        }
        Ok(imp) => {
            let body = imp.body;
            let sm = &imp.summary;
            let skipped = sm.skipped.iter().fold(Obj::new(), |o, (k, n)| o.put(k.as_str(), *n as f64)).build();
            let tess = ck_tess::mesh::tessellate(&body);
            let renorm = (vec![], vec![]);
            set_body(&mut out, body_json(&body, &tess, cx, &renorm));
            if let Json::Obj(kv) = &mut out {
                kv.push((
                    "import".into(),
                    Obj::new()
                        .put("format", "sat")
                        .put("name", name.as_str())
                        .put("units", imp.units)
                        .put("sheet", imp.sheet)
                        .put("faces", body.num_faces() as f64)
                        .put("version", sm.version as f64)
                        .put("records", sm.records as f64)
                        .put("bodies", sm.bodies as f64)
                        .put("lumps", sm.lumps as f64)
                        .put("shells", sm.shells as f64)
                        .put("skipped", skipped)
                        .build(),
                ));
            }
            let state = State { has_body: true, data: Some(Box::new(Snapshot { body, sketches: vec![], asm: Default::default(), heal_base: None })) };
            Ok((out, Some(state)))
        }
    }
}

fn set_body(result: &mut Json, body: Json) {
    set_key(result, "body", body)
}

/// Faces of a surface body carry `meta.surface: true` (oracle).
fn mark_surface_faces(body: &mut Json) {
    let Json::Obj(kv) = body else { return };
    let Some((_, Json::Arr(faces))) = kv.iter_mut().find(|(k, _)| k == "faces") else { return };
    for f in faces {
        let Json::Obj(fk) = f else { continue };
        if let Some((_, Json::Obj(meta))) = fk.iter_mut().find(|(k, _)| k == "meta") {
            meta.push(("surface".into(), Json::Bool(true)));
        }
    }
}

fn set_key(result: &mut Json, key: &str, v: Json) {
    if let Json::Obj(kv) = result {
        if let Some(slot) = kv.iter_mut().find(|(k, _)| k == key) {
            slot.1 = v;
        }
    }
}

/// Wire `body`: `{box, faces, edges, silhouette, mesh}`. Buffer creation
/// order is pinned (positions 0, normals 1, mids 2, triFace 3, silhouette 4,
/// silhouette serialised first). GUESS: until ck-tess lands every mesh
/// buffer is EMPTY (the client gets a valid but invisible mesh; the oracle
/// sends 3 f32 per vertex in positions/normals/mids and one i32 face index
/// per triangle in triFace, non-indexed triangle soup).
fn body_json(b: &Body, tess: &ck_tess::mesh::Tess, cx: &mut Cx, renorm: &(Vec<(usize, usize)>, Vec<(usize, usize)>)) -> Json {
    attach_mesh(body_static(b, tess, renorm), &tess.mesh, cx)
}

/// The mesh buffers of a body: `silhouette` and `mesh`. Buffer creation order is pinned (positions, normals, mids,
/// triFace, silhouette), and the `body` buffers come before the `surfaces` ones.
fn attach_mesh(mut body: Json, m: &ck_tess::mesh::Mesh, cx: &mut Cx) -> Json {
    let positions = cx.bufs.f32(&m.positions);
    let normals = cx.bufs.f32(&m.normals);
    let mids = cx.bufs.f32(&m.mids);
    let tri_face = cx.bufs.i32(&m.tri_face);
    let silhouette = cx.bufs.f32(&m.silhouette);
    if let Json::Obj(kv) = &mut body {
        kv.push(("silhouette".into(), silhouette));
        kv.push((
            "mesh".into(),
            Obj::new().put("positions", positions).put("normals", normals).put("mids", mids).put("triFace", tri_face).build(),
        ));
    }
    body
}

/// Everything of the wire body except the mesh buffers.
fn body_static(b: &Body, tess: &ck_tess::mesh::Tess, renorm: &(Vec<(usize, usize)>, Vec<(usize, usize)>)) -> Json {
    let m = &tess.mesh;
    let mut kv = body_wire_with(b, &|f| renorm.0.iter().any(|r| (r.0..r.1).contains(&f.idx())));
    // The oracle box is the union of the f32 display-mesh positions and the
    // f64 edge polyline points (pinned: brep cyl/zeroDepth/rev90 boxes take
    // each coordinate from whichever source is more extreme).
    let mut bounds = m.bbox();
    for e in b.wire_edges() {
        for p in tess.edge_polyline(b, e) {
            bounds = Some(match bounds {
                None => (p, p),
                Some((lo, hi)) => (
                    ck_core::linalg::Vec3::new(lo.x.min(p.x), lo.y.min(p.y), lo.z.min(p.z)),
                    ck_core::linalg::Vec3::new(hi.x.max(p.x), hi.y.max(p.y), hi.z.max(p.z)),
                ),
            });
        }
    }
    if let Some((lo, hi)) = bounds {
        let v = |p: ck_core::linalg::Vec3| Json::Arr(vec![Json::Num(p.x), Json::Num(p.y), Json::Num(p.z)]);
        kv[0].1 = Obj::new().put("min", v(lo)).put("max", v(hi)).build();
    }
    if let Some(slot) = kv.iter_mut().find(|(k, _)| k == "edges") {
        slot.1 = b.edges_json(&|b, e| tess.edge_polyline(b, e));
        ck_model::features::renorm_edges(b, &mut slot.1, &|e| renorm.1.iter().any(|r| (r.0..r.1).contains(&e.idx())));
    }
    Json::Obj(kv)
}

/// Opt-in repair after a feature: keep the healed body only when it has
/// fewer validator issues, and record a `healed` warning on the feature.
fn auto_heal(body: &mut Body, tol: f64, f: &ck_model::Feature, state: &mut regen::RegenState) {
    let before = ck_check::validate(body, tol).len();
    if before == 0 {
        return;
    }
    let mut c = body.clone();
    let rep = ck_heal::heal(&mut c, tol);
    if rep.fixed.is_empty() || rep.remaining.len() >= before {
        return;
    }
    *body = c;
    let mut codes: Vec<&str> = rep.fixed.iter().map(|x| x.code).collect();
    codes.dedup();
    let err = regen::FeatureError::new("healed")
        .with("fixed", Json::Num(rep.fixed.len() as f64))
        .with("codes", Json::Arr(codes.into_iter().map(|c| Json::Str(c.into())).collect()))
        .with("remaining", Json::Num(rep.remaining.len() as f64));
    state.result.put_error(regen::RegenError { feature: f.id.clone(), error: err });
}

/// `heal`: repair the current body in place (`dryRun` leaves it alone). The
/// optional `damage` kind (see `ck_heal::damage::KINDS`) breaks a COPY first
/// and reports whether healing restored it: the `heal.repair` gate metric.
fn heal_query(cx: &mut Cx) -> Result<Json, String> {
    let tol = cx.msg.get_f64("tol").unwrap_or(1e-5);
    let damage = cx.msg.get_str("damage").map(str::to_string);
    let dry = cx.msg.get_bool("dryRun") == Some(true);
    let tag = cx.tag();
    let snap = cx
        .session
        .get_mut(&tag)
        .and_then(|s| s.data.as_mut())
        .and_then(|d| d.downcast_mut::<Snapshot>())
        .ok_or_else(|| ck_server::session::todo("state without a snapshot"))?;
    let mut b = snap.body.clone();
    let (healthy, v0) = match snap.heal_base {
        Some((t, h, v)) if t == tol => (h, v),
        _ => {
            let base = (ck_check::validate(&b, tol).is_empty(), ck_topo::mass::mass_props(&b).volume);
            snap.heal_base = Some((tol, base.0, base.1));
            base
        }
    };
    // self-test only on bodies that are healthy at the healing tolerance
    let applied = damage.as_deref().map(|k| healthy && ck_heal::damage::apply(&mut b, k, 2e-6));
    let mut ho = heal_opts(cx.msg).unwrap_or_else(|| ck_heal::HealOpts::new(tol));
    if cx.msg.get_f64("tol").is_some() || cx.msg.get("heal").and_then(|h| h.get_f64("tol")).is_none() {
        ho.tol = tol;
    }
    let tol = ho.tol;
    ho.tolerant = ho.tolerant && damage.is_none();
    let rep = ck_heal::heal_with(&mut b, &ho);
    let v1 = ck_topo::mass::mass_props(&b).volume;
    let obj = |i: &ck_check::Issue| Obj::new().put("code", i.code).put("entity", i.entity.as_str()).put("detail", i.detail.as_str()).build();
    let fixed = rep.fixed.iter().map(|x| Obj::new().put("code", x.code).put("entity", x.entity.as_str()).put("detail", x.detail.as_str()).build()).collect();
    let remaining = rep.remaining.iter().map(obj).collect();
    let mut o = Obj::new()
        .put("fixed", Json::Arr(fixed))
        .put("remaining", Json::Arr(remaining))
        .put("volume", Obj::new().put("before", v0).put("after", v1).build());
    match &damage {
        Some(k) => {
            let clean = applied == Some(true)
                && (v1 - v0).abs() <= 1e-6 * v0.abs()
                && (if tol <= 1e-6 { rep.remaining.is_empty() } else { ck_check::validate(&b, 1e-6).is_empty() });
            o = o.put("damage", Obj::new().put("kind", k.as_str()).put("applied", applied == Some(true)).put("clean", clean).build());
        }
        None if !dry => {
            snap.body = b;
            snap.heal_base = None;
        }
        None => {}
    }
    Ok(o.build())
}

fn snapshot<'a>(cx: &'a Cx) -> Result<&'a Snapshot, String> {
    cx.state()?
        .data
        .as_ref()
        .and_then(|d| d.downcast_ref::<Snapshot>())
        .ok_or_else(|| ck_server::session::todo("state without a snapshot"))
}

/// Body-bearing requests.
fn query_hook(ty: &str, cx: &mut Cx) -> Result<Json, String> {
    match ty {
        "massProps" => {
            let s = snapshot(cx)?;
            let bodies = cx.msg.get("bodies").map_or(false, |b| matches!(b, Json::Bool(true)));
            let ds: Option<Vec<f64>> =
                cx.msg.get_arr("densities").map(|a| a.iter().map(|d| d.as_f64().unwrap_or(1.0)).collect());
            // Pinned: `byMass` only with `bodies:true` AND one density per lump.
            let ds = ds.filter(|d| bodies && d.len() == s.body.num_lumps());
            Ok(ck_topo::mass::mass_props_json(&s.body, bodies, ds.as_deref()))
        }
        "validate" => {
            let tol = cx.msg.get_f64("tol").unwrap_or(1e-6);
            let cap = cx.msg.get_f64("healTolerance").unwrap_or(ck_check::DEFAULT_TOL_CAP);
            let body = &snapshot(cx)?.body;
            let mut iss = ck_check::validate_cap(body, tol, cap);
            // an independent witness that the B-rep claim holds: the display mesh of a valid solid is watertight (only in
            // the validate request with `mesh:true` (default off: a few existing bodies have faces the tessellator gives up on, which would flip their verdict), never in regen)
            if cx.msg.get_bool("mesh") == Some(true) {
                let mesh = ck_tess::mesh::tessellate_memo(body).mesh;
                iss.extend(ck_check::tessellation::check_mesh(body, &mesh.positions, &mesh.tri_face, &mesh.failed));
            }
            let arr = iss
                .iter()
                .map(|i| Obj::new().put("code", i.code).put("entity", i.entity.as_str()).put("detail", i.detail.as_str()).build())
                .collect();
            let used = ck_check::tolerance_used(body);
            let mut o = Obj::new().put("ok", iss.is_empty()).put("issues", Json::Arr(arr)).put("toleranceUsed", used);
            if used > 0.0 {
                let note = format!("body carries edge/vertex tolerances up to {used:.3e} mm (cap {cap:.3e})");
                o = o.put("notes", Json::Arr(vec![Json::Str(note)]));
            }
            Ok(o.build())
        }
        "heal" => heal_query(cx),
        "check" => {
            let body = &snapshot(cx)?.body;
            let tol = cx.msg.get_f64("tol").unwrap_or(1e-6);
            Ok(ck_ops::agent::check(body, &cx.msg, &|| ck_check::validate(body, tol).len()))
        }
        "section" => ck_ops::section::section_reply(&snapshot(cx)?.body, cx.msg),
        "sections" => {
            let body = &snapshot(cx)?.body;
            // `exact:false` skips the (slow on big bodies) massProps cross-check
            let exact = (cx.msg.get_bool("exact") != Some(false)).then(|| ck_topo::mass::mass_props(body).volume);
            ck_ops::section::sections_reply(body, cx.msg, exact)
        }
        "probe" => ck_ops::section::probe_reply(&snapshot(cx)?.body, cx.msg),
        "recognize" => recognize_query(cx),
        "distance" | "maxDistance" => {
            let body = &snapshot(cx)?.body;
            // `tagB`: measure against the body of another tag (a rebuilt part against the original)
            let other = match cx.msg.get_str("tagB") {
                Some(t) => Some(
                    &cx.session
                        .get(t)
                        .and_then(|s| s.data.as_ref())
                        .and_then(|d| d.downcast_ref::<Snapshot>())
                        .ok_or_else(|| format!("{ty}: tagB {t:?} has no body"))?
                        .body,
                ),
                None => None,
            };
            if ty == "distance" {
                ck_ops::analysis::distance_reply(body, other, cx.msg)
            } else {
                ck_ops::analysis::max_distance_reply(body, other, cx.msg)
            }
        }
        "thickness" => ck_ops::analysis::thickness_reply(&snapshot(cx)?.body, cx.msg),
        "thicknessAt" => ck_ops::analysis::thickness_at_reply(&snapshot(cx)?.body, cx.msg),
        "closestPoint" => ck_ops::analysis::closest_point_reply(&snapshot(cx)?.body, cx.msg),
        "angles" => ck_ops::analysis::angles_reply(&snapshot(cx)?.body, cx.msg),
        "clearance" => ck_ops::analysis::clearance_reply(&snapshot(cx)?.body, cx.msg),
        "symmetry" => ck_ops::analysis::symmetry_reply(&snapshot(cx)?.body, cx.msg),
        "resolve" => Ok(ck_ops::agent::resolve(&snapshot(cx)?.body, &cx.msg)),
        "describe" => Ok(ck_ops::agent::describe(&snapshot(cx)?.body, &cx.msg)),
        "refDiff" => crate::refdiff::reply(cx),
        "profiles" => Ok(crate::profile_q::profiles()),
        "checkProfile" => {
            if cx.msg.get("part").is_some() {
                crate::profile_q::check_profile(cx, None)
            } else {
                crate::profile_q::check_profile(cx, Some(&snapshot(cx)?.body))
            }
        }
        "threadData" => {
            let get = |k: &str| match cx.msg.get(k) {
                Some(Json::Str(s)) if !s.is_empty() => Ok(Some(s.as_str())),
                None | Some(Json::Null) => Ok(None),
                Some(_) => Err(format!("threadData: {k} must be a string")),
            };
            ck_model::threads::reply(get("standard")?, get("size")?)
        }
        "inspect" => inspect(cx),
        "continuity" => Ok(ck_ops::continuity::reply(&snapshot(cx)?.body, &cx.msg)),
        "measure" => Ok(ck_ops::measure::reply(&snapshot(cx)?.body, &cx.msg)),
        "drag" => {
            let s = snapshot(cx)?;
            Ok(crate::drag::reply(&s.body, &s.asm, &cx.msg))
        }
        "interference" => Ok(ck_ops::interference::reply(&snapshot(cx)?.body)),
        "bodyBoolean" => Ok(ck_model::features::combine::preview(&snapshot(cx)?.body, cx.msg)),
        "comb" => Ok(comb(&snapshot(cx)?.body, &cx.msg)),
        "curvatureMap" => {
            let body = &snapshot(cx)?.body;
            let tess = ck_tess::mesh::tessellate(body);
            let m = ck_tess::curvmap::curvature_map(body, &tess);
            let ids: Vec<Json> = m.faces.iter().map(|&f| Json::Str(body.face_id_str(f))).collect();
            let failed: Vec<Json> = body.face_ids().enumerate().filter(|(i, _)| m.failed.contains(i)).map(|(_, f)| Json::Str(body.face_id_str(f))).collect();
            let mesh = Obj::new()
                .put("positions", cx.bufs.f32(&m.positions))
                .put("normals", cx.bufs.f32(&m.normals))
                .put("k", cx.bufs.f32(&m.k))
                .put("triFace", cx.bufs.i32(&m.tri_face))
                .build();
            let b = Obj::new().put("id", Json::Null).put("faces", Json::Arr(ids)).put("mesh", mesh).build();
            Ok(Obj::new().put("bodies", Json::Arr(vec![b])).put("failed", Json::Arr(failed)).build())
        }
        "convert" => {
            let s = snapshot(cx)?;
            Ok(match cx.msg.get("ref").and_then(|r| r.get_str("kind")) {
                Some("sketchCurve") => crate::sketch_q::convert(&s.sketches, &cx.msg),
                _ => ck_ops::convert_proj::reply(&s.body, &cx.msg),
            })
        }
        "refCurve" => {
            let s = snapshot(cx)?;
            let r = cx.msg.get("ref").unwrap_or(&Json::Null);
            Ok(match r.get_str("kind") {
                Some("edge") => ck_ops::ref_curve::edge_reply(&s.body, r),
                Some("sketchCurve") => crate::sketch_q::ref_curve(&s.sketches, r),
                _ => ck_ops::ref_curve::missing_reference(),
            })
        }
        "curvatureAt" => Ok(ck_ops::curvature_at::reply(&snapshot(cx)?.body, &cx.msg)),
        "drawing" => {
            let s = snapshot(cx)?;
            Ok(ck_ops::drawing::reply(&s.body, &cx.msg))
        }
        "step" => {
            // Reply: the whole STEP file as one JSON string (oracle); `name`
            // defaults to "part". Void shells must be split out first.
            let name = cx.msg.get_str("name").unwrap_or("part").to_string();
            let mut body = snapshot(cx)?.body.clone();
            ck_topo::region::rebuild_lumps(&mut body);
            let opts = ck_io::step::StepOpts { unix_ms: ck_server::clock::unix_ms() };
            Ok(Json::Str(ck_io::step::write_step(&body, &name, &opts)))
        }
        _ => Err(ck_server::session::todo(ty)),
    }
}

/// `inspect` with a body: only `sketch` refs so far.
fn inspect(cx: &mut Cx) -> Result<Json, String> {
    let r = cx.msg.get("ref");
    let kind = r.and_then(|r| r.get_str("kind")).unwrap_or("");
    match kind {
        "sketch" => {
            let s = snapshot(cx)?;
            let id = r.and_then(|r| r.get_str("id")).unwrap_or("");
            let sk = s.sketches.iter().find(|k| k.id == id).ok_or_else(|| "sketch not found".to_string())?;
            let areas: Vec<Json> = sk.profiles.regions.iter().map(|g| Json::Num(region_area(g))).collect();
            Ok(Obj::new()
                .put("type", "sketch")
                .put("frame", regen::frame_json(&sk.frame))
                .put("regions", sk.profiles.regions.len())
                .put("areas", Json::Arr(areas))
                .put("open", sk.profiles.open)
                .build())
        }
        k => Err(ck_server::session::todo(&format!("inspect {k}"))),
    }
}

/// Region area (outer + signed holes): Green's theorem on the exact
/// segment curves (8 × 8-point Gauss-Legendre per segment).
/// GUESS: last-ulp rounding vs the oracle (40×20 -> 800.0000000000001).
fn region_area(r: &ck_model::sketch::profile::Region) -> f64 {
    const X: [f64; 4] = [0.1834346424956498, 0.5255324099163290, 0.7966664774136267, 0.9602898564975363];
    const W: [f64; 4] = [0.3626837833783620, 0.3137066458778873, 0.2223810344533745, 0.1012285362903763];
    let lp = |l: &Vec<ck_model::sketch::profile::Seg>| -> f64 {
        let mut a = 0.0;
        for s in l {
            let g = &s.curve.geom;
            let (ta, tb) = (s.curve.ta, s.curve.tb);
            let k = 8;
            for i in 0..k {
                let (p, q) = (ta + (tb - ta) * i as f64 / k as f64, ta + (tb - ta) * (i + 1) as f64 / k as f64);
                let (m, h) = ((p + q) / 2.0, (q - p) / 2.0);
                for (x, w) in X.iter().zip(W) {
                    for t in [m - h * x, m + h * x] {
                        let (pt, d) = (g.eval(t), g.d1(t));
                        a += 0.5 * w * h * (pt[0] * d[1] - pt[1] * d[0]);
                    }
                }
            }
        }
        a
    };
    lp(&r.outer) + r.holes.iter().map(lp).sum::<f64>()
}

/// `sketchOffset {sketch, ids, distance}`.
fn sketch_offset_hook(cx: &mut Cx, distance: f64) -> Result<Json, String> {
    let sk = Sketch::from_json(cx.msg.get("sketch").unwrap_or(&Json::Null));
    let ids: Vec<String> = cx
        .msg
        .get_arr("ids")
        .map(|a| a.iter().filter_map(|j| j.as_str().map(str::to_string)).collect())
        .unwrap_or_default();
    Ok(sketch_offset_json(&sk, &ids, distance))
}

/// `comb {refs, density, lines, dir}`: one entry per ref.
fn comb(body: &Body, msg: &Json) -> Json {
    use ck_ops::measure::{resolve_ref, Ref};
    // oracle clamps: density 10..=300 (default 50), lines 1..=30 (default 5)
    let num = |k: &str, d: usize, lo: f64, hi: f64| msg.get_f64(k).map_or(d, |v| v.round().clamp(lo, hi) as usize);
    let (density, lines) = (num("density", 50, 10.0, 300.0), num("lines", 5, 1.0, 30.0));
    let dir = msg.get_str("dir").unwrap_or("uv");
    let curve_json = |c: ck_tess::comb::CombCurve| {
        let f = |v: &[f64]| Json::Arr(v.iter().map(|&x| Json::Num(x)).collect());
        let mut o = Obj::new().put("p", f(&c.p)).put("k", f(&c.k));
        if let Some(d) = c.dir {
            o = o.put("dir", d);
        }
        o.build()
    };
    let one = |r: &Json| -> Json {
        let err = |e: &str| Obj::new().put("error", e).build();
        match r.get_str("kind") {
            Some(k @ ("face" | "edge")) => match resolve_ref(body, r) {
                Some(Ref::Edge(e)) => {
                    let ed = body.edge(e);
                    let c = ck_tess::comb::edge_comb(&ed.curve, ed.t0, ed.t1, density);
                    Obj::new().put("curves", Json::Arr(vec![curve_json(c)])).build()
                }
                Some(Ref::Face(f)) => {
                    let cs = ck_tess::comb::face_comb(body, f, density, lines, dir);
                    Obj::new().put("curves", Json::Arr(cs.into_iter().map(curve_json).collect())).build()
                }
                None => {
                    let _ = k;
                    err("notFound")
                }
            },
            Some("curve") => err("notFound"),
            _ => err("notCurve"),
        }
    };
    Json::Arr(msg.get_arr("refs").unwrap_or(&[]).iter().map(one).collect())
}

/// `matchEdges` on a part with features: a private regen (span
/// "regenerate N features", nothing stored), then the edge matcher.
fn match_edges_hook(part: &Json, cx: &mut Cx) -> Result<Json, String> {
    let part = Part::from_json(part)?;
    let opts = RegenOpts { up_to: cx.msg.get_f64("upTo").filter(|v| *v >= 0.0).map(|v| v as u32), ..RegenOpts::default() };
    let n = opts.up_to.map_or(part.features.len(), |u| part.features.len().min(u as usize));
    let body = cx.span(regen_label(n), |cx| {
        let mut ex = Exec { brep: Brep::new(), sink: TraceSink { trace: &mut *cx.trace, ids: vec![] }, auto_heal: None };
        regen::regen(&part, &opts, &mut ex);
        ex.brep.body
    });
    Ok(match body {
        Some(b) => ck_ops::match_edges::reply(&b, &cx.msg),
        None => Json::Arr(vec![]),
    })
}

/// `recognize`: feature recognition of the tag's body; `apply` stores the rebuilt body under `<tag>.rebuilt`.
fn recognize_query(cx: &mut Cx) -> Result<Json, String> {
    let (out, rebuilt) = ck_recog::request::run(&snapshot(cx)?.body, cx.msg);
    if let Some(b) = rebuilt {
        let tag = format!("{}.rebuilt", cx.tag());
        let snap = Snapshot { body: b, sketches: vec![], asm: Default::default(), heal_base: None };
        cx.session.put(&tag, State { has_body: true, data: Some(Box::new(snap)) });
    }
    Ok(out)
}
