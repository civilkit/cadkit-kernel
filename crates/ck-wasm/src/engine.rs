//! Composition root: installs the ck-model engine into ck-server's hooks.
//! ck-server stays geometry-free; everything engine-side is reached from here.

use ck_json::{Json, Obj};
use ck_model::features::{body_wire_with, Brep};
use ck_model::regen::{self, RegenOpts, SketchResult, Tracer};
use ck_model::sketch::offset::sketch_offset_json;
use ck_model::{Part, Sketch};
use ck_server::session::{regen_label, Cx, Hooks, State, SNAPSHOT_LABEL};
use ck_server::trace::{SpanId, Trace};
use ck_topo::body::Body;

pub fn install() {
    ck_server::set_hooks(Hooks {
        regen: Some(regen_hook),
        query: Some(query_hook),
        sketch_offset: Some(sketch_offset_hook),
        ..Hooks::default()
    });
}

/// What a regen leaves behind for later requests on its tag.
pub struct Snapshot {
    pub body: Body,
    pub sketches: Vec<SketchResult>,
}

/// ck-server trace as a ck-model [`Tracer`].
struct TraceSink<'a> {
    trace: &'a mut Trace,
    ids: Vec<Option<SpanId>>,
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
struct Exec<'a> {
    brep: Brep,
    sink: TraceSink<'a>,
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
    fn prepare(&mut self, state: &regen::RegenState, features: &[&ck_model::Feature]) {
        self.brep.prepare(state, features);
        self.brep.flush_spans(&mut self.sink);
    }
    fn exec(&mut self, state: &mut regen::RegenState, f: &ck_model::Feature) -> Result<(), regen::FeatureError> {
        let r = self.brep.exec(state, f);
        self.brep.flush_spans(&mut self.sink);
        r
    }
}

/// `regen`: parse, run the feature program, snapshot.
fn regen_hook(part: &Json, cx: &mut Cx) -> Result<(Json, State), String> {
    let part = Part::from_json(part)?;
    let opts = RegenOpts::from_json(cx.msg.get("opts"));
    let n = opts.up_to.map_or(part.features.len(), |u| part.features.len().min(u as usize));
    let (mut result, body) = cx.span(regen_label(n), |cx| {
        let mut ex = Exec { brep: Brep::new(), sink: TraceSink { trace: &mut *cx.trace, ids: vec![] } };
        let mut r = regen::regen(&part, &opts, &mut ex);
        r.components = ex.brep.components_json();
        let rn = ex.brep.renorm_faces();
        (r, (ex.brep.body, rn))
    });
    let (body, renorm) = body;
    let mut out = result.to_json();
    cx.span(SNAPSHOT_LABEL, |cx| {
        if let Some(b) = &body {
            let t = cx.trace.begin(String::new());
            let tess = ck_tess::mesh::tessellate(b);
            cx.trace.relabel(&t, format!("tessellate {} faces → {} triangles", b.num_faces(), tess.mesh.tri_face.len()));
            cx.trace.end(t);
            set_body(&mut out, body_json(b, &tess, cx, &renorm));
        }
    });
    let state = match body {
        Some(body) => {
            let sketches = std::mem::take(&mut result.sketches);
            State { has_body: true, data: Some(Box::new(Snapshot { body, sketches })) }
        }
        None => State::empty(),
    };
    Ok((out, state))
}

fn set_body(result: &mut Json, body: Json) {
    if let Json::Obj(kv) = result {
        if let Some(slot) = kv.iter_mut().find(|(k, _)| k == "body") {
            slot.1 = body;
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
    let m = &tess.mesh;
    let positions = cx.bufs.f32(&m.positions);
    let normals = cx.bufs.f32(&m.normals);
    let mids = cx.bufs.f32(&m.mids);
    let tri_face = cx.bufs.i32(&m.tri_face);
    let silhouette = cx.bufs.f32(&m.silhouette);
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
    kv.push(("silhouette".into(), silhouette));
    kv.push((
        "mesh".into(),
        Obj::new().put("positions", positions).put("normals", normals).put("mids", mids).put("triFace", tri_face).build(),
    ));
    Json::Obj(kv)
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
        "inspect" => inspect(cx),
        "continuity" => Ok(ck_ops::continuity::reply(&snapshot(cx)?.body, &cx.msg)),
        "measure" => Ok(ck_ops::measure::reply(&snapshot(cx)?.body, &cx.msg)),
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
