//! Keep the rebuilt prefix (WP4, `docs/sldprt-editable-wp4.md`): when the history cannot be rebuilt to the end, the
//! first `k` SolidWorks steps stay as our features and an `importBody` feature (`replace`) puts the file's own body in
//! place of theirs, so the final body is exactly the plain import.
//!
//! `k` is chosen by local checks on one regeneration of the candidate prefix with a body snapshot after every step
//! (there is no per-step reference body in the file, so nothing is compared with a SolidWorks intermediate state):
//!
//! 1. **regen**: no feature of the step has a regen error, and the step left a body;
//! 2. **settled faces**: every face of the final body that no later step touched (all its tags at or before the step)
//!    lies on the snapshot: each of its sample points (vertices, edge mid points) is on a face of the snapshot with
//!    the same surface. When a later step may add material (a boss, or a step of unknown kind) the face may have
//!    grown without taking that step's tag (a coplanar boss merges into it: seen on the corpus), so then it only has
//!    to overlap a face of the snapshot (at least one sample point on it). A face of the step that a later step also
//!    touched only needs a face with the same surface;
//! 3. **volume**: a step that only removes material (cut, hole, shell, convex fillet) does not add any, one that only
//!    adds (boss, concave fillet) does not remove any; and where every later step of the file only removes (adds)
//!    material, the snapshot holds at least (at most) the final volume.
//!
//! `k` = the number of leading steps that pass, at most up to the first step that was not read back.

use crate::fastverify::same_surface;
use crate::{classify, Class, Status, Step};
use ck_json::{Json, Obj};
use ck_model::features::Brep;
use ck_model::regen::{self, FeatureExec, RegenOpts, Tracer};
use ck_model::Part;
use ck_recog::model::{Kind, Model};
use ck_sldprt::meta::SwFeature;
use ck_topo::body::{Body, FaceId};
use std::collections::HashMap;

/// Distance tolerance of the settled-face check (mm).
const PTOL: f64 = 1e-4;
/// Surface tolerance of the settled-face check (relative, as `fastverify`).
const STOL: f64 = 1e-5;

/// The checks of one step (`Outcome::checks`, aligned with `Outcome::steps` up to the step that failed).
#[derive(Debug, Clone, PartialEq)]
pub struct Check {
    /// `None` = passed; else what failed.
    pub regen: Option<String>,
    pub settled: Option<String>,
    pub volume: Option<String>,
    /// Final-body faces checked (full, surface only) and the snapshot's volume.
    pub faces: (usize, usize),
    pub vol: Option<f64>,
}

impl Check {
    pub fn ok(&self) -> bool {
        self.regen.is_none() && self.settled.is_none() && self.volume.is_none()
    }
    /// The first failing check, as `regen: ...` / `settled: ...` / `volume: ...`.
    pub fn failure(&self) -> Option<String> {
        self.regen.as_ref().map(|s| format!("regen: {s}")).or_else(|| self.settled.as_ref().map(|s| format!("settled faces: {s}"))).or_else(|| self.volume.as_ref().map(|s| format!("volume: {s}")))
    }
    pub fn json(&self) -> Json {
        let s = |v: &Option<String>| v.as_deref().map_or(Json::Null, |x| Json::Str(x.into()));
        Obj::new()
            .put("ok", self.ok())
            .put("regen", s(&self.regen))
            .put("settled", s(&self.settled))
            .put("volume", s(&self.volume))
            .put("facesChecked", self.faces.0 as f64)
            .put("facesSurfaceOnly", self.faces.1 as f64)
            .put("vol", self.vol.map_or(Json::Null, Json::Num))
            .build()
    }
}

/// What the imported remainder stands for.
#[derive(Debug, Clone)]
pub struct Remainder {
    /// The first SolidWorks step not kept: (id, name, kind).
    pub from: (i64, String, String),
    pub reason: String,
    /// Every SolidWorks step from `from` on (sketches and other no-geometry features left out).
    pub steps: Vec<(i64, String, String)>,
}

impl Remainder {
    pub fn json(&self) -> Json {
        let one = |s: &(i64, String, String)| Obj::new().put("id", s.0 as f64).put("name", s.1.as_str()).put("kind", s.2.as_str()).build();
        Obj::new().put("mode", "replace").put("from", one(&self.from)).put("reason", self.reason.as_str()).put("steps", Json::Arr(self.steps.iter().map(one).collect())).build()
    }

    /// The `importBody` feature (`blob` left empty: [`crate::Outcome::part_json`] fills it with the file's key).
    pub fn feature(&self, id: &str) -> Json {
        let r = self.json();
        let mut o = Obj::new().put("type", "importBody").put("id", id).put("name", "Imported remainder").put("format", "sldprt").put("blob", "").put("replace", true);
        for k in ["from", "reason", "steps"] {
            o = o.put(k, r.get(k).cloned().unwrap_or(Json::Null));
        }
        o.build()
    }
}

#[derive(Default)]
struct Noop;
impl Tracer for Noop {
    fn begin(&mut self, _label: &str) -> usize {
        0
    }
    fn relabel(&mut self, _id: usize, _label: &str) {}
    fn end(&mut self, _id: usize) {}
}

/// `ck_recog::rebuild`'s executor, keeping the body after every feature of a step (the last one run wins).
struct Exec {
    brep: Brep,
    labels: Noop,
    step_of: HashMap<String, usize>,
    snaps: Vec<Option<Body>>,
    ran: Vec<bool>,
}

impl FeatureExec for Exec {
    fn tracer(&mut self) -> Option<&mut dyn Tracer> {
        Some(&mut self.labels)
    }
    fn face_count(&self) -> Option<usize> {
        self.brep.face_count()
    }
    fn face_frame(&mut self, s: &regen::RegenState, face: &str) -> Option<ck_core::linalg::Frame> {
        self.brep.face_frame(s, face)
    }
    fn edge_line(&mut self, s: &regen::RegenState, faces: &[String], at: Option<[f64; 3]>) -> Option<(ck_core::linalg::Vec3, ck_core::linalg::Vec3)> {
        self.brep.edge_line(s, faces, at)
    }
    fn edge_seg(&mut self, s: &regen::RegenState, faces: &[String], at: Option<ck_core::linalg::Vec3>) -> Option<ck_ops::sweep::PathSeg> {
        self.brep.edge_seg(s, faces, at)
    }
    fn vertex_point(&mut self, s: &regen::RegenState, faces: &[String], at: Option<ck_core::linalg::Vec3>) -> Option<ck_core::linalg::Vec3> {
        self.brep.vertex_point(s, faces, at)
    }
    fn face_surf(&mut self, s: &regen::RegenState, face: &str) -> Option<regen::FaceSurf> {
        self.brep.face_surf(s, face)
    }
    fn exec(&mut self, s: &mut regen::RegenState, f: &ck_model::Feature) -> Result<(), regen::FeatureError> {
        let r = self.brep.exec(s, f);
        if let Some(&j) = self.step_of.get(&f.id) {
            self.snaps[j] = self.brep.body.clone();
            self.ran[j] = true;
        }
        r
    }
}

/// Regenerate a whole part document (with its `blobs`, so an `importBody` remainder reads the file); the body and the
/// regen errors (`feature: code`).
pub fn regen_part(part_json: &Json) -> Result<(Option<Body>, Vec<String>), String> {
    let part = Part::from_json(part_json)?;
    let mut ex = Exec { brep: Brep::new(), labels: Noop, step_of: HashMap::new(), snaps: vec![], ran: vec![] };
    let r = regen::regen(&part, &RegenOpts::default(), &mut ex);
    Ok((ex.brep.body, r.errors.iter().map(|e| format!("{}: {}", e.feature, e.error.code)).collect()))
}

/// One regeneration of `features` with a body snapshot after each step (`ranges[j]` = the step's features). A step
/// with no features of its own (skipped, cosmetic) carries the previous snapshot. Also the regen errors per step.
pub fn snapshots(features: &[Json], ranges: &[(usize, usize)]) -> Result<(Vec<Option<Body>>, Vec<Vec<String>>), String> {
    let part_json = Obj::new().put("format", "ck-part").put("version", 1.0).put("features", Json::Arr(features.to_vec())).build();
    let part = Part::from_json(&part_json)?;
    let mut step_of = HashMap::new();
    for (j, &(a, b)) in ranges.iter().enumerate() {
        for i in a..b.min(features.len()) {
            if let Some(id) = features[i].get_str("id") {
                step_of.insert(id.to_string(), j);
            }
        }
    }
    let mut ex = Exec { brep: Brep::new(), labels: Noop, step_of: step_of.clone(), snaps: vec![None; ranges.len()], ran: vec![false; ranges.len()] };
    let r = regen::regen(&part, &RegenOpts::default(), &mut ex);
    let mut errs = vec![vec![]; ranges.len()];
    for e in &r.errors {
        if let Some(&j) = step_of.get(&e.feature) {
            errs[j].push(format!("{}: {}", e.feature, e.error.code));
        }
    }
    let mut snaps = ex.snaps;
    for j in 0..ranges.len() {
        if !ex.ran[j] && ranges[j].0 == ranges[j].1 && j > 0 {
            snaps[j] = snaps[j - 1].clone();
        }
    }
    Ok((snaps, errs))
}

thread_local! {
    /// Volume share of a face (about the origin) by a hash of its surface and trimming.
    static FACE_VOL: std::cell::RefCell<HashMap<u64, f64>> = std::cell::RefCell::new(HashMap::new());
}

/// The volume of a body, as `mass_props(b).volume` to rounding, with each face's share remembered: the snapshots of
/// consecutive steps share most of their faces, and the integral over a free-form face (a blend) is the cost of a
/// snapshot.
fn body_volume(b: &Body) -> f64 {
    let mut total = 0.0;
    for l in b.lump_ids() {
        for f in ck_topo::mass::lump_faces(b, l) {
            let h = crate::fastverify::face_hash(b, f);
            let hit = FACE_VOL.with(|m| m.borrow().get(&h).copied());
            total += hit.unwrap_or_else(|| {
                let v = ck_topo::mass::face_integral_vpoly::<1>(b, f, 1, &|p, n| [p.dot(n) / 3.0])[0];
                FACE_VOL.with(|m| {
                    let mut m = m.borrow_mut();
                    if m.len() >= 1 << 16 {
                        m.clear();
                    }
                    m.insert(h, v);
                });
                v
            });
        }
    }
    total
}

/// Material a SolidWorks step adds or removes, where it is known from its type (and a fillet's faces).
#[derive(Debug, Clone, Copy, PartialEq)]
enum Dir {
    Adds,
    Removes,
    /// No geometry (cosmetic, skipped).
    Neutral,
    Unknown,
}

fn dir_of(f: &SwFeature, own: &[FaceId], m: &Model) -> Dir {
    match classify(f) {
        // (a modelled thread, one with faces of its own, adds a ridge or cuts a groove)
        Class::Cosmetic if !own.is_empty() => match crate::thread::adds(m, own) {
            Some(true) => Dir::Adds,
            Some(false) => Dir::Removes,
            None => Dir::Unknown,
        },
        Class::Skip | Class::Cosmetic | Class::Draft | Class::SheetRules => Dir::Neutral,
        Class::BaseFlange => Dir::Adds,
        Class::Extrude { cut } | Class::Revolve { cut } => {
            if cut {
                Dir::Removes
            } else {
                Dir::Adds
            }
        }
        Class::Hole | Class::Shell => Dir::Removes,
        Class::Rib => Dir::Adds,
        // a round on a convex edge takes material away, one in a concave corner fills it
        Class::Fillet => {
            let cv: Vec<Option<bool>> = own
                .iter()
                .map(|&g| match m.kind(g) {
                    Kind::Cyl { convex, .. } | Kind::Torus { convex, .. } | Kind::Sphere { convex, .. } => Some(*convex),
                    _ => None,
                })
                .collect();
            if cv.is_empty() || cv.iter().any(|c| c.is_none()) {
                Dir::Unknown
            } else if cv.iter().all(|c| *c == Some(true)) {
                Dir::Removes
            } else if cv.iter().all(|c| *c == Some(false)) {
                Dir::Adds
            } else {
                Dir::Unknown
            }
        }
        // a chamfer may cut a convex edge or fill a concave one; patterns copy either; the rest is not known
        _ => Dir::Unknown,
    }
}

/// Sample points of a face of the final body: vertices and edge mid points, without repeats.
fn samples(m: &Model, f: FaceId) -> Vec<ck_geom::Vec3> {
    let mut out: Vec<ck_geom::Vec3> = vec![];
    for p in crate::face_points(m, f) {
        if !out.iter().any(|q| (*q - p).len() < 1e-9) {
            out.push(p);
        }
    }
    out
}

/// The file stores an edge as an approximating B-spline when it is the intersection with a free-form neighbour: its
/// end points lie on the face's analytic surface to 1e-15, but the interior can be off that surface by a few 1e-4 mm
/// (nist_ftc_10: the mid point of a spline edge is 2.9e-4 from the cylinder it bounds). Such a sample is as far from
/// the face as the file's own storage puts it, so the test of it against the rebuilt face allows that deviation on
/// top of `PTOL`. A sample on its surface (every vertex, every analytic edge) keeps `PTOL`, and so does one more
/// than `SNAP` off the surface; the surface itself is still held to `STOL` by `same_surface`.
const SNAP: f64 = 1e-3;
fn surface_dev(k: &Kind, p: ck_geom::Vec3) -> f64 {
    let d = match *k {
        Kind::Plane { n, d } => (n.dot(p) - d).abs(),
        Kind::Cyl { o, a, r, .. } => {
            let w = p - o;
            ((w - a * a.dot(w)).len() - r).abs()
        }
        Kind::Sphere { c, r, .. } => ((p - c).len() - r).abs(),
        Kind::Torus { c, a, major, minor, .. } => {
            let w = p - c;
            let rad = w - a * a.dot(w);
            let l = rad.len();
            if l < 1e-12 {
                return 0.0;
            }
            let q = p - (c + rad * (major / l));
            (q.len() - minor).abs()
        }
        _ => return 0.0,
    };
    if d <= SNAP {
        d
    } else {
        0.0
    }
}

/// The inputs of the checks.
pub struct Input<'a> {
    pub body: &'a Body,
    pub tags: &'a [Vec<(i64, i64)>],
    pub features: &'a [SwFeature],
    pub steps: &'a [Step],
    pub snaps: &'a [Option<Body>],
    pub errs: &'a [Vec<String>],
}

/// The checks of steps `0..` until the first one that fails (that one included).
pub fn check_steps(inp: &Input) -> Vec<Check> {
    let m = Model::new(inp.body);
    // tree position of every feature of the file (tags of features not in the list count as later than any step)
    let pos: HashMap<i64, usize> = inp.features.iter().enumerate().map(|(i, f)| (f.id, i)).collect();
    let tpos = |id: i64| pos.get(&id).copied().unwrap_or(usize::MAX);
    let mut own: HashMap<i64, Vec<FaceId>> = HashMap::new();
    for &g in &m.faces {
        if let Some(c) = inp.tags.get(g.idx()).and_then(|v| v.iter().map(|t| t.0).min()) {
            own.entry(c).or_default().push(g);
        }
    }
    let blends: std::collections::HashSet<i64> = inp.features.iter().filter(|f| matches!(classify(f), Class::Fillet | Class::Chamfer)).map(|f| f.id).collect();
    let dirs: Vec<Dir> = inp.features.iter().map(|f| dir_of(f, own.get(&f.id).map_or(&[][..], |v| v), &m)).collect();
    // faces a later draft tilts: not settled before the draft step
    let tilted_by: HashMap<FaceId, usize> = crate::later_draft::find(&m, inp.tags, inp.features, 0).into_iter().map(|l| (l.face, l.pos)).collect();
    // (the file's volume is only compared with the steps' bodies: a part whose first step is unsupported has none to
    // compare, and the integral over a whole free-form body is seconds)
    let vol_f = if inp.steps.first().is_some_and(|s| s.status != Status::Unsupported) { body_volume(inp.body) } else { 0.0 };
    let eps = 1e-6 * vol_f.abs().max(1e-9);
    let mut out = vec![];
    let mut prev_vol = 0.0;
    for (j, st) in inp.steps.iter().enumerate() {
        let mut c = Check { regen: None, settled: None, volume: None, faces: (0, 0), vol: None };
        if st.status == Status::Unsupported {
            break;
        }
        let snap = inp.snaps.get(j).and_then(|s| s.as_ref());
        if let Some(e) = inp.errs.get(j).filter(|e| !e.is_empty()) {
            c.regen = Some(e.join(", "));
        } else if snap.is_none() && st.status == Status::Rebuilt && st.kind != "Sheet-Metal" {
            c.regen = Some("the step left no body".into());
        }
        let pj = tpos(st.sw_id);
        if let (None, Some(p)) = (&c.regen, snap) {
            let mp = Model::new(p);
            // can a later step have grown a face of this one? (only a step that adds material, or one of unknown kind)
            let extendable = dirs.iter().skip(pj.saturating_add(1)).any(|d| matches!(d, Dir::Adds | Dir::Unknown));
            let mut loops = ck_ops::boolean::classify::LoopCache::new();
            // (a face of the step's body is only asked about a point inside its box, grown by the test's tolerance
            // and a hundredth of its size: the box of a curved face comes from samples of its surface)
            let boxes: Vec<ck_ops::bvh::Aabb> = mp.faces.iter().map(|&h| ck_ops::bvh::face_box(p, h)).collect();
            let at: HashMap<FaceId, usize> = mp.faces.iter().enumerate().map(|(i, &h)| (h, i)).collect();
            let near = |h: FaceId, q: ck_geom::Vec3, tl: f64| {
                let bb = boxes[at[&h]];
                let g = tl + 1e-6 + 0.01 * bb.diagonal();
                !(bb.min.x <= bb.max.x) || q.x >= bb.min.x - g && q.x <= bb.max.x + g && q.y >= bb.min.y - g && q.y <= bb.max.y + g && q.z >= bb.min.z - g && q.z <= bb.max.z + g
            };
            // (the faces of the step's body that can pass `near` for a point: their boxes grown as `near` grows them
            // at the largest tolerance of a sample, in a tree, so a face of a large body is not compared with every
            // face of the snapshot; a face without a box passes `near` everywhere and is always a candidate)
            let open: Vec<FaceId> = mp.faces.iter().zip(&boxes).filter(|(_, bb)| !(bb.min.x <= bb.max.x)).map(|(&h, _)| h).collect();
            let boxed: Vec<usize> = (0..mp.faces.len()).filter(|&i| boxes[i].min.x <= boxes[i].max.x).collect();
            let tree = ck_ops::bvh::Bvh::build(&boxed.iter().map(|&i| boxes[i].padded(PTOL + SNAP + 1e-6 + 0.01 * boxes[i].diagonal())).collect::<Vec<_>>());
            let near_any = |pts: &[ck_geom::Vec3]| -> Vec<FaceId> {
                let mut hit: Vec<usize> = pts.iter().flat_map(|&q| tree.query_box(&ck_ops::bvh::Aabb::new(q, q))).map(|k| boxed[k]).collect();
                hit.sort_unstable();
                hit.dedup();
                hit.into_iter().map(|i| mp.faces[i]).chain(open.iter().copied()).collect()
            };
            'faces: for &g in &m.faces {
                let Some(ts) = inp.tags.get(g.idx()).filter(|v| !v.is_empty()) else { continue };
                if tilted_by.get(&g).map_or(false, |&pd| pj < pd) {
                    continue;
                }
                let last = ts.iter().map(|t| tpos(t.0)).max().unwrap();
                let mine = ts.iter().any(|t| t.0 == st.sw_id);
                let full = last <= pj;
                if !full && !mine {
                    continue;
                }
                let k = m.kind(g);
                if matches!(k, Kind::Other(_)) {
                    continue;
                }
                // (a blend the kernel builds as a free-form surface where the file has a cylinder or torus: the strip
                // is a spline there, which the point test below still holds to the face within tolerance)
                let blend = ts.iter().any(|t| blends.contains(&t.0));
                let spline_ok = blend && matches!(k, Kind::Cyl { .. } | Kind::Torus { .. } | Kind::Cone { .. } | Kind::Sphere { .. });
                let pts = samples(&m, g);
                let on_surface = |h: &FaceId| same_surface(k, mp.kind(*h), STOL) || (spline_ok && matches!(mp.kind(*h), Kind::Other(_)));
                // (the faces near the face's points first: only those can take part in the point test below; the whole
                // body only when none of them is on the surface)
                let around = near_any(&pts);
                let mut cands: Vec<FaceId> = around.iter().copied().filter(on_surface).collect();
                if cands.is_empty() {
                    cands = mp.faces.iter().copied().filter(on_surface).collect();
                }
                let what = || format!("face {} ({}, tags {:?})", inp.body.face_id_str(g), k.name(), ts.iter().map(|t| t.0).collect::<Vec<_>>());
                if cands.is_empty() {
                    c.settled = Some(format!("{}: no face of the step's body on its surface", what()));
                    break 'faces;
                }
                if !full {
                    c.faces.1 += 1;
                    continue;
                }
                c.faces.0 += 1;
                let tols: Vec<f64> = pts.iter().map(|&q| PTOL + surface_dev(&k, q)).collect();
                // (a sample the file's own edge storage put off the surface lies on the boundary shared with a tangent blend,
                // where a few 1e-4 normal gap is a slide of 1e-2 along the surface: it must lie on the step's body
                // boundary (any face of it) within that deviation)
                let on: Vec<bool> = pts
                    .iter()
                    .zip(&tols)
                    .map(|(&q, &tl)| {
                        let fs: &[FaceId] = if tl > PTOL { &around } else { &cands };
                        fs.iter().any(|&h| near(h, q, tl) && ck_ops::boolean::classify::point_in_face_cached(p, h, q, tl, &mut loops) != ck_ops::boolean::classify::Where::Out)
                    })
                    .collect();
                if extendable {
                    // a later step that adds material may have grown the face (a coplanar boss merges into it and
                    // the merged face keeps the earlier tag only): the face must still overlap the step's face
                    if !pts.is_empty() && !on.iter().any(|&o| o) {
                        c.settled = Some(format!("{}: none of its points is on the step's body", what()));
                        break 'faces;
                    }
                } else if let Some(i) = on.iter().position(|&o| !o) {
                    let q = pts[i];
                    c.settled = Some(format!("{}: point ({:.4}, {:.4}, {:.4}) is not on the step's body", what(), q.x, q.y, q.z));
                    break 'faces;
                }
            }
            let v = body_volume(p);
            c.vol = Some(v);
            let d = dirs.get(pj).copied().unwrap_or(Dir::Unknown);
            if d == Dir::Removes && v > prev_vol + eps {
                c.volume = Some(format!("a step that removes material grew the body from {prev_vol:.6} to {v:.6}"));
            } else if d == Dir::Adds && v < prev_vol - eps {
                c.volume = Some(format!("a step that adds material shrank the body from {prev_vol:.6} to {v:.6}"));
            } else {
                let later: Vec<Dir> = dirs.iter().skip(pj.saturating_add(1)).copied().filter(|d| *d != Dir::Neutral).collect();
                if !later.is_empty() && later.iter().all(|d| *d == Dir::Removes) && v < vol_f - eps {
                    c.volume = Some(format!("every later step removes material, but the body ({v:.6}) is smaller than the file's ({vol_f:.6})"));
                } else if !later.is_empty() && later.iter().all(|d| *d == Dir::Adds) && v > vol_f + eps {
                    c.volume = Some(format!("every later step adds material, but the body ({v:.6}) is larger than the file's ({vol_f:.6})"));
                }
            }
            prev_vol = v;
        }
        let ok = c.ok();
        out.push(c);
        if !ok {
            break;
        }
    }
    out
}
