//! Feature recognition: turns a history-free B-rep (STEP / SolidWorks import) into our parametric feature tree.
//!
//! Deterministic, no ML. Order: holes (coaxial concave cylinder / cone groups) are claimed first; then the dominant
//! prismatic extrusion (wall cycles -> sketch profile, fillets / chamfers between cap and wall left out and recorded
//! as blend features); failing that a revolve of the half profile. Everything else is reported as unexplained.
//! The caller verifies numerically against the original (`rebuild::verify`).

pub mod emit;
pub mod holes;
pub mod model;
pub mod prism;
pub mod rebuild;
pub mod request;
pub mod revolve;

use emit::*;
use ck_geom::Vec3;
use ck_json::{Json, Obj};
use ck_topo::body::{Body, FaceId};
use holes::Hole;
use model::*;
use prism::{BlendKind, Cycle, Geom, Prism};
use std::collections::{HashMap, HashSet};

pub struct Item {
    pub id: String,
    pub ty: &'static str,
    pub kind: String,
    pub faces: Vec<FaceId>,
    pub confidence: f64,
    pub params: Json,
}

pub struct Recognition {
    pub features: Vec<Json>,
    pub items: Vec<Item>,
    pub unexplained: Vec<FaceId>,
    pub strategy: &'static str,
    pub note: Option<String>,
}

fn letter(i: usize) -> &'static str {
    ["x", "y", "z"][i]
}

pub fn recognize(body: &Body) -> Recognition {
    let m = Model::new(body);
    let all: Vec<FaceId> = m.faces.clone();
    let mut rec = Recognition { features: vec![], items: vec![], unexplained: all.clone(), strategy: "none", note: None };
    if body.num_lumps() != 1 {
        rec.note = Some("more than one lump".into());
        return rec;
    }
    let mut claimed: HashSet<FaceId> = HashSet::new();
    let hs = holes::find_holes(&m, &mut claimed);
    // best prism over the three principal axes
    let mut best: Option<Prism> = None;
    for ai in 0..3 {
        if let Some(p) = prism::analyze(&m, &claimed, ai) {
            let better = match &best {
                None => true,
                Some(b) => p.leftover.len() < b.leftover.len(),
            };
            if better {
                best = Some(p);
            }
        }
    }
    let prism_clean = best.as_ref().map_or(false, |p| p.leftover.is_empty());
    let mut tree = Tree::default();
    let mut items: Vec<Item> = vec![];
    let mut explained: HashSet<FaceId> = HashSet::new();
    if prism_clean || (best.is_some() && revolve::analyze(&m).is_none()) {
        let p = best.unwrap();
        rec.strategy = "prism";
        let names = emit_prism(&mut tree, &p, &mut items, &mut explained);
        emit_holes(&mut tree, &m, &hs, &mut items, &mut explained);
        emit_blends(&mut tree, &p, &names, &mut items, &mut explained);
    } else if let Some(r) = revolve::analyze(&m) {
        rec.strategy = "revolve";
        emit_revolve(&mut tree, &r, &mut items, &mut explained);
    } else {
        rec.note = Some("no dominant extrusion or revolution".into());
    }
    rec.unexplained = all.into_iter().filter(|f| !explained.contains(f)).collect();
    rec.features = tree.feats;
    rec.items = items;
    rec
}

// ---------------------------------------------------------------- prism

fn emit_prism(tree: &mut Tree, p: &Prism, items: &mut Vec<Item>, explained: &mut HashSet<FaceId>) -> HashMap<FaceId, String> {
    let plane = tree.plane(&p.spec, p.zmin);
    let sid = tree.ids.next();
    let mut sk = Sk::new(sid.clone(), plane);
    let mut ents: HashMap<FaceId, String> = HashMap::new();
    for c in &p.cycles {
        match c {
            Cycle::Circle { c, r, face } => {
                let pc = sk.point(c.0, c.1);
                let e = sk.circle(&pc, *r);
                ents.insert(*face, e);
            }
            Cycle::Poly(segs) => {
                let pts: Vec<String> = segs.iter().map(|s| sk.point(s.a.0, s.a.1)).collect();
                for (i, s) in segs.iter().enumerate() {
                    let (pa, pb) = (&pts[i], &pts[(i + 1) % pts.len()]);
                    let e = match &s.g {
                        Geom::Line => sk.line(pa, pb, false),
                        Geom::Arc { c, ccw } => {
                            let pc = sk.point(c.0, c.1);
                            if *ccw {
                                sk.arc(&pc, pa, pb)
                            } else {
                                sk.arc(&pc, pb, pa)
                            }
                        }
                    };
                    ents.insert(s.face, e);
                }
            }
        }
    }
    tree.push(sk.json());
    let eid = tree.ids.next();
    let depth = p.zmax - p.zmin;
    tree.push(extrude(&eid, &sid, "add", depth, false, false));
    let mut names: HashMap<FaceId, String> = HashMap::new();
    let mut faces: Vec<FaceId> = vec![];
    for (f, e) in &ents {
        names.insert(*f, format!("{eid}.S.{e}"));
        faces.push(*f);
    }
    if let Some(t) = p.top {
        names.insert(t, format!("{eid}.C1"));
        faces.push(t);
    }
    if let Some(b) = p.bottom {
        names.insert(b, format!("{eid}.C0"));
        faces.push(b);
    }
    faces.sort();
    for f in &faces {
        explained.insert(*f);
    }
    let params = Obj::new()
        .put("axis", letter(p.ai))
        .put("depth", depth)
        .put("level", p.zmin)
        .put("profileLoops", p.cycles.len() as f64)
        .build();
    items.push(Item { id: eid, ty: "extrude", kind: "base".into(), faces, confidence: 0.95, params });
    names
}

fn emit_blends(tree: &mut Tree, p: &Prism, names: &HashMap<FaceId, String>, items: &mut Vec<Item>, explained: &mut HashSet<FaceId>) {
    // group by kind and parameter
    let mut groups: Vec<(BlendKind, Vec<&prism::Blend>)> = vec![];
    for b in &p.blends {
        let same = |k: &BlendKind| match (k, &b.kind) {
            (BlendKind::Fillet(x), BlendKind::Fillet(y)) | (BlendKind::Chamfer(x), BlendKind::Chamfer(y)) => (x - y).abs() < 1e-6,
            _ => false,
        };
        match groups.iter_mut().find(|(k, _)| same(k)) {
            Some((_, v)) => v.push(b),
            None => groups.push((b.kind, vec![b])),
        }
    }
    for (kind, bs) in groups {
        let mut edges = vec![];
        let mut faces = vec![];
        for b in &bs {
            let (Some(cn), Some(wn)) = (names.get(&b.cap), names.get(&b.wall)) else { continue };
            edges.push(Obj::new().put("faces", arr(vec![s(cn), s(wn)])).put("at", vec3(b.at)).build());
            faces.extend(b.faces.iter().copied());
        }
        if edges.is_empty() {
            continue;
        }
        let id = tree.ids.next();
        let (ty, param) = match kind {
            BlendKind::Fillet(r) => ("fillet", r),
            BlendKind::Chamfer(d) => ("chamfer", d),
        };
        let o = Obj::new().put("type", ty).put("id", id.as_str());
        let o = match kind {
            BlendKind::Fillet(r) => o.put("radius", n(r)),
            BlendKind::Chamfer(d) => o.put("chamferType", "equal").put("distance", n(d)),
        };
        tree.push(o.put("edges", arr(edges)).build());
        for f in &faces {
            explained.insert(*f);
        }
        items.push(Item { id, ty, kind: ty.into(), faces, confidence: 0.85, params: Obj::new().put(if ty == "fillet" { "radius" } else { "distance" }, param).build() });
    }
}

// ---------------------------------------------------------------- holes

#[derive(PartialEq)]
struct HKey {
    ai: usize,
    sign: i32,
    level: i64,
    d: i64,
    through: bool,
    depth: i64,
    tip: bool,
    cb: Option<(i64, i64)>,
    csk: Option<(i64, i64)>,
}

fn q(v: f64) -> i64 {
    (v * 1e5).round() as i64
}

fn emit_holes(tree: &mut Tree, m: &Model, hs: &[Hole], items: &mut Vec<Item>, explained: &mut HashSet<FaceId>) {
    let _ = m;
    let mut groups: Vec<(HKey, Vec<&Hole>)> = vec![];
    for h in hs {
        let Some((ai, sg)) = principal(h.axis_in) else { continue };
        let spec = PlaneSpec::for_axis(ai);
        let key = HKey {
            ai,
            sign: sg as i32,
            level: q(spec.level(h.centre)),
            d: q(h.d),
            through: h.through,
            depth: if h.through { 0 } else { q(h.depth) },
            tip: h.tip,
            cb: h.cbore.map(|(d, t)| (q(d), q(t))),
            csk: h.csk.map(|(d, a, _)| (q(d), q(a))),
        };
        match groups.iter_mut().find(|(k, _)| *k == key) {
            Some((_, v)) => v.push(h),
            None => groups.push((key, vec![h])),
        }
    }
    for (key, hv) in groups {
        let ai = key.ai;
        let spec = PlaneSpec::for_axis(ai);
        let h0 = hv[0];
        let level = spec.level(h0.centre);
        let flip_hole = key.sign > 0;
        let pts: Vec<(f64, f64)> = hv.iter().map(|h| spec.xy(h.centre)).collect();
        let pat = pattern_of(&hv, ai);
        let seeds: Vec<(f64, f64)> = match &pat {
            Some(pt) => vec![pt.seed],
            None => pts.clone(),
        };
        // sketch of the hole centres
        let plane = tree.plane(&spec, level);
        let sid = tree.ids.next();
        let mut sk = Sk::new(sid.clone(), plane.clone());
        for p in &seeds {
            let pid = sk.point(p.0, p.1);
            sk.point_entity(&pid);
        }
        tree.push(sk.json());
        let hid = tree.ids.next();
        let mut hf = Obj::new().put("type", "hole").put("id", hid.as_str()).put("sketch", sid.as_str()).put("kind", "simple").put("diameter", n(h0.d));
        if h0.through {
            hf = hf.put("end", "through");
        } else {
            hf = hf.put("end", "blind").put("depth", n(h0.depth));
            if h0.tip {
                hf = hf.put("tip", true);
            }
        }
        if flip_hole {
            hf = hf.put("flip", true);
        }
        tree.push(hf.build());
        let mut seed_ids = vec![hid.clone()];
        if let Some((cd, ch)) = h0.cbore {
            let sid2 = tree.ids.next();
            let mut sk2 = Sk::new(sid2.clone(), plane.clone());
            for p in &seeds {
                let pid = sk2.point(p.0, p.1);
                sk2.circle(&pid, cd / 2.0);
            }
            tree.push(sk2.json());
            let eid = tree.ids.next();
            tree.push(extrude(&eid, &sid2, "cut", ch, key.sign < 0, false));
            seed_ids.push(eid);
        }
        if let Some((hd, _ang, cd)) = h0.csk {
            // revolve-cut of the cone frustum, one per seed centre
            let ni = if ai == 2 { 1 } else { 2 };
            let spec2 = PlaneSpec::for_axis(ni);
            let a = h0.axis_in;
            let perp = if par(spec2.xs, a) { spec2.ys } else { spec2.xs };
            for p in hv.iter().take(seeds.len()) {
                let c = match &pat {
                    Some(pt) => pt.seed_world(ai, level, &spec),
                    None => p.centre,
                };
                let l2 = spec2.level(c);
                let plane2 = tree.plane(&spec2, l2);
                let sid3 = tree.ids.next();
                let mut sk3 = Sk::new(sid3.clone(), plane2);
                let r_head = hd / 2.0;
                let r_hole = h0.d / 2.0;
                let t = (r_head - r_hole) / cd;
                let e = 0.5;
                let w = |s_: f64, rho: f64| spec2.xy(c + a * s_ + perp * rho);
                let (q0, q1, q2, q3) = (w(-e, 0.0), w(-e, r_head + e * t), w(cd, r_hole), w(cd, 0.0));
                let (p0, p1, p2, p3) = (sk3.point(q0.0, q0.1), sk3.point(q1.0, q1.1), sk3.point(q2.0, q2.1), sk3.point(q3.0, q3.1));
                sk3.line(&p0, &p1, false);
                sk3.line(&p1, &p2, false);
                sk3.line(&p2, &p3, false);
                sk3.line(&p3, &p0, false);
                let ax = sk3.line(&p0, &p3, true);
                tree.push(sk3.json());
                let rid = tree.ids.next();
                tree.push(revolve(&rid, &sid3, "cut", &ax));
                seed_ids.push(rid);
            }
        }
        if let Some(pt) = &pat {
            let pid = tree.ids.next();
            let mut o = Obj::new().put("type", pt.ty).put("id", pid.as_str()).put("seeds", arr(seed_ids.iter().map(|x| s(x)).collect()));
            o = o.put("count", pt.count as f64);
            if pt.ty == "cpattern" {
                o = o.put("axis", Obj::new().put("kind", "axis").put("axis", letter(ai)).build()).put("angle", 360.0).put("equal", true);
            } else {
                o = o.put("dir", Obj::new().put("kind", "axis").put("axis", pt.dir_axis).build()).put("spacing", n(pt.spacing));
            }
            tree.push(o.build());
        }
        let mut faces = vec![];
        for h in &hv {
            faces.extend(h.faces.iter().copied());
        }
        for f in &faces {
            explained.insert(*f);
        }
        let params = Obj::new()
            .put("diameter", h0.d)
            .put("through", h0.through)
            .put("depth", if h0.through { Json::Null } else { Json::Num(h0.depth) })
            .put("count", hv.len() as f64)
            .put("pattern", pat.as_ref().map_or("none", |p| p.ty))
            .put("axis", letter(ai))
            .build();
        items.push(Item { id: hid, ty: "hole", kind: h0.kind().into(), faces, confidence: 0.9, params });
    }
}

struct Pat {
    ty: &'static str,
    count: usize,
    seed: (f64, f64),
    spacing: f64,
    dir_axis: &'static str,
}

impl Pat {
    fn seed_world(&self, _ai: usize, _level: f64, _spec: &PlaneSpec) -> Vec3 {
        // only called for csk seeds of a pattern; the caller falls back to the first hole's centre
        Vec3::ZERO
    }
}

/// Circular (about a world axis through the origin) or linear (along a world axis) arrangement of identical holes.
fn pattern_of(hv: &[&Hole], ai: usize) -> Option<Pat> {
    if hv.len() < 2 || hv.iter().any(|h| h.csk.is_some()) {
        return None;
    }
    let spec = PlaneSpec::for_axis(ai);
    let pts: Vec<(f64, f64)> = hv.iter().map(|h| spec.xy(h.centre)).collect();
    let nn = pts.len();
    // linear along a sketch axis
    let eq = |a: f64, b: f64| (a - b).abs() < 1e-5;
    for (dir_idx, dir_axis_world) in [(0usize, spec.xs), (1usize, spec.ys)] {
        let other = |p: &(f64, f64)| if dir_idx == 0 { p.1 } else { p.0 };
        let along = |p: &(f64, f64)| if dir_idx == 0 { p.0 } else { p.1 };
        if pts.iter().all(|p| eq(other(p), other(&pts[0]))) {
            let mut v: Vec<f64> = pts.iter().map(along).collect();
            v.sort_by(|a, b| a.partial_cmp(b).unwrap());
            let sp = v[1] - v[0];
            if sp > 1e-6 && v.windows(2).all(|w| eq(w[1] - w[0], sp)) {
                let (wi, sg) = principal(dir_axis_world)?;
                let seed_along = if sg > 0.0 { v[0] } else { v[nn - 1] };
                let seed = if dir_idx == 0 { (seed_along, pts[0].1) } else { (pts[0].0, seed_along) };
                return Some(Pat { ty: "lpattern", count: nn, seed, spacing: sp, dir_axis: letter(wi) });
            }
        }
    }
    // circular about the world axis through the origin
    if nn >= 3 {
        let c = pts.iter().fold((0.0, 0.0), |a, p| (a.0 + p.0 / nn as f64, a.1 + p.1 / nn as f64));
        if c.0.abs() < 1e-5 && c.1.abs() < 1e-5 {
            let r0 = (pts[0].0.powi(2) + pts[0].1.powi(2)).sqrt();
            if r0 > 1e-6 && pts.iter().all(|p| ((p.0.powi(2) + p.1.powi(2)).sqrt() - r0).abs() < 1e-5) {
                let mut a: Vec<f64> = pts.iter().map(|p| p.1.atan2(p.0)).collect();
                a.sort_by(|x, y| x.partial_cmp(y).unwrap());
                let step = std::f64::consts::TAU / nn as f64;
                let ok = (0..nn).all(|i| {
                    let d = if i + 1 < nn { a[i + 1] - a[i] } else { a[0] + std::f64::consts::TAU - a[nn - 1] };
                    (d - step).abs() < 1e-5
                });
                if ok {
                    return Some(Pat { ty: "cpattern", count: nn, seed: pts[0], spacing: 0.0, dir_axis: "x" });
                }
            }
        }
    }
    None
}

// ---------------------------------------------------------------- revolve

fn emit_revolve(tree: &mut Tree, r: &revolve::Rev, items: &mut Vec<Item>, explained: &mut HashSet<FaceId>) {
    let plane = tree.plane(&r.spec, r.level);
    let sid = tree.ids.next();
    let mut sk = Sk::new(sid.clone(), plane);
    let pts: Vec<String> = r.segs.iter().map(|s| sk.point(s.a.0, s.a.1)).collect();
    for (i, sg) in r.segs.iter().enumerate() {
        let (pa, pb) = (&pts[i], &pts[(i + 1) % pts.len()]);
        match &sg.g {
            Geom::Line => {
                sk.line(pa, pb, false);
            }
            Geom::Arc { c, ccw } => {
                let pc = sk.point(c.0, c.1);
                if *ccw {
                    sk.arc(&pc, pa, pb);
                } else {
                    sk.arc(&pc, pb, pa);
                }
            }
        }
    }
    let a0 = sk.point(r.axis.0 .0, r.axis.0 .1);
    let a1 = sk.point(r.axis.1 .0, r.axis.1 .1);
    let ax = sk.line(&a0, &a1, true);
    tree.push(sk.json());
    let rid = tree.ids.next();
    tree.push(revolve(&rid, &sid, "add", &ax));
    for f in &r.faces {
        explained.insert(*f);
    }
    items.push(Item {
        id: rid,
        ty: "revolve",
        kind: "base".into(),
        faces: r.faces.clone(),
        confidence: 0.9,
        params: Obj::new().put("axis", letter(r.ai)).put("segments", r.segs.len() as f64).build(),
    });
}

// ---------------------------------------------------------------- report

pub fn report(body: &Body, rec: &Recognition) -> Json {
    let m = Model::new(body);
    let feats: Vec<Json> = rec
        .items
        .iter()
        .map(|it| {
            Obj::new()
                .put("id", it.id.as_str())
                .put("type", it.ty)
                .put("kind", it.kind.as_str())
                .put("confidence", it.confidence)
                .put("faces", arr(it.faces.iter().map(|f| s(&body.face_id_str(*f))).collect()))
                .put("params", it.params.clone())
                .build()
        })
        .collect();
    let un: Vec<Json> = rec
        .unexplained
        .iter()
        .map(|f| Obj::new().put("face", body.face_id_str(*f)).put("kind", m.kind(*f).name()).put("area", m.area(*f)).build())
        .collect();
    let total = m.faces.len().max(1);
    Obj::new()
        .put("strategy", rec.strategy)
        .put("note", rec.note.as_deref().map_or(Json::Null, |x| Json::Str(x.to_string())))
        .put("features", arr(feats))
        .put("unexplained", arr(un))
        .put("faces", total as f64)
        .put("explainedFraction", 1.0 - rec.unexplained.len() as f64 / total as f64)
        .build()
}
