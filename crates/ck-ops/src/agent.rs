//! Agent-interface queries over a regenerated body: `check` (intent
//! expectations), `resolve` (description -> persistent names) and
//! `describe` (name -> readable description). See docs/agent-queries.md for
//! the wire shapes and the description grammar. ADR 0002 items 3-4, ADR 0003
//! item 5.
//!
//! Everything here is read-only and keyed by the persistent names the body
//! already carries: face ids (`F2.C1`) and edge keys (`A|B#seq`, as listed in
//! the regen reply's `body.edges[].key`).

use ck_core::linalg::Vec3;
use ck_geom::curve::Curve;
use ck_geom::surface::Surface;
use ck_json::{Json, Obj};
use ck_topo::body::{Body, FaceId, Role};
use std::collections::HashMap;
use std::f64::consts::{PI, TAU};

const COS_TOL: f64 = 1.0 - 1e-6;

fn comp(v: Vec3, i: usize) -> f64 {
    [v.x, v.y, v.z][i]
}

fn rnd(x: f64) -> f64 {
    if !x.is_finite() {
        return x;
    }
    let r = (x * 1e9).round() / 1e9;
    if r == 0.0 {
        0.0
    } else {
        r
    }
}

fn vj(v: Vec3) -> Json {
    Json::Arr(vec![Json::Num(rnd(v.x)), Json::Num(rnd(v.y)), Json::Num(rnd(v.z))])
}

fn fmt_num(x: f64) -> String {
    let r = rnd(x);
    let s = format!("{:.4}", r);
    s.trim_end_matches('0').trim_end_matches('.').to_string()
}

fn fmt_pt(v: Vec3) -> String {
    format!("({}, {}, {})", fmt_num(v.x), fmt_num(v.y), fmt_num(v.z))
}

/// `"+z"` / `"-x"` when `v` is axis-aligned, else the rounded vector.
pub fn dir_label(v: Vec3) -> String {
    for (i, n) in ["x", "y", "z"].iter().enumerate() {
        let c = comp(v, i);
        if c.abs() >= COS_TOL {
            return format!("{}{}", if c > 0.0 { "+" } else { "-" }, n);
        }
    }
    fmt_pt(v)
}

/// Axis letter of an axis-aligned direction regardless of sign.
fn axis_letter(v: Vec3) -> Option<usize> {
    (0..3).find(|&i| comp(v, i).abs() >= COS_TOL)
}

// ---------------------------------------------------------------- facts

#[derive(Debug, Clone)]
pub struct FaceFact {
    pub id: String,
    pub kind: &'static str,
    pub area: f64,
    pub centroid: Vec3,
    pub normal: Option<Vec3>,
    pub axis: Option<Vec3>,
    pub radius: Option<f64>,
    pub major: Option<f64>,
    pub concave: bool,
    pub hole: bool,
    pub feature: String,
    pub role: &'static str,
    pub entity: Option<String>,
    pub index: Option<u32>,
    pub adj: Vec<String>,
    pub adj_kinds: String,
    pub dup: usize,
    pub inner_loops: usize,
    /// Name of the lump (body) the face is in, when it has one.
    pub body: Option<String>,
}

#[derive(Debug, Clone)]
pub struct EdgeFact {
    pub key: String,
    pub faces: Vec<String>,
    pub curve: &'static str,
    pub length: f64,
    pub mid: Vec3,
    pub dir: Option<Vec3>,
    pub radius: Option<f64>,
    pub center: Option<Vec3>,
    pub axis: Option<Vec3>,
    pub adj_kinds: String,
    /// Name of the lump (body) the edge is in, when it has one.
    pub body: Option<String>,
}

pub struct Facts {
    pub faces: Vec<FaceFact>,
    pub edges: Vec<EdgeFact>,
    pub lo: Vec3,
    pub hi: Vec3,
    pub diag: f64,
}

fn face_fact(body: &Body, f: FaceId, light: bool) -> FaceFact {
    let face = body.face(f);
    if light {
        // only the name and the surface kind (what the neighbours' and the duplicate checks read); no integration
        return FaceFact {
            id: face.name.id(),
            kind: face.surface.kind(),
            area: 0.0,
            centroid: Vec3::ZERO,
            normal: None,
            axis: None,
            radius: None,
            major: None,
            concave: false,
            hole: false,
            feature: String::new(),
            role: "other",
            entity: None,
            index: None,
            adj: vec![],
            adj_kinds: String::new(),
            dup: 1,
            inner_loops: 0,
            body: None,
        };
    }
    let sg = if face.sense { 1.0 } else { -1.0 };
    let m = ck_topo::mass::face_integral::<4>(body, f, 1, &|p, n| {
        let w = n.len();
        [w, p.x * w, p.y * w, p.z * w]
    });
    let area = sg * m[0];
    let centroid = if area.abs() > 1e-300 {
        Vec3::new(sg * m[1] / area, sg * m[2] / area, sg * m[3] / area)
    } else {
        let es = body.face_edges(f);
        let n = es.len().max(1) as f64;
        es.iter().fold(Vec3::ZERO, |a, &e| a + body.edge_mid(e)) * (1.0 / n)
    };
    let (mut normal, mut axis, mut radius, mut major) = (None, None, None, None);
    match &face.surface {
        Surface::Plane { .. } => normal = Some(body.planar_normal(f)),
        Surface::Cylinder { frame, r } => {
            axis = Some(frame.z);
            radius = Some(*r)
        }
        Surface::Cone { frame, r, .. } => {
            axis = Some(frame.z);
            radius = Some(*r)
        }
        Surface::Sphere { r, .. } => radius = Some(*r),
        Surface::Torus { frame, major: a, minor } => {
            axis = Some(frame.z);
            radius = Some(*minor);
            major = Some(*a)
        }
        Surface::Revolution { axis: a, .. } => axis = Some(*a),
        _ => {}
    }
    // concave cylinder (material outside): outward normal points at the axis
    let mut concave = false;
    if let Surface::Cylinder { frame, .. } = &face.surface {
        if let Some(&e) = body.face_edges(f).first() {
            let p = body.edge_mid(e);
            let (u, v) = face.surface.closest_param(p);
            let n = body.outward_normal(f, u, v);
            let d = p - frame.o;
            let radial = d - frame.z * d.dot(frame.z);
            concave = n.dot(radial) < 0.0;
        }
    }
    // a hole barrel: concave cylinder whose non-seam boundary is only full circles
    let mut hole = false;
    if concave {
        let mut sweep = 0.0;
        let mut ok = true;
        for e in body.face_edges(f) {
            if body.is_seam(e) {
                continue;
            }
            let ed = body.edge(e);
            if matches!(ed.curve, Curve::Circle { .. }) {
                sweep += (ed.t1 - ed.t0).abs();
            } else {
                ok = false;
            }
        }
        hole = ok && sweep >= 2.0 * TAU - 1e-3;
    }
    let (role, entity, index) = match &face.name.role {
        Role::Side(e) => ("side", Some(e.clone()), None),
        Role::Cap(k) => ("cap", None, Some(*k)),
        Role::Blend(k) => ("blend", None, Some(*k)),
        Role::Other(s) => ("other", Some(s.clone()), None),
    };
    let id = face.name.id();
    let mut adj: Vec<String> = body.face_neighbors(f).iter().map(|&g| body.face_id_str(g)).filter(|s| *s != id).collect();
    adj.sort();
    adj.dedup();
    FaceFact {
        id,
        kind: face.surface.kind(),
        area,
        centroid,
        normal,
        axis,
        radius,
        major,
        concave,
        hole,
        feature: face.name.feature.clone(),
        role,
        entity,
        index,
        adj,
        adj_kinds: String::new(),
        dup: 1,
        inner_loops: face.loops.len().saturating_sub(1),
        body: body.lump(body.face_lump(f)).name.clone(),
    }
}

/// Exact-ish bounds: edge ends, exact circle-arc extrema, sampled other
/// curves, plus (when `mesh`) the display mesh if a face is free-form / spherical.
pub fn exact_bbox(body: &Body, mesh: bool) -> (Vec3, Vec3) {
    let inf = f64::INFINITY;
    let (mut lo, mut hi) = (Vec3::new(inf, inf, inf), Vec3::new(-inf, -inf, -inf));
    let mut add = |p: Vec3| {
        lo = lo.min(p);
        hi = hi.max(p);
    };
    for v in body.vertex_ids() {
        add(body.vertex(v).p);
    }
    for e in body.edge_ids() {
        let ed = body.edge(e);
        // endpoints from the curve: some bodies (knit results) carry no live vertices
        add(ed.curve.eval(ed.t0));
        add(ed.curve.eval(ed.t1));
        match &ed.curve {
            Curve::Line { .. } => {}
            Curve::Circle { frame, r } => {
                let (t0, t1) = (ed.t0.min(ed.t1), ed.t0.max(ed.t1));
                let in_arc = |t: f64| (t - t0).rem_euclid(TAU) <= (t1 - t0) + 1e-12;
                for i in 0..3 {
                    let (xa, ya) = (comp(frame.x, i), comp(frame.y, i));
                    let ts = ya.atan2(xa);
                    for t in [ts, ts + PI] {
                        if in_arc(t) {
                            add(ed.curve.eval(t));
                        }
                    }
                }
                let _ = r;
            }
            c => {
                for p in c.sample(ed.t0, ed.t1, 128) {
                    add(p);
                }
            }
        }
    }
    let freeform = body.face_ids().any(|f| !matches!(body.face(f).surface, Surface::Plane { .. } | Surface::Cylinder { .. } | Surface::Cone { .. }));
    if mesh && freeform && body.num_faces() > 0 {
        let t = ck_tess::mesh::tessellate(body);
        for p in t.mesh.positions.chunks_exact(3) {
            add(Vec3::new(p[0] as f64, p[1] as f64, p[2] as f64));
        }
    }
    (lo, hi)
}

pub fn gather(body: &Body) -> Facts {
    gather_with(body, None)
}

/// The faces (by name) and edges (by sorted pair of face names) whose facts a caller reads.
pub struct Want<'a> {
    pub faces: &'a [String],
    pub pairs: &'a [Vec<String>],
}

/// [`gather`] for a few faces / edges only: the facts of those are exactly those of `gather`, every other face and edge
/// is left out of the expensive part (face integrals, edge lengths), so `face_sig` / `edge_sig` of the wanted entities
/// give the same signatures at a fraction of the cost on a big body. Other faces carry only their name and surface kind;
/// do not read anything else from them.
pub fn gather_for(body: &Body, want: &Want) -> Facts {
    gather_with(body, Some(want))
}

fn gather_with(body: &Body, want: Option<&Want>) -> Facts {
    let mut faces: Vec<FaceFact> = body
        .face_ids()
        .map(|f| {
            let light = want.is_some_and(|w| {
                let id = body.face(f).name.id();
                !w.faces.iter().any(|x| *x == id)
            });
            face_fact(body, f, light)
        })
        .collect();
    let mut counts: HashMap<String, usize> = HashMap::new();
    for f in &faces {
        *counts.entry(f.id.clone()).or_default() += 1;
    }
    let kinds: HashMap<String, &'static str> = faces.iter().map(|f| (f.id.clone(), f.kind)).collect();
    for f in &mut faces {
        f.dup = counts[&f.id];
        let mut ks: Vec<&str> = f.adj.iter().filter_map(|a| kinds.get(a).copied()).collect();
        ks.sort();
        ks.dedup();
        f.adj_kinds = ks.join(",");
    }
    let mut edges = vec![];
    for (seq, chain) in body.wire_numbered().0 {
        let e = chain[0];
        if body.is_split_rail(e) {
            continue;
        }
        let mut names: Vec<String> = body.edge_faces(e).iter().map(|&f| body.face_id_str(f)).collect();
        names.sort();
        names.dedup();
        if names.is_empty() {
            continue;
        }
        if want.is_some_and(|w| !w.pairs.contains(&names)) {
            continue;
        }
        let key = if names.len() >= 2 {
            ck_topo::body::edge_key(&names[0], &names[1], seq)
        } else {
            format!("{}#{}", names[0], seq)
        };
        let ed = body.edge(e);
        let length: f64 = chain.iter().map(|&c| body.edge_length(c)).sum();
        let (mut dir, mut radius, mut center, mut axis) = (None, None, None, None);
        let curve = match &ed.curve {
            Curve::Line { d, .. } => {
                dir = Some(d.normalize());
                "line"
            }
            Curve::Circle { .. } => {
                let (c, n, r) = ed.curve.circle_info().unwrap();
                (center, axis, radius) = (Some(c), Some(n), Some(r));
                "circle"
            }
            _ => "other",
        };
        let mut ks: Vec<&str> = names.iter().filter_map(|n| kinds.get(n).copied()).collect();
        ks.sort();
        ks.dedup();
        edges.push(EdgeFact {
            key,
            faces: names,
            curve,
            length,
            mid: body.chain_mid(&chain),
            dir,
            radius,
            center,
            axis,
            adj_kinds: ks.join(","),
            body: body.edge_faces(e).first().and_then(|&f| body.lump(body.face_lump(f)).name.clone()),
        });
    }
    let (mut lo, mut hi) = exact_bbox(body, false);
    if !(lo.is_finite() && hi.is_finite()) {
        (lo, hi) = (Vec3::ZERO, Vec3::ZERO);
    }
    let diag = if lo.is_finite() && hi.is_finite() { (hi - lo).len() } else { 0.0 };
    Facts { faces, edges, lo, hi, diag }
}

// ---------------------------------------------------------------- describe

impl Facts {
    /// Which sides of the body's bounding box a planar face lies on.
    fn boundary_sides(&self, f: &FaceFact) -> Vec<String> {
        let tol = 1e-6 * self.diag.max(1.0);
        let mut out = vec![];
        if let Some(n) = f.normal {
            if let Some(i) = axis_letter(n) {
                let pos = comp(f.centroid, i);
                let up = comp(n, i) > 0.0;
                let lim = if up { comp(self.hi, i) } else { comp(self.lo, i) };
                if (pos - lim).abs() <= tol {
                    out.push(format!("{}{}", if up { "+" } else { "-" }, ["x", "y", "z"][i]));
                }
            }
        }
        out
    }

    pub fn face_json(&self, f: &FaceFact) -> Json {
        let mut o = Obj::new()
            .put("name", f.id.as_str())
            .put("ref", Obj::new().put("kind", "face").put("id", f.id.as_str()).build())
            .put("kind", "face")
            .put("surface", f.kind)
            .put("area", rnd(f.area))
            .put("centroid", vj(f.centroid))
            .put("feature", f.feature.as_str())
            .put("role", f.role);
        if let Some(e) = &f.entity {
            o = o.put("entity", e.as_str());
        }
        if let Some(i) = f.index {
            o = o.put("index", i as f64);
        }
        if let Some(n) = f.normal {
            o = o.put("normal", dir_label(n));
        }
        if let Some(a) = f.axis {
            o = o.put("axis", dir_label(a));
        }
        if let Some(r) = f.radius {
            o = o.put("radius", rnd(r));
        }
        if f.kind == "cylinder" {
            o = o.put("concave", f.concave).put("hole", f.hole);
        }
        let sides = self.boundary_sides(f);
        if !sides.is_empty() {
            o = o.put("outermost", Json::Arr(sides.iter().map(|s| Json::from(s.as_str())).collect()));
        }
        o = o.put("adjacent", Json::Arr(f.adj.iter().map(|s| Json::from(s.as_str())).collect()));
        o = o.put("text", self.face_text(f, &sides));
        o.build()
    }

    fn face_text(&self, f: &FaceFact, sides: &[String]) -> String {
        let mut s = match f.kind {
            "plane" => format!("planar face {}: normal {}", f.id, f.normal.map(dir_label).unwrap_or_default()),
            "cylinder" => format!(
                "{} cylinder face {}: radius {}, axis {}",
                if f.hole { "hole barrel (concave)" } else if f.concave { "concave" } else { "convex" },
                f.id,
                fmt_num(f.radius.unwrap_or(0.0)),
                f.axis.map(dir_label).unwrap_or_default()
            ),
            k => format!("{} face {}", k, f.id),
        };
        if f.kind != "cylinder" {
            if let (Some(r), true) = (f.radius, f.kind != "plane") {
                s += &format!(", radius {}", fmt_num(r));
            }
            if let (Some(a), true) = (f.axis, f.kind != "plane") {
                s += &format!(", axis {}", dir_label(a));
            }
        }
        s += &format!(", area {}, centre {}", fmt_num(f.area), fmt_pt(f.centroid));
        if !sides.is_empty() {
            s += &format!(" (outermost {} face of the body)", sides.join(" "));
        }
        s += &match f.role {
            "cap" => format!("; cap {} of {}", f.index.unwrap_or(0), f.feature),
            "side" => format!("; side face of sketch entity {} in {}", f.entity.as_deref().unwrap_or("?"), f.feature),
            "blend" => format!("; blend face {} of {}", f.index.unwrap_or(0), f.feature),
            _ => format!("; {} of {}", f.entity.as_deref().unwrap_or("face"), f.feature),
        };
        if !f.adj.is_empty() {
            s += &format!("; adjacent to {}", f.adj.join(", "));
        }
        if f.inner_loops > 0 {
            s += &format!("; {} inner loop(s)", f.inner_loops);
        }
        s
    }

    pub fn edge_json(&self, e: &EdgeFact) -> Json {
        let mut o = Obj::new().put("name", e.key.as_str());
        let r = Obj::new()
            .put("faces", Json::Arr(e.faces.iter().map(|s| Json::from(s.as_str())).collect()))
            .put("at", vj(e.mid))
            .build();
        o = o
            .put("ref", r)
            .put("kind", "edge")
            .put("curve", e.curve)
            .put("length", rnd(e.length))
            .put("mid", vj(e.mid))
            .put("faces", Json::Arr(e.faces.iter().map(|s| Json::from(s.as_str())).collect()));
        if let Some(d) = e.dir {
            o = o.put("dir", dir_label(d));
        }
        if let Some(r) = e.radius {
            o = o.put("radius", rnd(r)).put("center", vj(e.center.unwrap())).put("axis", dir_label(e.axis.unwrap()));
        }
        o = o.put("text", self.edge_text(e));
        o.build()
    }

    fn edge_text(&self, e: &EdgeFact) -> String {
        let mut s = match e.curve {
            "line" => format!("line edge {}: length {}, direction {}", e.key, fmt_num(e.length), e.dir.map(dir_label).unwrap_or_default()),
            "circle" => format!(
                "circle edge {}: radius {}, axis {}, length {}",
                e.key,
                fmt_num(e.radius.unwrap_or(0.0)),
                e.axis.map(dir_label).unwrap_or_default(),
                fmt_num(e.length)
            ),
            _ => format!("curved edge {}: length {}", e.key, fmt_num(e.length)),
        };
        s += &format!(", midpoint {}", fmt_pt(e.mid));
        s += &if e.faces.len() >= 2 {
            format!("; between {} and {}", e.faces[0], e.faces[1])
        } else {
            format!("; boundary of {}", e.faces[0])
        };
        s
    }

    fn face_by_id(&self, id: &str) -> Option<&FaceFact> {
        self.faces.iter().find(|f| f.id == id)
    }
}

fn similarity(a: &str, b: &str) -> usize {
    let fa = a.split('.').next().unwrap_or("");
    let fb = b.split('.').next().unwrap_or("");
    let common = a.bytes().zip(b.bytes()).take_while(|(x, y)| x == y).count();
    common + if fa == fb { 5 } else { 0 }
}

/// `describe {names:[..]}`: each name is a face id, an edge key
/// (`A|B#seq`), `{kind:"face",id}` or an edge ref `{faces:[a,b],at}`.
pub fn describe(body: &Body, msg: &Json) -> Json {
    let facts = gather(body);
    let names: Vec<&Json> = match msg.get("names") {
        Some(Json::Arr(a)) => a.iter().collect(),
        Some(j @ Json::Str(_)) => vec![j],
        _ => vec![],
    };
    let items: Vec<Json> = names.into_iter().map(|n| describe_one(&facts, n)).collect();
    Obj::new().put("items", Json::Arr(items)).build()
}

fn near_names(facts: &Facts, name: &str, edge: bool) -> Vec<Json> {
    let mut c: Vec<(usize, &str)> = if edge {
        let parts: Vec<&str> = name.split('#').next().unwrap_or("").split('|').collect();
        facts
            .edges
            .iter()
            .filter(|e| parts.iter().any(|p| e.faces.iter().any(|f| f == p)))
            .map(|e| (similarity(name, &e.key), e.key.as_str()))
            .collect()
    } else {
        facts.faces.iter().map(|f| (similarity(name, &f.id), f.id.as_str())).collect()
    };
    c.sort_by(|a, b| b.0.cmp(&a.0).then(a.1.cmp(b.1)));
    c.dedup_by(|a, b| a.1 == b.1);
    c.into_iter().take(3).map(|(_, n)| Json::from(n)).collect()
}

fn describe_one(facts: &Facts, n: &Json) -> Json {
    let lost = |name: &str, edge: bool, why: &str| {
        Obj::new()
            .put("name", name)
            .put("found", false)
            .put("error", why)
            .put("nearest", Json::Arr(near_names(facts, name, edge)))
            .build()
    };
    // normalise to (name, is_edge, optional hint point)
    let (name, edge, at): (String, bool, Option<Vec3>) = match n {
        Json::Str(s) => (s.clone(), s.contains('|') || s.contains('#'), None),
        Json::Obj(_) => match n.get_str("kind") {
            Some("face") => (n.get_str("id").unwrap_or("").to_string(), false, None),
            _ => {
                let fs: Vec<String> =
                    n.get_arr("faces").map(|a| a.iter().filter_map(|j| j.as_str().map(String::from)).collect()).unwrap_or_default();
                let at = n.get("at").and_then(Json::as_vec3).map(Vec3::from_array);
                if fs.len() == 2 {
                    let mut s = fs.clone();
                    s.sort();
                    let cands: Vec<&EdgeFact> = facts.edges.iter().filter(|e| e.faces == s).collect();
                    let pick = match at {
                        Some(p) => cands.iter().copied().min_by(|a, b| a.mid.dist(p).total_cmp(&b.mid.dist(p))),
                        None => cands.first().copied(),
                    };
                    return match pick {
                        Some(e) => {
                            let mut j = facts.edge_json(e);
                            j.set("found", true);
                            if cands.len() > 1 && at.is_none() {
                                j.set("ambiguous", true);
                            }
                            j
                        }
                        None => lost(&format!("{}|{}", s[0], s[1]), true, "no edge between these faces"),
                    };
                }
                (String::new(), false, None)
            }
        },
        _ => (String::new(), false, None),
    };
    let _ = at;
    if name.is_empty() {
        return Obj::new().put("name", Json::Null).put("found", false).put("error", "expected a face id, an edge key or a ref object").build();
    }
    if edge {
        return match facts.edges.iter().find(|e| e.key == name) {
            Some(e) => {
                let mut j = facts.edge_json(e);
                j.set("found", true);
                j
            }
            None => lost(&name, true, "edge key not found (a topology change renumbers the #seq suffix)"),
        };
    }
    match facts.face_by_id(&name) {
        Some(f) => {
            let mut j = facts.face_json(f);
            j.set("found", true);
            if f.dup > 1 {
                j.set("ambiguous", true);
            }
            j
        }
        None => lost(&name, false, "face id not found"),
    }
}

// ---------------------------------------------------------------- resolve

const FACE_KEYS: &[&str] = &[
    "kind", "surface", "normal", "axis", "radius", "role", "entity", "concave", "hole", "feature", "name", "touches", "area", "near",
    "within", "extreme", "along", "body",
];
const EDGE_KEYS: &[&str] =
    &["kind", "curve", "dir", "radius", "length", "feature", "name", "touches", "between", "near", "within", "extreme", "along", "body"];

fn parse_dir(j: &Json) -> Result<(Vec3, bool), String> {
    match j {
        Json::Str(s) => {
            let s = s.to_ascii_lowercase();
            let (signed, sign, rest) = match s.strip_prefix('+') {
                Some(r) => (true, 1.0, r.to_string()),
                None => match s.strip_prefix('-') {
                    Some(r) => (true, -1.0, r.to_string()),
                    None => (false, 1.0, s.clone()),
                },
            };
            let v = match rest.as_str() {
                "x" => Vec3::X,
                "y" => Vec3::Y,
                "z" => Vec3::Z,
                _ => return Err(format!("bad direction {s:?} (use x|y|z with optional +/-, or a [x,y,z] vector)")),
            };
            Ok((v * sign, signed))
        }
        Json::Arr(_) => {
            let a = j.as_vec3().ok_or("direction vector must be [x,y,z]")?;
            let v = Vec3::from_array(a);
            if v.len() < 1e-12 {
                return Err("zero direction vector".into());
            }
            Ok((v.normalize(), true))
        }
        _ => Err("direction must be a string or [x,y,z]".into()),
    }
}

fn dir_matches(have: Vec3, want: (Vec3, bool)) -> bool {
    let d = have.normalize().dot(want.0);
    if want.1 {
        d >= COS_TOL
    } else {
        d.abs() >= COS_TOL
    }
}

/// number spec: `5`, `{value, tol}` or `{min, max}`
fn parse_num(j: &Json) -> Result<(f64, f64), String> {
    match j {
        Json::Num(x) => Ok((*x - 1e-6 * x.abs().max(1e-3), *x + 1e-6 * x.abs().max(1e-3))),
        Json::Obj(_) => {
            if let Some(v) = j.get_f64("value") {
                let t = j.get_f64("tol").unwrap_or(1e-6 * v.abs().max(1e-3));
                Ok((v - t, v + t))
            } else {
                Ok((j.get_f64("min").unwrap_or(f64::NEG_INFINITY), j.get_f64("max").unwrap_or(f64::INFINITY)))
            }
        }
        _ => Err("number spec must be a number, {value,tol} or {min,max}".into()),
    }
}

fn str_list(j: &Json) -> Result<Vec<String>, String> {
    match j {
        Json::Str(s) => Ok(vec![s.clone()]),
        Json::Arr(a) => a.iter().map(|x| x.as_str().map(String::from).ok_or_else(|| "expected strings".to_string())).collect(),
        _ => Err("expected a string or array of strings".into()),
    }
}

fn glob(pat: &str, s: &str) -> bool {
    match pat.strip_suffix('*') {
        Some(p) => s.starts_with(p),
        None => s == pat,
    }
}

struct Narrow(Vec<Json>);
impl Narrow {
    fn step(&mut self, c: &mut Vec<usize>, label: &str, mut keep: impl FnMut(usize) -> bool) {
        c.retain(|&i| keep(i));
        self.0.push(Obj::new().put("filter", label).put("remaining", c.len() as f64).build());
    }
}

/// `resolve {descriptions:[..]}` (or `description: {..}`).
pub fn resolve(body: &Body, msg: &Json) -> Json {
    let facts = gather(body);
    let list: Vec<&Json> = match (msg.get("descriptions"), msg.get("description")) {
        (Some(Json::Arr(a)), _) => a.iter().collect(),
        (_, Some(d)) => vec![d],
        _ => vec![],
    };
    let results: Vec<Json> = list.into_iter().map(|d| resolve_one(&facts, d)).collect();
    Obj::new().put("results", Json::Arr(results)).build()
}

fn invalid(d: &Json, why: String) -> Json {
    Obj::new()
        .put("description", d.clone())
        .put("status", "invalid")
        .put("ambiguous", false)
        .put("count", 0.0)
        .put("matches", Json::Arr(vec![]))
        .put("error", why)
        .build()
}

fn resolve_one(facts: &Facts, d: &Json) -> Json {
    let Json::Obj(map) = d else { return invalid(d, "a description must be an object".into()) };
    let kind = d.get_str("kind").unwrap_or("");
    let allowed = match kind {
        "face" => FACE_KEYS,
        "edge" => EDGE_KEYS,
        _ => return invalid(d, "kind must be \"face\" or \"edge\"".into()),
    };
    for (k, _) in map.iter() {
        if !allowed.contains(&k.as_str()) {
            return invalid(d, format!("unknown field {k:?} for kind {kind:?} (allowed: {})", allowed.join(", ")));
        }
    }
    match resolve_inner(facts, d, kind == "edge") {
        Ok(j) => j,
        Err(e) => invalid(d, e),
    }
}

fn resolve_inner(facts: &Facts, d: &Json, edge: bool) -> Result<Json, String> {
    let n = if edge { facts.edges.len() } else { facts.faces.len() };
    let mut c: Vec<usize> = (0..n).collect();
    let mut nar = Narrow(vec![]);
    let g = |k: &str| d.get(k);
    let eps = 1e-6 * facts.diag.max(1.0);

    // shared
    if let Some(j) = g("feature") {
        let fs = str_list(j)?;
        if edge {
            nar.step(&mut c, "feature", |i| {
                facts.edges[i].faces.iter().any(|f| fs.iter().any(|w| f.split('.').next() == Some(w.as_str())))
            });
        } else {
            nar.step(&mut c, "feature", |i| fs.contains(&facts.faces[i].feature));
        }
    }
    if let Some(j) = g("body") {
        let names = str_list(j)?;
        nar.step(&mut c, "body", |i| {
            let b = if edge { &facts.edges[i].body } else { &facts.faces[i].body };
            b.as_ref().is_some_and(|b| names.contains(b))
        });
    }
    if let Some(j) = g("name") {
        let pats = str_list(j)?;
        nar.step(&mut c, "name", |i| {
            let nm = if edge { &facts.edges[i].key } else { &facts.faces[i].id };
            pats.iter().any(|p| glob(p, nm))
        });
    }
    if let Some(j) = g("touches") {
        let w = j.as_str().ok_or("touches must be a face name")?.to_string();
        if edge {
            nar.step(&mut c, "touches", |i| facts.edges[i].faces.contains(&w));
        } else {
            nar.step(&mut c, "touches", |i| facts.faces[i].adj.contains(&w));
        }
    }
    if let Some(j) = g("radius") {
        let (lo, hi) = parse_num(j)?;
        if edge {
            nar.step(&mut c, "radius", |i| facts.edges[i].radius.is_some_and(|r| r >= lo && r <= hi));
        } else {
            nar.step(&mut c, "radius", |i| facts.faces[i].radius.is_some_and(|r| r >= lo && r <= hi));
        }
    }

    let mut along_hint: Option<usize> = None;
    if edge {
        if let Some(j) = g("curve") {
            let cs = str_list(j)?;
            nar.step(&mut c, "curve", |i| cs.iter().any(|w| w == facts.edges[i].curve));
        }
        if let Some(j) = g("dir") {
            let w = parse_dir(j)?;
            along_hint = axis_letter(w.0);
            nar.step(&mut c, "dir", |i| facts.edges[i].dir.is_some_and(|v| dir_matches(v, w)));
        }
        if let Some(j) = g("length") {
            let (lo, hi) = parse_num(j)?;
            nar.step(&mut c, "length", |i| facts.edges[i].length >= lo && facts.edges[i].length <= hi);
        }
        if let Some(j) = g("between") {
            let fs = str_list(j)?;
            if fs.len() != 2 {
                return Err("between must list exactly two face names".into());
            }
            nar.step(&mut c, "between", |i| fs.iter().all(|w| facts.edges[i].faces.contains(w)));
        }
    } else {
        if let Some(j) = g("surface") {
            let ss = str_list(j)?;
            nar.step(&mut c, "surface", |i| ss.iter().any(|w| w == facts.faces[i].kind));
        }
        if let Some(j) = g("normal") {
            let w = parse_dir(j)?;
            along_hint = axis_letter(w.0);
            nar.step(&mut c, "normal", |i| facts.faces[i].normal.is_some_and(|v| dir_matches(v, w)));
        }
        if let Some(j) = g("axis") {
            let w = parse_dir(j)?;
            along_hint = along_hint.or(axis_letter(w.0));
            nar.step(&mut c, "axis", |i| facts.faces[i].axis.is_some_and(|v| dir_matches(v, w)));
        }
        if let Some(j) = g("role") {
            let rs = str_list(j)?;
            nar.step(&mut c, "role", |i| rs.iter().any(|w| w == facts.faces[i].role));
        }
        if let Some(j) = g("entity") {
            let es = str_list(j)?;
            nar.step(&mut c, "entity", |i| facts.faces[i].entity.as_ref().is_some_and(|e| es.contains(e)));
        }
        if let Some(j) = g("concave") {
            let w = j.as_bool().ok_or("concave must be true/false")?;
            nar.step(&mut c, "concave", |i| facts.faces[i].kind == "cylinder" && facts.faces[i].concave == w);
        }
        if let Some(j) = g("hole") {
            let w = j.as_bool().ok_or("hole must be true/false")?;
            nar.step(&mut c, "hole", |i| facts.faces[i].hole == w);
        }
        if let Some(j) = g("area") {
            let (lo, hi) = parse_num(j)?;
            nar.step(&mut c, "area", |i| facts.faces[i].area >= lo && facts.faces[i].area <= hi);
        }
    }
    let pos = |i: usize| if edge { facts.edges[i].mid } else { facts.faces[i].centroid };

    if let Some(j) = g("extreme") {
        let which = j.as_str().filter(|s| *s == "max" || *s == "min").ok_or("extreme must be \"max\" or \"min\"")?;
        let along = match g("along") {
            Some(a) => {
                let (v, _) = parse_dir(a)?;
                axis_letter(v).ok_or("along must be x, y or z")?
            }
            None => along_hint.ok_or("extreme needs \"along\": \"x\"|\"y\"|\"z\" (it cannot be inferred without an axis-aligned normal/axis/dir)")?,
        };
        if !c.is_empty() {
            let best = c
                .iter()
                .map(|&i| comp(pos(i), along))
                .fold(if which == "max" { f64::NEG_INFINITY } else { f64::INFINITY }, |a, b| if which == "max" { a.max(b) } else { a.min(b) });
            nar.step(&mut c, "extreme", |i| (comp(pos(i), along) - best).abs() <= eps);
        }
    } else if g("along").is_some() {
        return Err("\"along\" is only meaningful together with \"extreme\"".into());
    }

    let mut tie_broken = false;
    let mut tie_msg: Option<String> = None;
    if let Some(j) = g("near") {
        let p = Vec3::from_array(j.as_vec3().ok_or("near must be [x,y,z]")?);
        if let Some(w) = g("within") {
            let r = w.as_f64().ok_or("within must be a number")?;
            nar.step(&mut c, "within", |i| pos(i).dist(p) <= r);
        }
        if c.len() > 1 {
            c.sort_by(|&a, &b| pos(a).dist(p).total_cmp(&pos(b).dist(p)));
            let (d0, d1) = (pos(c[0]).dist(p), pos(c[1]).dist(p));
            if d1 - d0 > eps.max(1e-9) {
                c.truncate(1);
                tie_broken = true;
            } else {
                tie_msg = Some("near does not separate the candidates (equidistant)".into());
            }
            nar.0.push(Obj::new().put("filter", "near").put("remaining", c.len() as f64).build());
        }
    } else if g("within").is_some() {
        return Err("\"within\" needs \"near\"".into());
    }

    // duplicate ids make a face name itself ambiguous
    let dup = !edge && c.iter().any(|&i| facts.faces[i].dup > 1);
    let mut names: Vec<Json> = c.iter().map(|&i| if edge { facts.edge_json(&facts.edges[i]) } else { facts.face_json(&facts.faces[i]) }).collect();
    if names.len() > 1 {
        names.sort_by(|a, b| a.get_str("name").cmp(&b.get_str("name")));
    }
    let count = names.len();
    let status = match count {
        0 => "none",
        1 if !dup => "unique",
        _ => "ambiguous",
    };
    let mut o = Obj::new()
        .put("description", d.clone())
        .put("status", status)
        .put("ambiguous", status == "ambiguous")
        .put("count", count as f64);
    if tie_broken {
        o = o.put("resolvedBy", "near");
    }
    if status == "none" {
        o = o.put("hint", "no entity matches; see narrowing for the filter that removed the last candidates");
    }
    if status == "ambiguous" {
        o = o.put(
            "hint",
            tie_msg.unwrap_or_else(|| "several entities match; add feature/touches/extreme+along/near to narrow, or use a returned name explicitly".into()),
        );
    }
    Ok(o.put("matches", Json::Arr(names)).put("narrowing", Json::Arr(nar.0)).build())
}

// ---------------------------------------------------------------- check

/// Measured intent facts of the body.
pub struct Measured {
    pub volume: f64,
    pub area: f64,
    pub com: Vec3,
    pub lo: Vec3,
    pub hi: Vec3,
    pub faces: usize,
    pub edges: usize,
    pub vertices: usize,
    pub solids: usize,
    pub holes: usize,
    pub inner_loops: usize,
}

pub fn measure_body(body: &Body, facts: &Facts) -> Measured {
    let mp = ck_topo::mass::mass_props(body);
    let solids = ck_topo::mass::lump_moments(body).iter().filter(|m| m.volume > 0.0).count();
    Measured {
        volume: mp.volume,
        area: mp.area,
        com: mp.com,
        lo: facts.lo,
        hi: facts.hi,
        faces: facts.faces.len(),
        edges: facts.edges.len(),
        vertices: body.vertex_ids().count(),
        solids,
        holes: facts.faces.iter().filter(|f| f.hole).count(),
        inner_loops: facts.faces.iter().map(|f| f.inner_loops).sum(),
    }
}

const COUNT_KINDS: &[&str] = &["faces", "edges", "vertices", "solids", "holes", "innerLoops"];
const SCALAR_KINDS: &[&str] = &["volume", "area", "mass"];

fn range_of(e: &Json, default_rel: f64, default_abs: f64, scale: f64) -> Option<(f64, f64, Json)> {
    let abs = e.get_f64("tol");
    let rel = e.get_f64("relTol");
    if let Some(v) = e.get_f64("equals").or_else(|| e.get_f64("value")) {
        let t = match (abs, rel) {
            (None, None) => (default_rel * v.abs()).max(default_abs),
            (a, r) => a.unwrap_or(0.0).max(r.unwrap_or(0.0) * v.abs()),
        };
        let _ = scale;
        return Some((v - t, v + t, Obj::new().put("equals", v).put("tol", t).build()));
    }
    let (lo, hi) = (e.get_f64("min"), e.get_f64("max"));
    if lo.is_none() && hi.is_none() {
        return None;
    }
    let mut o = Obj::new();
    if let Some(l) = lo {
        o = o.put("min", l);
    }
    if let Some(h) = hi {
        o = o.put("max", h);
    }
    Some((lo.unwrap_or(f64::NEG_INFINITY), hi.unwrap_or(f64::INFINITY), o.build()))
}

fn arr3(e: &Json, k: &str) -> Option<[f64; 3]> {
    e.get(k).and_then(Json::as_vec3)
}

/// Normalise `expect` (array of objects, or `{volume: 1000, faces: 6, ...}`).
fn expectations(msg: &Json) -> Vec<Json> {
    match msg.get("expect") {
        Some(Json::Arr(a)) => a.clone(),
        Some(Json::Obj(m)) => m
            .iter()
            .map(|(k, v)| match v {
                Json::Obj(_) => v.clone().with("kind", k.as_str()),
                Json::Num(n) => Obj::new().put("kind", k.as_str()).put("equals", *n).build(),
                other => Obj::new().put("kind", k.as_str()).put("equals", other.clone()).build(),
            })
            .collect(),
        _ => vec![],
    }
}

/// `check {expect:[..]}` -> per-expectation pass/fail with the measured value.
/// `valid_issues` lazily counts validator issues for `{kind:"valid"}`.
pub fn check(body: &Body, msg: &Json, valid_issues: &dyn Fn() -> usize) -> Json {
    let facts = gather(body);
    let exps = expectations(msg);
    let mut m = measure_body(body, &facts);
    if exps.iter().any(|e| e.get_str("kind") == Some("bbox")) {
        // free-form faces need the display mesh; only paid for when a bbox is asked
        (m.lo, m.hi) = exact_bbox(body, true);
    }
    let mut results = vec![];
    for (i, e) in exps.iter().enumerate() {
        let kind = e.get_str("kind").unwrap_or("").to_string();
        let mut o = Obj::new();
        if let Some(id) = e.get_str("id") {
            o = o.put("id", id);
        }
        o = o.put("index", i as f64).put("kind", kind.as_str());
        let diag = (m.hi - m.lo).len().max(1.0);
        let (pass, measured, expected, detail): (bool, Json, Json, String) = match kind.as_str() {
            k if SCALAR_KINDS.contains(&k) => {
                let (val, err) = match k {
                    "volume" => (m.volume, None),
                    "area" => (m.area, None),
                    _ => match e.get_f64("density") {
                        Some(d) => (m.volume * d, None),
                        None => (f64::NAN, Some("mass needs \"density\" (mass per unit volume)")),
                    },
                };
                match (err, range_of(e, 1e-4, 1e-9, 1.0)) {
                    (Some(er), _) => (false, Json::Null, Json::Null, er.to_string()),
                    (None, None) => (false, rnd9(val), Json::Null, "give equals, or min and/or max".into()),
                    (None, Some((lo, hi, ex))) => {
                        let ok = val >= lo && val <= hi;
                        (ok, rnd9(val), ex, format!("{k} {} vs [{}, {}]", fmt_num(val), fmt_num(lo), fmt_num(hi)))
                    }
                }
            }
            k if COUNT_KINDS.contains(&k) => {
                let val = match k {
                    "faces" => m.faces,
                    "edges" => m.edges,
                    "vertices" => m.vertices,
                    "solids" => m.solids,
                    "holes" => m.holes,
                    _ => m.inner_loops,
                } as f64;
                match range_of(e, 0.0, 0.0, 1.0) {
                    None => (false, Json::Num(val), Json::Null, "give equals, or min and/or max".into()),
                    Some((lo, hi, ex)) => {
                        let ok = val >= lo - 1e-9 && val <= hi + 1e-9;
                        (ok, Json::Num(val), ex, format!("{k} {} vs [{}, {}]", fmt_num(val), fmt_num(lo), fmt_num(hi)))
                    }
                }
            }
            "bbox" => {
                let tol = e.get_f64("tol").unwrap_or(1e-4 * diag);
                let size = m.hi - m.lo;
                let mut ok = true;
                let mut why = vec![];
                let mut ex = Obj::new();
                for (key, have) in [("min", m.lo), ("max", m.hi), ("size", size)] {
                    if let Some(w) = arr3(e, key) {
                        ex = ex.put(key, Json::Arr(w.iter().map(|&x| Json::Num(x)).collect()));
                        for ax in 0..3 {
                            if (comp(have, ax) - w[ax]).abs() > tol {
                                ok = false;
                                why.push(format!("{key}.{} is {} not {}", ["x", "y", "z"][ax], fmt_num(comp(have, ax)), fmt_num(w[ax])));
                            }
                        }
                    }
                }
                let meas = Obj::new().put("min", vj(m.lo)).put("max", vj(m.hi)).put("size", vj(size)).build();
                if arr3(e, "min").is_none() && arr3(e, "max").is_none() && arr3(e, "size").is_none() {
                    (false, meas, Json::Null, "give min, max and/or size as [x,y,z]".into())
                } else {
                    (ok, meas, ex.put("tol", tol).build(), if ok { "bbox matches".into() } else { why.join("; ") })
                }
            }
            "com" => match arr3(e, "at") {
                None => (false, vj(m.com), Json::Null, "give at:[x,y,z]".into()),
                Some(w) => {
                    let tol = e.get_f64("tol").unwrap_or(1e-4 * diag);
                    let d = m.com.dist(Vec3::from_array(w));
                    (d <= tol, vj(m.com), Obj::new().put("at", vj(Vec3::from_array(w))).put("tol", tol).build(), format!("centre of mass is {} from the expected point", fmt_num(d)))
                }
            },
            "valid" => {
                let n = valid_issues();
                (n == 0, Json::Num(n as f64), Json::from("0 validator issues"), format!("{n} validator issue(s)"))
            }
            "" => (false, Json::Null, Json::Null, "expectation has no kind".into()),
            other => (
                false,
                Json::Null,
                Json::Null,
                format!("unknown expectation kind {other:?} (volume, area, mass, bbox, com, faces, edges, vertices, solids, holes, innerLoops, valid)"),
            ),
        };
        o = o.put("pass", pass).put("measured", measured).put("expected", expected).put("detail", detail);
        results.push(o.build());
    }
    let failed = results.iter().filter(|r| r.get_bool("pass") != Some(true)).count();
    Obj::new()
        .put("ok", failed == 0)
        .put("passed", (results.len() - failed) as f64)
        .put("failed", failed as f64)
        .put("results", Json::Arr(results))
        .put(
            "measured",
            Obj::new()
                .put("volume", rnd(m.volume))
                .put("area", rnd(m.area))
                .put("com", vj(m.com))
                .put("bbox", Obj::new().put("min", vj(m.lo)).put("max", vj(m.hi)).put("size", vj(m.hi - m.lo)))
                .put("faces", m.faces as f64)
                .put("edges", m.edges as f64)
                .put("vertices", m.vertices as f64)
                .put("solids", m.solids as f64)
                .put("holes", m.holes as f64)
                .put("innerLoops", m.inner_loops as f64),
        )
        .build()
}

fn rnd9(x: f64) -> Json {
    Json::Num(rnd(x))
}

// ---------------------------------------------------------------- reference signatures

/// Geometric signature of a referenced entity, for "did this reference
/// silently point somewhere else" comparisons (same rules as
/// tools/naming_stability.mjs).
#[derive(Debug, Clone)]
pub struct Sig {
    pub is_edge: bool,
    pub kind: &'static str,
    pub dir: Vec3,
    pub planar: bool,
    pub adj: String,
    pub size: f64,
    pub pos: Vec3,
    pub diag: f64,
    pub lo: Vec3,
    pub hi: Vec3,
    pub dup: usize,
    pub text: String,
}

pub fn face_sig(facts: &Facts, id: &str) -> Option<Sig> {
    let f = facts.face_by_id(id)?;
    Some(Sig {
        is_edge: false,
        kind: f.kind,
        dir: f.normal.or(f.axis).unwrap_or(Vec3::ZERO),
        planar: f.normal.is_some(),
        adj: f.adj_kinds.clone(),
        size: f.area,
        pos: f.centroid,
        diag: facts.diag,
        lo: facts.lo,
        hi: facts.hi,
        dup: f.dup,
        text: facts.face_text(f, &facts.boundary_sides(f)),
    })
}

/// Edge between `faces` nearest `at` (the kernel's edge-reference rule).
pub fn edge_sig(facts: &Facts, faces: &[String], at: Option<Vec3>) -> Option<Sig> {
    let mut want = faces.to_vec();
    want.sort();
    want.dedup();
    let c: Vec<&EdgeFact> = facts.edges.iter().filter(|e| e.faces == want).collect();
    let e = match at {
        Some(p) => c.iter().copied().min_by(|a, b| a.mid.dist(p).total_cmp(&b.mid.dist(p)))?,
        None => *c.first()?,
    };
    Some(Sig {
        is_edge: true,
        kind: e.curve,
        dir: e.dir.unwrap_or(Vec3::ZERO),
        planar: false,
        adj: e.adj_kinds.clone(),
        size: e.length,
        pos: e.mid,
        diag: facts.diag,
        lo: facts.lo,
        hi: facts.hi,
        dup: 1,
        text: facts.edge_text(e),
    })
}

fn ratio_ok(a: f64, b: f64) -> bool {
    let r = if b.abs() > 1e-9 { a / b } else { 1.0 };
    r > 0.25 && r < 4.0
}

/// Why `after` is not "the same entity" as `before`, or `None` when it is.
pub fn sig_diff(before: &Sig, after: &Sig) -> Option<String> {
    if before.is_edge != after.is_edge {
        return Some("entity type".into());
    }
    if after.dup > 1 {
        return Some("duplicate id".into());
    }
    if before.kind != after.kind {
        return Some(format!("kind {} -> {}", before.kind, after.kind));
    }
    let d = before.dir.dot(after.dir);
    if before.is_edge {
        if before.kind == "line" && d.abs() < 0.999 {
            return Some("direction changed".into());
        }
    } else if (before.planar && d < 0.999) || (!before.planar && d.abs() < 0.99) {
        return Some(format!("direction {} -> {}", dir_label(before.dir), dir_label(after.dir)));
    }
    if before.adj != after.adj {
        return Some(format!("neighbour kinds {{{}}} -> {{{}}}", before.adj, after.adj));
    }
    if !ratio_ok(after.size, before.size) {
        return Some(format!("{} {} -> {}", if before.is_edge { "length" } else { "area" }, fmt_num(before.size), fmt_num(after.size)));
    }
    let frac = |s: &Sig| -> [f64; 3] {
        let f = |i: usize| {
            let w = comp(s.hi, i) - comp(s.lo, i);
            if w > 1e-9 {
                (comp(s.pos, i) - comp(s.lo, i)) / w
            } else {
                0.5
            }
        };
        [f(0), f(1), f(2)]
    };
    let (fb, fa) = (frac(before), frac(after));
    let fmax = (0..3).map(|i| (fb[i] - fa[i]).abs()).fold(0.0, f64::max);
    let amax = (0..3).map(|i| (comp(before.pos, i) - comp(after.pos, i)).abs()).fold(0.0, f64::max);
    if fmax > 0.2 && amax > 0.07 * before.diag.max(after.diag) {
        return Some(format!("position {} -> {}", fmt_pt(before.pos), fmt_pt(after.pos)));
    }
    None
}
