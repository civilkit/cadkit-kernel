//! The `measure` request: per-reference items (faces / edges) and, for
//! exactly two references, their minimum distance, angle and offsets.
//!
//! Wire shape (oracle probes, tools/measure_golden.mjs):
//! `{refs:[{kind:"face",id}|{kind:"edge",faces:[a,b],at}]}` ->
//! `{items:[..], distance?, points?, angle?, normalDistance?, centerDistance?}`
//! or `null` when a reference does not resolve (also for any other kind).
//!
//! * face item: `kind surface area` + plane `normal` (sense applied) /
//!   cylinder `radius center axis` / cone `center axis` / sphere `radius
//!   center` (torus and the free-form kinds carry nothing more);
//! * edge item: `kind length` + line `direction` / circle `radius center
//!   axis`, then `wedge` (angle between the outward normals of the two
//!   faces at the edge mid parameter) when the edge has two coedges;
//! * pair distance (wasm f297/f238/f239): sample both references (edge: 65
//!   parameter samples; face: boundary samples, then a 17x17 uv grid inside
//!   the face), project every sample onto the other reference, stable-sort
//!   the pairs by distance and refine the best three by alternating
//!   projection (<= 100 steps, stop when the gain falls under
//!   `(d + 1) * 1e-13`); the first reference's point comes first.
//!   RECONSTRUCTED from the wasm structure: the sample counts / order of
//!   the boundary samples are GUESSES (they only matter for ties).

use crate::boolean::classify::{face_loops_uv, in_loops};
use crate::boolean::imprint::dist_in_range;
use ck_core::linalg::Vec3;
use ck_geom::Vec2;
use ck_geom::curve::Curve;
use ck_geom::surface::Surface;
use ck_json::{Json, Obj};
use ck_topo::body::{Body, EdgeId, FaceId};

#[derive(Clone, Copy, PartialEq)]
pub enum Ref {
    Face(FaceId),
    Edge(EdgeId),
}

/// What the pair logic needs from an item.
#[derive(Default)]
struct Item {
    is_edge: bool,
    normal: Option<Vec3>,
    direction: Option<Vec3>,
    axis: Option<Vec3>,
    center: Option<Vec3>,
    /// plane origin (for `normalDistance`)
    plane_o: Option<Vec3>,
}

fn arr(v: Vec3) -> Json {
    Json::Arr(vec![Json::Num(v.x), Json::Num(v.y), Json::Num(v.z)])
}

fn unit(v: Vec3) -> Vec3 {
    // the kernel's normalize: zero (or non-finite) length -> zero vector
    let l = v.len();
    if l <= 1e-300 || !l.is_finite() {
        Vec3::new(0.0, 0.0, 0.0)
    } else {
        let k = 1.0 / l;
        Vec3::new(v.x * k, v.y * k, v.z * k)
    }
}

/// Resolve `{kind,..}` to a face / live edge (`None` -> the reply is null).
pub fn resolve_ref(body: &Body, r: &Json) -> Option<Ref> {
    match r.get_str("kind")? {
        "face" => body.find_face(r.get_str("id")?).map(Ref::Face),
        "edge" => {
            let mut want: Vec<String> =
                r.get_arr("faces")?.iter().filter_map(|j| j.as_str().map(String::from)).collect();
            want.sort();
            if want.len() != 2 {
                return None;
            }
            let cand: Vec<EdgeId> = body
                .wire_chains()
                .into_iter()
                .map(|c| c[0])
                .filter(|&e| !body.is_split_rail(e))
                .filter(|&e| {
                    let mut fs: Vec<String> = body.edge_faces(e).iter().map(|&f| body.face_id_str(f)).collect();
                    fs.sort();
                    fs == want
                })
                .collect();
            let at = r.get("at").and_then(Json::as_vec3).map(Vec3::from_array);
            let pick = match at {
                Some(p) => cand.iter().copied().min_by(|&x, &y| {
                    let dx = body.edge(x).curve.closest_point(p).1.dist(p);
                    let dy = body.edge(y).curve.closest_point(p).1.dist(p);
                    dx.total_cmp(&dy)
                }),
                None => cand.first().copied(),
            };
            pick.map(Ref::Edge)
        }
        _ => None,
    }
}

fn face_item(body: &Body, f: FaceId) -> (Json, Item) {
    let face = body.face(f);
    let sg = if face.sense { 1.0 } else { -1.0 };
    let area = sg * ck_topo::mass::face_integral::<1>(body, f, 2, &|_, n| [n.len()])[0];
    let mut o = Obj::new().put("kind", "face").put("surface", face.surface.kind()).put("area", area);
    let mut it = Item::default();
    match &face.surface {
        Surface::Plane { frame } => {
            // x cross y, normalised; flipped with the face sense
            let n = unit(frame.x.cross(frame.y));
            let n = if face.sense { n } else { -n };
            it.normal = Some(n);
            it.plane_o = Some(frame.o);
            o = o.put("normal", arr(n));
        }
        Surface::Cylinder { frame, r } => {
            o = o.put("radius", *r).put("center", arr(frame.o)).put("axis", arr(frame.z));
            it.center = Some(frame.o);
            it.axis = Some(frame.z);
        }
        Surface::Cone { frame, .. } => {
            o = o.put("center", arr(frame.o)).put("axis", arr(frame.z));
            it.center = Some(frame.o);
            it.axis = Some(frame.z);
        }
        Surface::Sphere { frame, r } => {
            o = o.put("radius", *r).put("center", arr(frame.o));
            it.center = Some(frame.o);
        }
        _ => {}
    }
    (o.build(), it)
}

fn edge_item(body: &Body, e: EdgeId) -> (Json, Item) {
    let ed = body.edge(e);
    let len = ed.curve.length_between(ed.t0, ed.t1);
    let mut o = Obj::new().put("kind", "edge").put("length", len);
    let mut it = Item { is_edge: true, ..Default::default() };
    match &ed.curve {
        Curve::Line { d, .. } => {
            let d = unit(*d);
            o = o.put("direction", arr(d));
            it.direction = Some(d);
        }
        Curve::Circle { frame, r } => {
            o = o.put("radius", *r).put("center", arr(frame.o)).put("axis", arr(frame.z));
            it.center = Some(frame.o);
            it.axis = Some(frame.z);
        }
        _ => {}
    }
    let cs = body.edge_coedges(e);
    if cs.len() == 2 {
        let t = (ed.t0 + ed.t1) * 0.5;
        let n = |c| {
            let uv = body.coedge_uv(c, t, None);
            body.outward_normal(body.coedge_face(c), uv.x, uv.y)
        };
        let (a, b) = (n(cs[0]), n(cs[1]));
        o = o.put("wedge", a.cross(b).len().atan2(-a.dot(b)));
    }
    (o.build(), it)
}

// ---------------------------------------------------------- projection

/// Closest point of a face (surface projection when inside the trimmed
/// face, else the closest boundary point).
fn closest_on_face(body: &Body, f: FaceId, loops: &[Vec<Vec2>], q: Vec3) -> Vec3 {
    let face = body.face(f);
    let (u, v, p) = face.surface.closest_point(q);
    let unbounded = loops.iter().all(|l| l.len() < 3);
    if unbounded || in_loops(&face.surface, Vec2::new(u, v), loops) {
        return p;
    }
    let mut best: Option<(f64, Vec3)> = None;
    for e in body.face_edges(f) {
        let ed = body.edge(e);
        let (t, d) = dist_in_range(&ed.curve, (ed.t0, ed.t1), q);
        if best.map_or(true, |(bd, _)| d < bd) {
            best = Some((d, ed.curve.eval(t)));
        }
    }
    best.map_or(p, |b| b.1)
}

enum Target<'a> {
    Face(FaceId, Vec<Vec<Vec2>>),
    Edge(&'a Curve, f64, f64),
}

impl Target<'_> {
    fn closest(&self, body: &Body, q: Vec3) -> Vec3 {
        match self {
            Target::Face(f, loops) => closest_on_face(body, *f, loops, q),
            Target::Edge(c, t0, t1) => {
                let (t, _) = dist_in_range(c, (*t0, *t1), q);
                c.eval(t)
            }
        }
    }

    fn samples(&self, body: &Body) -> Vec<Vec3> {
        match self {
            Target::Edge(c, t0, t1) => (0..=64).map(|i| c.eval(t0 + ((t1 - t0) * i as f64) / 64.0)).collect(),
            Target::Face(f, loops) => {
                let face = body.face(*f);
                let mut pts = vec![];
                let (mut lo, mut hi) = (Vec2::new(f64::INFINITY, f64::INFINITY), Vec2::new(f64::NEG_INFINITY, f64::NEG_INFINITY));
                // boundary: the face's edges in arena order (the oracle has
                // no seam edges), 33 curve samples each, t0..t1 inclusive
                let mut es: Vec<EdgeId> = body.face_edges(*f).into_iter().filter(|&e| !body.is_seam(e)).collect();
                es.sort_by_key(|e| e.idx());
                for e in es {
                    let ed = body.edge(e);
                    let c = body.edge_coedges(e).into_iter().find(|&c| body.coedge_face(c) == *f);
                    let mut near = None;
                    for i in 0..=32 {
                        let t = ed.t0 + ((ed.t1 - ed.t0) * i as f64) / 32.0;
                        pts.push(ed.curve.eval(t));
                        if let Some(c) = c {
                            let uv = body.coedge_uv(c, t, near);
                            near = Some(uv);
                            lo = Vec2::new(lo.x.min(uv.x), lo.y.min(uv.y));
                            hi = Vec2::new(hi.x.max(uv.x), hi.y.max(uv.y));
                        }
                    }
                }
                if !(lo.x.is_finite() && hi.x.is_finite()) {
                    // no boundary (full sphere / torus): the surface domain
                    let ((u0, u1), (v0, v1)) = face.surface.domain();
                    (lo, hi) = (Vec2::new(u0, v0), Vec2::new(u1, v1));
                }
                if lo.x.is_finite() && hi.x.is_finite() && lo.y.is_finite() && hi.y.is_finite() {
                    for i in 0..=16 {
                        let u = lo.x + (hi.x - lo.x) * i as f64 * 0.0625;
                        for j in 0..=16 {
                            let v = lo.y + (hi.y - lo.y) * j as f64 * 0.0625;
                            if in_loops(&face.surface, Vec2::new(u, v), loops) {
                                pts.push(face.surface.eval(u, v));
                            }
                        }
                    }
                }
                pts
            }
        }
    }
}

fn target<'a>(body: &'a Body, r: Ref) -> Target<'a> {
    match r {
        Ref::Face(f) => Target::Face(f, face_loops_uv(body, f)),
        Ref::Edge(e) => {
            let ed = body.edge(e);
            Target::Edge(&ed.curve, ed.t0, ed.t1)
        }
    }
}

/// (distance, point on a, point on b).
fn min_distance(body: &Body, a: Ref, b: Ref) -> Option<(f64, Vec3, Vec3)> {
    let (ta, tb) = (target(body, a), target(body, b));
    // entries: (d, point on a, point on b)
    let mut es: Vec<(f64, Vec3, Vec3)> = vec![];
    for s in ta.samples(body) {
        let c = tb.closest(body, s);
        es.push((s.dist(c), s, c));
    }
    for s in tb.samples(body) {
        let c = ta.closest(body, s);
        es.push((s.dist(c), c, s));
    }
    // distances equal to 1e-9 are ties (the earliest sample wins, as in the
    // oracle where on-surface samples project with exactly zero distance)
    es.sort_by(|x, y| (x.0 * 1e9).round().total_cmp(&(y.0 * 1e9).round()));
    let first = *es.first()?;
    let mut best = first;
    for k in 0..es.len().min(3) {
        let (d0, mut pa, mut pb) = es[k];
        let mut cur = d0;
        let mut it = -99i32;
        loop {
            let ca = ta.closest(body, pb);
            let cb = tb.closest(body, ca);
            let d = ca.dist(cb);
            let prev = cur;
            if d <= cur {
                pb = cb;
                pa = ca;
                cur = d;
            }
            let go = it != 0;
            it += 1;
            if !((prev + 1.0) * 1e-13 <= prev - d && go) {
                break;
            }
        }
        if cur < best.0 {
            best = (cur, pa, pb);
        }
    }
    Some(best)
}

fn acos_clamped(x: f64) -> f64 {
    x.abs().min(1.0).acos()
}

/// `normal` else `direction` of an item (the angle's operand).
fn dir_of(i: &Item) -> Option<(Vec3, bool)> {
    i.normal.map(|n| (n, true)).or(i.direction.map(|d| (d, false)))
}

fn pair(body: &Body, refs: &[Ref], items: &[Item], out: &mut Obj) -> bool {
    let Some((d, pa, pb)) = min_distance(body, refs[0], refs[1]) else { return false };
    let mut o = std::mem::replace(out, Obj::new());
    o = o.put("distance", d).put("points", Json::Arr(vec![arr(pa), arr(pb)]));
    let (a, b) = (&items[0], &items[1]);
    // angle between normals / directions; plane vs line gives the
    // complement
    let mut angle = None;
    if let (Some((va, na)), Some((vb, nb))) = (dir_of(a), dir_of(b)) {
        let (ua, ub) = (unit(va), unit(vb));
        let ang = acos_clamped(ua.dot(ub));
        let ang = if na != nb { std::f64::consts::FRAC_PI_2 - ang } else { ang };
        o = o.put("angle", ang);
        angle = Some(ang);
    }
    // parallel planes: distance along the normal
    if let (Some(ang), Some(na), Some(nb)) = (angle, a.normal, b.normal) {
        if ang < 1e-9 && !a.is_edge && !b.is_edge {
            if let (Some(oa), Some(ob)) = (a.plane_o, b.plane_o) {
                let _ = nb;
                o = o.put("normalDistance", (na.z * (ob.z - oa.z) + na.x * (ob.x - oa.x) + na.y * (ob.y - oa.y)).abs());
            }
        }
    }
    // centre / axis distance
    if let (Some(ca), Some(cb)) = (a.center, b.center) {
        match (a.axis, b.axis) {
            (Some(xa), Some(xb)) if ((xa.dot(xb)).abs() - 1.0).abs() < 1e-9 => {
                let dd = cb - ca;
                let n = unit(xa);
                let k = dd.dot(n);
                let r = dd - n * k;
                o = o.put("centerDistance", r.len());
            }
            (axa, axb) => {
                let both = axa.is_some() && axb.is_some();
                if a.is_edge || b.is_edge || !both {
                    o = o.put("centerDistance", (cb - ca).len());
                }
            }
        }
    }
    *out = o;
    true
}

/// The `measure` reply (`null` when any reference fails to resolve).
pub fn reply(body: &Body, msg: &Json) -> Json {
    let Some(rs) = msg.get_arr("refs") else { return Json::Null };
    let mut refs = vec![];
    for r in rs {
        match resolve_ref(body, r) {
            Some(x) => refs.push(x),
            None => return Json::Null,
        }
    }
    if refs.is_empty() {
        return Json::Null;
    }
    let mut js = vec![];
    let mut items = vec![];
    for &r in &refs {
        let (j, i) = match r {
            Ref::Face(f) => face_item(body, f),
            Ref::Edge(e) => edge_item(body, e),
        };
        js.push(j);
        items.push(i);
    }
    let mut o = Obj::new().put("items", Json::Arr(js));
    if refs.len() == 2 {
        pair(body, &refs, &items, &mut o);
    }
    o.build()
}
