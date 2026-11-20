//! Fillet joints: two rounded edges that meet at a vertex whose third edge is left sharp. A result is judged by the
//! PROPERTIES a correct round has, not only by validity (a fillet face that overran its edge as a fin over the top
//! face passed `validate()` and had a plausible volume):
//! - (a) extent: every point of a blend face lies within reach of its own edge (no more than the mitre's run-on past
//!   either end, and no farther from the edge than the section reaches);
//! - (b) no crossing: no triangle of a face the fillet made crosses a triangle of a face it does not border;
//! - (c) containment: a fillet of convex edges only removes material: every face of the result lies in the original
//!   solid, and the volume removed is at least half and at most the analytic section area times the edge length plus
//!   a corner allowance per vertex;
//! - (d) the number of faces grows by at most the blend faces plus two per joint.
//!
//! The first case is a user's part (`fixtures/fillet_fin_repro.ckpart.json`: a D18 x 10 disc with a nearly round hole,
//! a 302 degree arc closed by a chord, its two top edges rounded R0.5). The others vary the joint: a pocket or a boss
//! whose outline has a corner of a given angle, arc + chord holes at several chord angles, equal and unequal radii.
//! A result may also be a refusal, when it carries a specific code with a detail and a hint.
use ck_geom::Vec3;
use ck_model::features::Brep;
use ck_model::regen::{self, FeatureExec, RegenOpts};
use ck_model::Part;
use ck_topo::body::{Body, EdgeId, FaceId};

struct Exec(Brep);
impl FeatureExec for Exec {
    fn face_count(&self) -> Option<usize> {
        self.0.face_count()
    }
    fn face_frame(&mut self, s: &regen::RegenState, face: &str) -> Option<ck_core::linalg::Frame> {
        self.0.face_frame(s, face)
    }
    fn exec(&mut self, s: &mut regen::RegenState, f: &ck_model::Feature) -> Result<(), regen::FeatureError> {
        self.0.exec(s, f)
    }
}

const USER: &str = include_str!("fixtures/fillet_fin_repro.ckpart.json");

/// regen of `src`; `drop_last`: without the last feature (the fillet). Errors as `code {params}`.
fn regen_src(src: &str, drop_last: bool) -> (Option<Body>, Vec<(String, String)>) {
    let mut p = Part::parse(src).unwrap();
    if drop_last {
        p.features.pop();
    }
    let mut ex = Exec(Brep::new());
    let r = regen::regen(&p, &RegenOpts::default(), &mut ex);
    let errs: Vec<(String, String)> = r.errors.iter().map(|e| (e.error.code.clone(), format!("{:?}", e.error.params))).collect();
    (ex.0.body.clone(), errs)
}

/// One fillet pick of the case: the two face names and the pick point, its radius.
#[derive(Clone)]
struct Pick {
    fa: String,
    fb: String,
    at: Vec3,
    r: f64,
}

/// The fillet feature's picks, read back from the part text (`faces`, `at`, optional per-pick `radius`).
fn picks_of(src: &str, default_r: f64) -> Vec<Pick> {
    let j = ck_json::parse(src).unwrap();
    let feats = j.get("features").unwrap().as_arr().unwrap();
    let fil = feats.last().unwrap();
    let r = fil.get_f64("radius").unwrap_or(default_r);
    fil.get("edges")
        .unwrap()
        .as_arr()
        .unwrap()
        .iter()
        .map(|e| {
            let fs = e.get("faces").unwrap().as_arr().unwrap();
            let at = e.get("at").unwrap().as_arr().unwrap();
            Pick {
                fa: fs[0].as_str().unwrap().into(),
                fb: fs[1].as_str().unwrap().into(),
                at: Vec3::new(at[0].as_f64().unwrap(), at[1].as_f64().unwrap(), at[2].as_f64().unwrap()),
                r: e.get_f64("radius").unwrap_or(r),
            }
        })
        .collect()
}

/// Triangles of a tessellation: corners and the face each belongs to (index into `face_ids()` order).
struct Tris {
    t: Vec<[Vec3; 3]>,
    face: Vec<usize>,
}

fn tris(b: &Body) -> Tris {
    let m = ck_tess::mesh::tessellate(b).mesh;
    let n = m.tri_face.len();
    let p = |t: usize, k: usize| Vec3::new(m.positions[9 * t + 3 * k] as f64, m.positions[9 * t + 3 * k + 1] as f64, m.positions[9 * t + 3 * k + 2] as f64);
    Tris { t: (0..n).map(|t| [p(t, 0), p(t, 1), p(t, 2)]).collect(), face: (0..n).map(|t| m.tri_face[t] as usize).collect() }
}

/// Segment `p q` crosses the interior of triangle `t` (strictly: away from its edges by `eps` in barycentrics, and away
/// from the segment's ends).
fn seg_hits_tri(p: Vec3, q: Vec3, t: &[Vec3; 3], eps: f64) -> bool {
    let (e1, e2) = (t[1] - t[0], t[2] - t[0]);
    let d = q - p;
    let h = d.cross(e2);
    let a = e1.dot(h);
    if a.abs() < 1e-14 * (e1.len() * e2.len() * d.len()).max(1e-300) {
        return false;
    }
    let f = 1.0 / a;
    let s = p - t[0];
    let u = f * s.dot(h);
    if u < eps || u > 1.0 - eps {
        return false;
    }
    let qv = s.cross(e1);
    let v = f * d.dot(qv);
    if v < eps || u + v > 1.0 - eps {
        return false;
    }
    let w = f * e2.dot(qv);
    w > eps && w < 1.0 - eps
}

fn tri_box(t: &[Vec3; 3]) -> (Vec3, Vec3) {
    (Vec3::new(t[0].x.min(t[1].x).min(t[2].x), t[0].y.min(t[1].y).min(t[2].y), t[0].z.min(t[1].z).min(t[2].z)), Vec3::new(t[0].x.max(t[1].x).max(t[2].x), t[0].y.max(t[1].y).max(t[2].y), t[0].z.max(t[1].z).max(t[2].z)))
}

/// (b) Pairs of faces (by id) where a triangle of a face of `feature` crosses a triangle of a face it shares no edge with.
fn crossings(b: &Body, feature: &str) -> Vec<String> {
    let ids: Vec<FaceId> = b.face_ids().collect();
    let tr = tris(b);
    let mine: Vec<bool> = ids.iter().map(|&f| b.face(f).name.feature == feature).collect();
    let nb: Vec<Vec<FaceId>> = ids.iter().map(|&f| b.face_neighbors(f)).collect();
    let boxes: Vec<(Vec3, Vec3)> = tr.t.iter().map(tri_box).collect();
    let mut out: Vec<String> = vec![];
    for i in 0..tr.t.len() {
        let fi = tr.face[i];
        if !mine[fi] {
            continue;
        }
        for j in 0..tr.t.len() {
            let fj = tr.face[j];
            if fj == fi || (mine[fj] && j < i) || nb[fi].contains(&ids[fj]) {
                continue;
            }
            let (a, c) = (boxes[i], boxes[j]);
            if a.1.x < c.0.x || c.1.x < a.0.x || a.1.y < c.0.y || c.1.y < a.0.y || a.1.z < c.0.z || c.1.z < a.0.z {
                continue;
            }
            let (s, t) = (&tr.t[i], &tr.t[j]);
            let hit = (0..3).any(|k| seg_hits_tri(s[k], s[(k + 1) % 3], t, 1e-6)) || (0..3).any(|k| seg_hits_tri(t[k], t[(k + 1) % 3], s, 1e-6));
            if hit {
                let m = format!("{} x {}", b.face_id_str(ids[fi]), b.face_id_str(ids[fj]));
                if !out.contains(&m) {
                    out.push(m);
                }
            }
        }
    }
    out
}

/// A polyline of edge `e` (200 segments).
fn edge_poly(b: &Body, e: EdgeId) -> Vec<Vec3> {
    let ed = b.edge(e);
    (0..=200).map(|i| ed.curve.eval(ed.t0 + (ed.t1 - ed.t0) * i as f64 / 200.0)).collect()
}

/// (distance from `q` to the polyline, run-on of `q` past the nearer end along the end tangent, 0 when beside the edge)
fn reach(poly: &[Vec3], q: Vec3) -> (f64, f64) {
    let mut best = (f64::INFINITY, 0usize, 0.0f64);
    for i in 0..poly.len() - 1 {
        let (a, c) = (poly[i], poly[i + 1]);
        let d = c - a;
        let t = ((q - a).dot(d) / d.dot(d)).clamp(0.0, 1.0);
        let dist = (a + d * t - q).len();
        if dist < best.0 {
            best = (dist, i, t);
        }
    }
    let n = poly.len() - 1;
    let over = if best.1 == 0 && best.2 <= 0.0 {
        (q - poly[0]).dot((poly[0] - poly[1]).normalize()).max(0.0)
    } else if best.1 == n - 1 && best.2 >= 1.0 {
        (q - poly[n]).dot((poly[n] - poly[n - 1]).normalize()).max(0.0)
    } else {
        0.0
    };
    (best.0, over)
}

/// Unit direction of edge `e` leaving vertex end `at_start`.
fn away(b: &Body, e: EdgeId, at_start: bool) -> Vec3 {
    let ed = b.edge(e);
    let fwd = if ed.t1 >= ed.t0 { 1.0 } else { -1.0 };
    if at_start { ed.curve.tangent(ed.t0).normalize() * fwd } else { ed.curve.tangent(ed.t1).normalize() * -fwd }
}

/// Smallest angle (radians) between two picked edges at a vertex they share; PI when no two share one.
fn joint_angle(b: &Body, es: &[EdgeId]) -> (f64, usize) {
    let mut ang = std::f64::consts::PI;
    let mut joints = 0;
    for i in 0..es.len() {
        for j in i + 1..es.len() {
            let (a, c) = (b.edge(es[i]), b.edge(es[j]));
            for (va, sa) in [(a.v0, true), (a.v1, false)] {
                for (vc, sc) in [(c.v0, true), (c.v1, false)] {
                    if va == vc {
                        joints += 1;
                        ang = ang.min(away(b, es[i], sa).dot(away(b, es[j], sc)).clamp(-1.0, 1.0).acos());
                    }
                }
            }
        }
    }
    (ang, joints)
}

/// Every property of a correct result, as a list of failures (empty = pass). `orig`: the body before the fillet.
fn judge(orig: &Body, out: &Body, picks: &[Pick], feature: &str) -> Vec<String> {
    let mut bad: Vec<String> = vec![];
    for i in ck_check::validate(out, 1e-6) {
        bad.push(format!("validate {} {} {}", i.code, i.entity, i.detail));
    }
    let es: Vec<EdgeId> = picks.iter().map(|p| ck_ops::blend::build::find_edge(orig, &p.fa, &p.fb, p.at).expect("pick")).collect();
    let rmax = picks.iter().map(|p| p.r).fold(0.0, f64::max);
    let (theta, joints) = joint_angle(orig, &es);
    // (a) the mitre of two rounds run on past a corner of angle theta reaches R / tan(theta / 2) past the vertex
    let run_on = rmax * (1.0 + 1.0 / (0.5 * theta).tan().max(0.05)) + 1e-6;
    let tr = tris(out);
    let ids: Vec<FaceId> = out.face_ids().collect();
    // every edge between the same two faces as a pick is blended with it (the arcs of a ring a seam splits): a blend face
    // may run along any of them
    let mut all: Vec<EdgeId> = es.clone();
    for &e in &es {
        let fs = orig.edge_faces(e);
        for x in orig.edge_ids() {
            let xf = orig.edge_faces(x);
            if !all.contains(&x) && xf.len() == 2 && fs.len() == 2 && xf.iter().all(|f| fs.contains(f)) {
                all.push(x);
            }
        }
    }
    let polys: Vec<Vec<Vec3>> = all.iter().map(|&e| edge_poly(orig, e)).collect();
    let r_of = |i: usize| picks.get(i).map_or(rmax, |p| p.r);
    for k in 0..picks.len() {
        let name = format!("{feature}.B{k}");
        let mut worst = (0.0f64, 0.0f64);
        let mut any = false;
        for t in 0..tr.t.len() {
            let id = out.face_id_str(ids[tr.face[t]]);
            if id != name && !id.starts_with(&format!("{name}~")) {
                continue;
            }
            any = true;
            for &q in &tr.t[t] {
                // the nearest blended edge decides
                let (d, o) = polys.iter().map(|pl| reach(pl, q)).min_by(|a, b| a.0.total_cmp(&b.0)).unwrap();
                worst = (worst.0.max(d), worst.1.max(o));
            }
        }
        if worst.1 > run_on {
            bad.push(format!("extent: {name} runs {:.4} past an end of its edge (at most {run_on:.4})", worst.1));
        }
        if worst.0 > run_on * 1.5 + 1.5 * r_of(k) {
            bad.push(format!("extent: {name} reaches {:.4} from its edge (at most {:.4})", worst.0, run_on * 1.5 + 1.5 * r_of(k)));
        }
        if !any {
            bad.push(format!("extent: no face {name}"));
        }
    }
    // (b)
    for c in crossings(out, feature) {
        bad.push(format!("crossing: {c}"));
    }
    // (c) every pick convex here: each face lies in the original solid
    let solid = ck_ops::boolean::classify::Solid::new(orig, 1e-6);
    let cont = ck_ops::contain::Containment::new(orig, 1e-6);
    let mut outside = 0;
    let mut where_out: Option<(String, Vec3)> = None;
    for t in 0..tr.t.len() {
        let f = ids[tr.face[t]];
        let c = (tr.t[t][0] + tr.t[t][1] + tr.t[t][2]) * (1.0 / 3.0);
        let c = out.face(f).surface.closest_point(c).2;
        // (two independent classifiers must agree: each has been seen to call a point near a tangent face Out alone)
        // (and off the boundary by more than the tessellation's reach: a centroid next to a contact line can land a
        // thousandth of a millimetre past the wall it is tangent to)
        let off = |q: Vec3| cont.nearest(q).map_or(f64::INFINITY, |x| x.0);
        if solid.classify(c, Vec3::Z) == ck_ops::boolean::classify::Class::Out && cont.classify_point(c, 1e-6) == ck_ops::contain::Loc::Out && off(c) > 1e-3 + 0.01 * rmax {
            outside += 1;
            where_out.get_or_insert((out.face_id_str(f), c));
        }
    }
    if outside > 0 {
        let (f, c) = where_out.unwrap();
        bad.push(format!("containment: {outside} triangles outside the original, first on {f} at ({:.4},{:.4},{:.4})", c.x, c.y, c.z));
    }
    let (v0, v1) = (ck_topo::mass::mass_props(orig).volume, ck_topo::mass::mass_props(out).volume);
    let mut area_len = 0.0;
    for (p, &e) in picks.iter().zip(&es) {
        let ed = orig.edge(e);
        let bend = ed.curve.circle_info().map_or(f64::INFINITY, |(_, _, rr)| rr);
        area_len += (1.0 - std::f64::consts::FRAC_PI_4) * p.r * p.r * orig.edge_length(e) * (1.0 + p.r / bend);
    }
    let removed = v0 - v1;
    let ends = 2 * es.len();
    let hi = area_len + 2.0 * rmax.powi(3) * ends as f64 * (1.0 + run_on / rmax) + 1e-9 * v0;
    if removed < -1e-9 * v0 || removed > hi || removed < 0.5 * area_len - rmax.powi(3) * ends as f64 {
        bad.push(format!("volume: removed {removed:.6}, expected about {area_len:.6} (at most {hi:.6})"));
    }
    // (d) a blend face per blended edge (a pick and the arcs of its ring), an end face per open end of the blended
    // chain, two more faces per joint (the flush ends of two rounds that stop at a cusp)
    let (n0, n1) = (orig.num_faces(), out.num_faces());
    let mut ends: Vec<ck_topo::body::VertexId> = vec![];
    for &e in &all {
        for v in [orig.edge(e).v0, orig.edge(e).v1] {
            let shared = all.iter().filter(|&&x| orig.edge(x).v0 == v || orig.edge(x).v1 == v).count() > 1;
            if !shared && !ends.contains(&v) {
                ends.push(v);
            }
        }
    }
    let most = n0 + all.len() + ends.len() + 2 * joints;
    if n1 > most {
        bad.push(format!("faces: {n0} -> {n1} (at most {most})"));
    }
    bad
}

/// Face list of a result (kind, triangle area, box) and a second opinion on containment (`JOINT_DUMP=1`).
fn dump(orig: &Body, out: &Body) {
    let tr = tris(out);
    let ids: Vec<FaceId> = out.face_ids().collect();
    let solid = ck_ops::boolean::classify::Solid::new(orig, 1e-6);
    let cont = ck_ops::contain::Containment::new(orig, 1e-6);
    for (i, &f) in ids.iter().enumerate() {
        let mut area = 0.0;
        let (mut lo, mut hi) = (Vec3::new(1e9, 1e9, 1e9), Vec3::new(-1e9, -1e9, -1e9));
        let mut outside = 0;
        for t in (0..tr.t.len()).filter(|&t| tr.face[t] == i) {
            let [a, b, c] = tr.t[t];
            area += 0.5 * (b - a).cross(c - a).len();
            for q in [a, b, c] {
                lo = Vec3::new(lo.x.min(q.x), lo.y.min(q.y), lo.z.min(q.z));
                hi = Vec3::new(hi.x.max(q.x), hi.y.max(q.y), hi.z.max(q.z));
            }
            let m = out.face(f).surface.closest_point((a + b + c) * (1.0 / 3.0)).2;
            if cont.classify_point(m, 1e-5) == ck_ops::contain::Loc::Out {
                outside += 1;
                if outside == 1 {
                    eprintln!("    out point ({:.4},{:.4},{:.4}) boolean classify {:?}", m.x, m.y, m.z, solid.classify(m, Vec3::Z));
                }
            }
        }
        eprintln!("  {} {} area {area:.5} box ({:.3},{:.3},{:.3})..({:.3},{:.3},{:.3}) outside {outside}", out.face_id_str(f), out.face(f).surface.kind(), lo.x, lo.y, lo.z, hi.x, hi.y, hi.z);
    }
}

/// Codes a correct refusal of a joint may carry (with a detail and a hint).
const REFUSALS: &[&str] = &["blendCorner", "blendUnsupported", "blendOverflow", "blendResultInvalid", "blendTooLarge", "blendTooLargeMax", "filletShape"];

/// Built and judged: `Ok(())` for a correct result, `Err(refusal)` for an acceptable refusal; panics on a wrong result.
fn check_case(tag: &str, src: &str, default_r: f64) -> Result<(), String> {
    let (orig, e0) = regen_src(src, true);
    assert!(e0.is_empty(), "{tag}: base errors {e0:?}");
    let orig = orig.unwrap();
    let (out, errs) = regen_src(src, false);
    let feature = "FIL";
    if let Some((code, params)) = errs.first() {
        assert!(REFUSALS.contains(&code.as_str()) && params.contains("detail") && params.contains("hint"), "{tag}: refusal without a usable reason: {code} {params}");
        eprintln!("{tag}: refused {code} {params}");
        return Err(code.clone());
    }
    let out = out.unwrap();
    if std::env::var("JOINT_DUMP").is_ok() {
        dump(&orig, &out);
    }
    let bad = judge(&orig, &out, &picks_of(src, default_r), feature);
    eprintln!("{tag}: faces {} -> {}, {}", orig.num_faces(), out.num_faces(), if bad.is_empty() { "ok".into() } else { bad.join("; ") });
    assert!(bad.is_empty(), "{tag}: wrong result: {bad:#?}");
    Ok(())
}

/// The user's part with the fillet renamed `FIL`, radius `r`, picks `which` (0: the arc edge, 1: the chord edge).
fn user_case(r: f64, which: &[usize]) -> String {
    let j = ck_json::parse(USER).unwrap();
    let mut feats = j.get("features").unwrap().as_arr().unwrap().to_vec();
    let fil = feats.pop().unwrap();
    let edges = fil.get("edges").unwrap().as_arr().unwrap();
    let sel: Vec<String> = which.iter().map(|&i| ck_json::to_string(&edges[i])).collect();
    let mut s: Vec<String> = feats.iter().map(ck_json::to_string).collect();
    s.push(format!(r#"{{"type":"fillet","id":"FIL","radius":{r},"edges":[{}]}}"#, sel.join(",")));
    format!(r#"{{"format":"ck-part","version":1,"features":[{}]}}"#, s.join(","))
}

#[test]
fn user_part_fin() {
    check_case("user r0.5 both", &user_case(0.5, &[0, 1]), 0.5).ok();
}

#[test]
fn user_part_variants() {
    for (r, which) in [(0.3, vec![0, 1]), (0.1, vec![0, 1]), (0.5, vec![0]), (0.5, vec![1])] {
        check_case(&format!("user r{r} {which:?}"), &user_case(r, &which), r).ok();
    }
}

// ---- generated joints ----

/// A sketch on the XY plane (`front`) of a closed outline of lines and arcs: `segs` as (`line`, p) or (`arc`, centre, p)
/// meaning a segment from the previous point to `p` (an arc about `centre`, counter-clockwise from the previous point).
fn sketch(id: &str, pts: &[(f64, f64)], segs: &[(char, usize, usize, Option<usize>)]) -> String {
    let ps: Vec<String> = pts.iter().enumerate().map(|(i, (x, y))| format!(r#"{{"id":"p{i}","x":{x},"y":{y}}}"#)).collect();
    let es: Vec<String> = segs
        .iter()
        .enumerate()
        .map(|(i, &(k, a, b, c))| match k {
            'l' => format!(r#"{{"id":"e{}","type":"line","p1":"p{a}","p2":"p{b}"}}"#, i + 1),
            _ => format!(r#"{{"id":"e{}","type":"arc","c":"p{}","p1":"p{a}","p2":"p{b}"}}"#, i + 1, c.unwrap()),
        })
        .collect();
    format!(r#"{{"type":"sketch","id":"{id}","plane":{{"kind":"std","id":"front"}},"points":[{}],"entities":[{}],"constraints":[]}}"#, ps.join(","), es.join(","))
}

fn rect(id: &str, w: f64, h: f64) -> String {
    let pts = [(-w, -h), (w, -h), (w, h), (-w, h)];
    sketch(id, &pts, &[('l', 0, 1, None), ('l', 1, 2, None), ('l', 2, 3, None), ('l', 3, 0, None)])
}

fn pick(fa: &str, fb: &str, at: (f64, f64, f64), r: Option<f64>) -> String {
    let rr = r.map_or(String::new(), |r| format!(r#","radius":{r}"#));
    format!(r#"{{"faces":["{fa}","{fb}"],"at":[{},{},{}]{rr}}}"#, at.0, at.1, at.2)
}

fn doc(feats: &[String]) -> String {
    format!(r#"{{"format":"ck-part","version":1,"features":[{}]}}"#, feats.join(","))
}

/// A triangle with a corner of `deg` degrees at the origin, its two sides along +x and at `deg` from it, `len` long; cut
/// as a pocket through a 40 x 40 x 10 block (`pocket`) or standing as a 6 high boss on it. The two top edges at that
/// corner are rounded with radii `r1`, `r2`.
fn corner_case(deg: f64, pocket: bool, r1: f64, r2: f64) -> String {
    let a = deg.to_radians();
    let len = 12.0;
    let pts = [(0.0, 0.0), (len, 0.0), (len * a.cos(), len * a.sin())];
    let tri = sketch("S2", &pts, &[('l', 0, 1, None), ('l', 1, 2, None), ('l', 2, 0, None)]);
    let base = rect("S1", 20.0, 20.0);
    let top = if pocket { 10.0 } else { 16.0 };
    let (mid1, mid2) = ((len * 0.5, 0.0, top), (len * 0.5 * a.cos(), len * 0.5 * a.sin(), top));
    let (fa, f1, f2) = if pocket { ("B1.C1", "C1.S.e1", "C1.S.e3") } else { ("C1.C1", "C1.S.e1", "C1.S.e3") };
    let cut = if pocket {
        r#"{"type":"extrude","op":"cut","sketch":"S2","end":"throughBoth","id":"C1"}"#.to_string()
    } else {
        r#"{"type":"extrude","op":"add","sketch":"S2","end":"blind","depth":16,"id":"C1"}"#.to_string()
    };
    let unequal = (r1 - r2).abs() > 1e-12;
    let fil = format!(
        r#"{{"type":"fillet","id":"FIL","radius":{r1},"edges":[{},{}]}}"#,
        pick(fa, f1, mid1, unequal.then_some(r1)),
        pick(fa, f2, mid2, unequal.then_some(r2))
    );
    doc(&[base, r#"{"type":"extrude","op":"add","sketch":"S1","end":"blind","depth":10,"id":"B1"}"#.into(), tri, cut, fil])
}

/// The user's hole shape at another chord angle: a circle of radius 3 about the origin, cut through a 40 x 40 x 10
/// block, closed by a chord whose ends make `deg` degrees with the arc (the chord subtends 2 deg of the circle).
fn chord_case(deg: f64, r: f64) -> String {
    let rc = 3.0;
    let h = deg.to_radians();
    // chord ends at angles -90 -/+ deg (below the centre); the arc runs counter-clockwise the long way from the right end
    let (pa, pb) = ((rc * (-std::f64::consts::FRAC_PI_2 + h).cos(), rc * (-std::f64::consts::FRAC_PI_2 + h).sin()), (rc * (-std::f64::consts::FRAC_PI_2 - h).cos(), rc * (-std::f64::consts::FRAC_PI_2 - h).sin()));
    let pts = [pa, pb, (0.0, 0.0)];
    let hole = sketch("S2", &pts, &[('a', 0, 1, Some(2)), ('l', 1, 0, None)]);
    let base = rect("S1", 20.0, 20.0);
    let arc_mid = (0.0, rc, 10.0);
    let chord_mid = (0.5 * (pa.0 + pb.0), 0.5 * (pa.1 + pb.1), 10.0);
    let fil = format!(r#"{{"type":"fillet","id":"FIL","radius":{r},"edges":[{},{}]}}"#, pick("B1.C1", "C1.S.e1", arc_mid, None), pick("B1.C1", "C1.S.e2", chord_mid, None));
    doc(&[base, r#"{"type":"extrude","op":"add","sketch":"S1","end":"blind","depth":10,"id":"B1"}"#.into(), hole, r#"{"type":"extrude","op":"cut","sketch":"S2","end":"throughBoth","id":"C1"}"#.into(), fil])
}

#[test]
fn chord_hole_angles() {
    let mut refused = vec![];
    for deg in [15.0, 29.0, 45.0, 60.0, 90.0] {
        for r in [0.5, 0.2] {
            if let Err(c) = check_case(&format!("chord {deg} r{r}"), &chord_case(deg, r), r) {
                refused.push(format!("{deg}/{r}: {c}"));
            }
        }
    }
    eprintln!("chord refusals: {refused:?}");
}

#[test]
fn pocket_corners() {
    for deg in [15.0, 29.0, 45.0, 60.0, 90.0, 120.0] {
        check_case(&format!("pocket {deg} r1"), &corner_case(deg, true, 1.0, 1.0), 1.0).ok();
    }
}

#[test]
fn boss_corners() {
    for deg in [15.0, 29.0, 45.0, 60.0, 90.0, 120.0] {
        check_case(&format!("boss {deg} r1"), &corner_case(deg, false, 1.0, 1.0), 1.0).ok();
    }
}

#[test]
fn unequal_corners() {
    for deg in [29.0, 60.0, 90.0] {
        check_case(&format!("pocket {deg} r1/0.5"), &corner_case(deg, true, 1.0, 0.5), 1.0).ok();
        check_case(&format!("boss {deg} r1/0.5"), &corner_case(deg, false, 1.0, 0.5), 1.0).ok();
    }
}

/// Two overlapping round holes (a figure-8 cut through the block): the two rim arcs meet at two cusps where the wall edge
/// is concave; both rims rounded.
fn figure8_case(r: f64) -> String {
    let h = (9.0f64 - 4.0).sqrt();
    let pts = [(0.0, h), (0.0, -h), (-2.0, 0.0), (2.0, 0.0)];
    let hole = sketch("S2", &pts, &[('a', 0, 1, Some(2)), ('a', 1, 0, Some(3))]);
    let base = rect("S1", 20.0, 20.0);
    let fil = format!(r#"{{"type":"fillet","id":"FIL","radius":{r},"edges":[{},{}]}}"#, pick("B1.C1", "C1.S.e1", (-5.0, 0.0, 10.0), None), pick("B1.C1", "C1.S.e2", (5.0, 0.0, 10.0), None));
    doc(&[base, r#"{"type":"extrude","op":"add","sketch":"S1","end":"blind","depth":10,"id":"B1"}"#.into(), hole, r#"{"type":"extrude","op":"cut","sketch":"S2","end":"throughBoth","id":"C1"}"#.into(), fil])
}

#[test]
fn figure8_hole_cusps() {
    for r in [0.5, 0.2] {
        check_case(&format!("figure8 r{r}"), &figure8_case(r), r).ok();
    }
}

/// The guard on its own: a "blend" result that grew material outside the solid it was cut from (a boss standing where
/// a convex round should only have removed some) is refused with `blendResultInvalid` and the failing check named; the
/// unchanged body passes.
#[test]
fn guard_refuses_material_outside() {
    let base = doc(&[rect("S1", 20.0, 20.0), r#"{"type":"extrude","op":"add","sketch":"S1","end":"blind","depth":10,"id":"B1"}"#.into()]);
    let (input, e0) = regen_src(&base, false);
    assert!(e0.is_empty());
    let input = input.unwrap();
    let grown = doc(&[
        rect("S1", 20.0, 20.0),
        r#"{"type":"extrude","op":"add","sketch":"S1","end":"blind","depth":10,"id":"B1"}"#.into(),
        sketch("S2", &[(18.0, -2.0), (21.0, -2.0), (21.0, 2.0), (18.0, 2.0)], &[('l', 0, 1, None), ('l', 1, 2, None), ('l', 2, 3, None), ('l', 3, 0, None)]),
        r#"{"type":"extrude","op":"add","sketch":"S2","end":"blind","depth":12,"id":"FIL"}"#.into(),
    ]);
    let (out, e1) = regen_src(&grown, false);
    assert!(e1.is_empty(), "{e1:?}");
    let out = out.unwrap();
    let picks = vec![("B1.C1".to_string(), "B1.S.e2".to_string(), Vec3::new(20.0, 10.0, 10.0))];
    assert!(ck_ops::blend::validity::guard(&input, &input, &picks, 1.0, 1e-6).is_ok());
    let err = ck_ops::blend::validity::guard(&input, &out, &picks, 1.0, 1e-6).expect_err("a result outside the input must be refused");
    assert_eq!(err.code, "blendResultInvalid");
    assert!(err.params.iter().any(|(k, v)| k == "check" && v == "containment"), "{err:?}");
    assert!(err.params.iter().any(|(k, _)| k == "hint"), "{err:?}");
}

/// The guard must pass the blend profiles whose contact lies farther out than a circle's (curvature-continuous sections),
/// and the distance / angle chamfers, on a plain box edge (gate cases `filletCurvShape1`, `chamferDD`, `chamferAD`).
#[test]
fn guard_passes_profiles_and_chamfers_on_a_box_edge() {
    let base = r#"{"type":"sketch","id":"F1","plane":{"kind":"std","id":"top"},"points":[{"id":"a","x":0,"y":0},{"id":"b","x":40,"y":0},{"id":"c","x":40,"y":20},{"id":"d","x":0,"y":20}],"entities":[{"type":"line","id":"e1","p1":"a","p2":"b"},{"type":"line","id":"e2","p1":"b","p2":"c"},{"type":"line","id":"e3","p1":"c","p2":"d"},{"type":"line","id":"e4","p1":"d","p2":"a"}],"constraints":[]},{"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":10}"#;
    let e1 = r#"{"faces":["F2.C1","F2.S.e1"],"at":[20,10,0]}"#;
    let cases = [
        (r#""type":"fillet","radius":2,"profile":"curvature","shape":1"#, Some(7932.365505495142)),
        (r#""type":"fillet","radius":2,"profile":"curvature","shape":0.5"#, None),
        (r#""type":"fillet","radius":2,"profile":"conic","rho":0.5"#, None),
        (r#""type":"chamfer","chamferType":"distanceDistance","distance":2,"distance2":3"#, Some(7880.0)),
        (r#""type":"chamfer","chamferType":"angleDistance","distance":2,"angle":30"#, None),
    ];
    for (f, want) in cases {
        let src = format!(r#"{{"format":"ck-part","version":1,"features":[{base},{{"id":"F3",{f},"edges":[{e1}]}}]}}"#);
        let (b, errs) = regen_src(&src, false);
        assert!(errs.is_empty(), "{f}: {errs:?}");
        let v = ck_topo::mass::mass_props(b.as_ref().unwrap()).volume;
        assert!(v < 8000.0 - 1.0, "{f}: volume {v}");
        if let Some(w) = want {
            // (the gate compares with the oracle to 1e-4 of the volume; this builder lands 0.2 mm3 off it on the curvature section)
            assert!((v - w).abs() < 1e-4 * w, "{f}: volume {v} vs {w}");
        }
    }
}
