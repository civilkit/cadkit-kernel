//! Spoked webs read back in a geometry-equivalent order: when the history stops on a fillet, a boss whose profile
//! is a set of sectors of an annulus (arcs about one centre, lines between them) whose openings a later fillet
//! rounded is read back as the whole annulus, the round fillets of its rim as full circles, and the openings as one
//! cut at the end whose outline already carries the rounded corners.
//!
//! The file (SolidWorks' order) has the sector plates, then the fillets of the vertical edges where the plates meet
//! the hub and the rim (cylinders parallel to the extrude direction, convex corners of the openings), and rounds of
//! the plates' top edges along the hub and the rim (tori about the annulus axis). The vertical rounds and a top
//! round meet at a vertex of unequal radii, and a round larger than the hub's step rolls over the hub's edge; the
//! history read in this order needs neither: the full-circle tori are plain (a fillet of a circle edge, or the
//! revolve of the round's section where the fillet would overflow), and the cut's outline (the inner loops of the
//! boss's start face in the file, lines and arcs) leaves the vertical rounds as the walls of the openings.
//!
//! The reordered history is kept only when the whole body verifies against the file; every step it changes says so
//! in its note (geometry-equivalent, not the file's order).

use crate::{full_regen, Cx, Outcome, Status, Step, Tagged};
use ck_geom::curve::Curve;
use ck_geom::Vec3;
use ck_json::{Json, Obj};
use ck_recog::emit::{arr, n, s, vec3, Sk};
use ck_recog::model::{Kind, PlaneSpec};
use ck_sldprt::meta::SwFeature;
use ck_topo::body::{EdgeId, FaceId};
use std::collections::HashMap;

/// Sketch frame of a plane reference of our document (`std`, an offset plane of one, a frame plane).
fn plane_frame(feats: &[Json], plane: &Json) -> Option<(Vec3, Vec3, Vec3, Vec3)> {
    let std = |id: &str| -> Option<(Vec3, Vec3, Vec3, Vec3)> {
        let i = match id {
            "right" => 0,
            "top" => 1,
            "front" => 2,
            _ => return None,
        };
        let p = PlaneSpec::for_axis(i);
        Some((Vec3::ZERO, p.xs, p.ys, p.zs))
    };
    match plane.get_str("kind")? {
        "std" => std(plane.get_str("id")?),
        "feature" => {
            let id = plane.get_str("id")?;
            let f = feats.iter().find(|f| f.get_str("id") == Some(id) && f.get_str("type") == Some("plane"))?;
            match f.get_str("mode")? {
                "offset" => {
                    let (o, x, y, z) = plane_frame(feats, f.get("base")?)?;
                    Some((o + z * f.get_f64("offset")?, x, y, z))
                }
                "frame" => {
                    let v = |k: &str| f.get(k).and_then(|j| j.as_f64_vec()).filter(|a| a.len() == 3).map(|a| Vec3::new(a[0], a[1], a[2]));
                    let (o, x, y) = (v("origin")?, v("xDir")?.normalize(), v("yDir")?.normalize());
                    Some((o, x, y, x.cross(y).normalize()))
                }
                _ => None,
            }
        }
        _ => None,
    }
}

/// Centre (world) and radii of the arcs / circles of a sketch whose curves are lines and arcs about one centre.
fn concentric(sk: &Json, fr: (Vec3, Vec3, Vec3, Vec3)) -> Option<(Vec3, f64, f64)> {
    let pts: HashMap<&str, (f64, f64)> = sk.get_arr("points")?.iter().filter_map(|p| Some((p.get_str("id")?, (p.get_f64("x")?, p.get_f64("y")?)))).collect();
    let (mut c0, mut rs): (Option<(f64, f64)>, Vec<f64>) = (None, vec![]);
    for e in sk.get_arr("entities")? {
        if e.get_bool("construction") == Some(true) {
            continue;
        }
        let (c, r) = match e.get_str("type")? {
            "line" => continue,
            "arc" => {
                let (c, p) = (*pts.get(e.get_str("c")?)?, *pts.get(e.get_str("p1")?)?);
                (c, ((p.0 - c.0).powi(2) + (p.1 - c.1).powi(2)).sqrt())
            }
            "circle" => (*pts.get(e.get_str("c")?)?, e.get_f64("r")?),
            _ => return None,
        };
        match c0 {
            None => c0 = Some(c),
            Some(q) if (q.0 - c.0).abs() + (q.1 - c.1).abs() < 1e-7 => {}
            _ => return None,
        }
        rs.push(r);
    }
    let c = c0?;
    let (lo, hi) = rs.iter().fold((f64::MAX, 0.0f64), |a, &r| (a.0.min(r), a.1.max(r)));
    (hi > lo + 1e-6).then(|| (fr.0 + fr.1 * c.0 + fr.2 * c.1, lo, hi))
}

/// What a fillet step after the boss is in the reordered history.
enum Round {
    /// Cylinders parallel to the axis bordering the boss's sides: the corners of the openings, part of the cut.
    Corner,
    /// Concave tori about the axis: one full circle each, (tube centre radius and level, minor radius, the two
    /// boundary circles (radius, level): on the plane support, on the other support).
    Ring(Vec<((f64, f64), f64, (f64, f64), (f64, f64), Option<FaceId>)>),
}

/// The read-back of a spoked web; `None` when the file is not one (the caller keeps its outcome).
pub(crate) fn reformulate(t: &Tagged, cx: &mut Cx, steps: &[Step], ranges: &[(usize, usize)], groups: &HashMap<i64, Vec<FaceId>>) -> Option<Outcome> {
    let feats = cx.tree.feats.clone();
    let tol = 1e-6;
    // SolidWorks steps (not skipped by class) in order, with their faces
    let all: Vec<&SwFeature> = t.features.iter().filter(|f| crate::classify(f) != crate::Class::Skip).collect();
    for (k, st) in steps.iter().enumerate() {
        if st.status != Status::Rebuilt || !matches!(crate::classify(all.get(k)?), crate::Class::Extrude { cut: false }) {
            continue;
        }
        let (a, b) = ranges[k];
        let (Some(sk), Some(ex)) = (feats.get(a..b).and_then(|r| r.iter().find(|f| f.get_str("type") == Some("sketch"))), feats.get(a..b).and_then(|r| r.iter().rev().find(|f| f.get_str("type") == Some("extrude")))) else { continue };
        if ex.get_str("end") != Some("blind") || ex.get_str("op") != Some("add") {
            continue;
        }
        let Some(depth) = ex.get_f64("depth") else { continue };
        let Some(fr) = sk.get("plane").and_then(|p| plane_frame(&feats, p)) else { continue };
        let Some((c, r_in, r_out)) = concentric(sk, fr) else { continue };
        let ax = if ex.get_bool("flip") == Some(true) { -fr.3 } else { fr.3 };
        if let Some(o) = web(t, cx, &all, steps, ranges, groups, k, c, ax, fr.0.dot(ax) - c.dot(ax), depth, r_in, r_out, tol) {
            return Some(o);
        }
    }
    None
}

#[allow(clippy::too_many_arguments)]
fn web(t: &Tagged, cx: &mut Cx, all: &[&SwFeature], steps: &[Step], ranges: &[(usize, usize)], groups: &HashMap<i64, Vec<FaceId>>, k: usize, c: Vec3, ax: Vec3, lvl: f64, depth: f64, r_in: f64, r_out: f64, tol: f64) -> Option<Outcome> {
    let m = cx.m;
    let o = c + ax * lvl; // the axis point on the start plane
    let boss = all[k].id;
    let rad = |p: Vec3| {
        let d = p - o;
        (d - ax * d.dot(ax)).len()
    };
    let hgt = |p: Vec3| (p - o).dot(ax);
    let coaxial = |cc: Vec3, a: Vec3| a.cross(ax).len() < 1e-7 && {
        let d = cc - o;
        (d - ax * d.dot(ax)).len() < 1e-6
    };
    let tagged = |f: FaceId, id: i64| t.tags.get(f.idx()).map_or(false, |v| v.iter().any(|x| x.0 == id));
    // the steps after the boss: rebuilt non-fillet steps first (emitted as read), then fillets of the two kinds
    let mut rounds: Vec<(usize, Round)> = vec![];
    let mut between = vec![];
    for j in k + 1..all.len() {
        let f = all[j];
        let faces = groups.get(&f.id).cloned().unwrap_or_default();
        if crate::classify(f) != crate::Class::Fillet {
            if !rounds.is_empty() || steps.get(j).map_or(true, |s| s.status != Status::Rebuilt) {
                return None;
            }
            between.push(j);
            continue;
        }
        if faces.is_empty() {
            return None;
        }
        let corner = faces.iter().all(|&g| match m.kind(g) {
            Kind::Cyl { a, convex: false, .. } => a.cross(ax).len() < 1e-7 && m.edges(g).iter().any(|&e| m.across(e, g).map_or(false, |h| tagged(h, boss))),
            _ => false,
        });
        if corner {
            rounds.push((j, Round::Corner));
            continue;
        }
        let mut ring = vec![];
        for &g in &faces {
            let Kind::Torus { c: tc, a, major, minor, convex: false } = m.kind(g).clone() else { return None };
            if !coaxial(tc, a) {
                return None;
            }
            let tube = (major, hgt(tc));
            // the boundary circles about the axis: the one on the plane support (tube radius, minor away along the axis)
            let mut circ: Vec<((f64, f64), EdgeId)> = vec![];
            for e in m.edges(g) {
                if let Curve::Circle { frame, r } = m.curve(e) {
                    if coaxial(frame.o, frame.z) {
                        let q = (*r, hgt(frame.o));
                        if !circ.iter().any(|x| (x.0 .0 - q.0).abs() < tol && (x.0 .1 - q.1).abs() < tol) {
                            circ.push((q, e));
                        }
                    }
                }
            }
            if circ.len() != 2 {
                return None;
            }
            let on_plane = |q: (f64, f64)| (q.0 - tube.0).abs() < 1e-6 && ((q.1 - tube.1).abs() - minor).abs() < 1e-6;
            let (p1, p2) = if on_plane(circ[0].0) { (circ[0], circ[1]) } else if on_plane(circ[1].0) { (circ[1], circ[0]) } else { return None };
            // the other support across the second circle (a coaxial cylinder of that radius gets a fillet pick)
            let other = m.across(p2.1, g).filter(|&h| matches!(m.kind(h), Kind::Cyl { o: co, a: ca, r, .. } if coaxial(*co, *ca) && (r - p2.0 .0).abs() < 1e-6));
            ring.push((tube, minor, p1.0, p2.0, other));
        }
        rounds.push((j, Round::Ring(ring)));
    }
    if !rounds.iter().any(|r| matches!(r.1, Round::Corner)) {
        return None;
    }
    // the openings: edges of the boss's start face(s) in the file inside the annulus, closed loops of lines and arcs
    let base: Vec<FaceId> = m.faces.iter().copied().filter(|&f| tagged(f, boss) && matches!(m.kind(f), Kind::Plane { n, d } if n.cross(ax).len() < 1e-9 && (d * n.dot(ax) - o.dot(ax)).abs() < 1e-6)).collect();
    let inside = |p: Vec3| rad(p) > r_in - 1e-6 && rad(p) < r_out + 1e-6 && hgt(p).abs() < 1e-6;
    let mut loop_edges: Vec<EdgeId> = vec![];
    for &f in &base {
        for e in m.edges(f) {
            if !loop_edges.contains(&e) && inside(m.p0(e)) && inside(m.p1(e)) && inside(m.mid(e)) {
                loop_edges.push(e);
            }
        }
    }
    if loop_edges.is_empty() || !loop_edges.iter().any(|&e| m.b.edge_faces(e).iter().any(|&h| tagged(h, boss) && !base.contains(&h))) {
        return None;
    }
    // closed: every end point is shared by exactly two edges
    let ends: Vec<Vec3> = loop_edges.iter().flat_map(|&e| [m.p0(e), m.p1(e)]).collect();
    if ends.iter().any(|p| ends.iter().filter(|q| q.dist(*p) < 1e-6).count() != 2) {
        return None;
    }
    // the seam direction: the middle of the narrowest arc about the axis (inside an opening)
    let mut seam: Option<(f64, Vec3)> = None;
    for &e in &loop_edges {
        match m.curve(e) {
            Curve::Line { .. } => {}
            Curve::Circle { frame, .. } => {
                if coaxial(frame.o, frame.z) {
                    let span = m.p0(e).dist(m.p1(e));
                    let d = m.mid(e) - o;
                    let d = d - ax * d.dot(ax);
                    if seam.map_or(true, |s| span < s.0) && d.len() > 1e-9 {
                        seam = Some((span, d.normalize()));
                    }
                }
            }
            _ => return None,
        }
    }
    let sx = seam?.1;
    let sy = ax.cross(sx);
    let frame_plane = |tree: &mut ck_recog::emit::Tree, x: Vec3, y: Vec3| -> Json {
        let id = tree.ids.next();
        tree.push(Obj::new().put("type", "plane").put("id", id.as_str()).put("mode", "frame").put("origin", vec3(o)).put("xDir", vec3(x)).put("yDir", vec3(y)).build());
        Obj::new().put("kind", "feature").put("id", id.as_str()).build()
    };
    // the reordered document: the steps before the boss as read, the annulus, the steps between, the rings, the cut
    let mut tree = ck_recog::emit::Tree { ids: ck_recog::emit::Ids(cx.tree.ids.0), feats: cx.tree.feats[..ranges[k].0].to_vec() };
    let mut new_steps: Vec<Step> = steps[..k].to_vec();
    let mut new_ranges: Vec<(usize, usize)> = ranges[..k].to_vec();
    let step = |j: usize, note: String| Step { sw_id: all[j].id, name: all[j].name.clone(), kind: all[j].kind.clone(), faces: groups.get(&all[j].id).map_or(0, |v| v.len()), status: Status::Rebuilt, note };
    let first = tree.feats.len();
    let pl = frame_plane(&mut tree, sx, sy);
    let sid = tree.ids.next();
    let mut sk = Sk::new(sid.clone(), pl);
    let pc = sk.point(0.0, 0.0);
    sk.circle(&pc, r_out);
    sk.circle(&pc, r_in);
    tree.push(sk.json());
    let bid = tree.ids.next();
    tree.push(Obj::new().put("type", "extrude").put("id", bid.as_str()).put("op", "add").put("sketch", sid.as_str()).put("end", "blind").put("depth", n(depth)).build());
    new_ranges.push((first, tree.feats.len()));
    new_steps.push(step(k, format!("reordered (geometry-equivalent, not the file's order): the full annulus R{:.4} to R{:.4}, depth {:.4}, its openings cut after the fillets", r_in, r_out, depth)));
    for &j in &between {
        let (a, b) = ranges[j];
        let at = tree.feats.len();
        tree.feats.extend_from_slice(&cx.tree.feats[a..b]);
        new_ranges.push((at, tree.feats.len()));
        new_steps.push(steps[j].clone());
    }
    // rings as fillets where a pick exists, else as revolves; a fillet regen error turns that ring into a revolve
    let ring_feats = |tree: &mut ck_recog::emit::Tree, ring: &[((f64, f64), f64, (f64, f64), (f64, f64), Option<FaceId>)], radius: f64, as_fillet: bool| -> Option<String> {
        if as_fillet {
            let p1 = ring[0].2;
            let cap = if (p1.1 - depth).abs() < 1e-6 { "C1" } else if p1.1.abs() < 1e-6 { "C0" } else { return None };
            let mut edges = vec![];
            for (_, _, p1, p2, other) in ring {
                let name = cx.lookup((*other)?)?;
                let q = (p2.0, p1.1);
                if !edges.iter().any(|e: &(String, (f64, f64))| e.0 == name && (e.1 .0 - q.0).abs() < 1e-6 && (e.1 .1 - q.1).abs() < 1e-6) {
                    edges.push((name, q));
                }
            }
            let picks: Vec<Json> = edges.iter().map(|(name, q)| Obj::new().put("faces", arr(vec![s(name), s(&format!("{bid}.{cap}"))])).put("at", vec3(o + sx * q.0 + ax * q.1)).build()).collect();
            let id = tree.ids.next();
            tree.push(Obj::new().put("type", "fillet").put("id", id.as_str()).put("radius", n(radius)).put("edges", arr(picks)).build());
            return Some(format!("reordered (geometry-equivalent, not the file's order): full-circle fillet R{radius:.4} of the annulus before its openings are cut"));
        }
        let mut done: Vec<(f64, f64, f64)> = vec![];
        for (tube, minor, p1, p2, _) in ring {
            if done.iter().any(|d| (d.0 - tube.0).abs() < 1e-6 && (d.1 - tube.1).abs() < 1e-6 && (d.2 - minor).abs() < 1e-6) {
                continue;
            }
            done.push((tube.0, tube.1, *minor));
            // the section: the arc from the plane support to the other, the corner the two supports make
            let q = (p2.0, p1.1);
            let pl = frame_plane(tree, sx, ax);
            let sid = tree.ids.next();
            let mut sk = Sk::new(sid.clone(), pl);
            let (a1, a2, ac, aq) = (sk.point(p1.0, p1.1), sk.point(p2.0, p2.1), sk.point(tube.0, tube.1), sk.point(q.0, q.1));
            // counter-clockwise from the start: the short arc facing the corner
            let (u, v) = ((p1.0 - tube.0, p1.1 - tube.1), (p2.0 - tube.0, p2.1 - tube.1));
            if u.0 * v.1 - u.1 * v.0 > 0.0 {
                sk.arc(&ac, &a1, &a2);
            } else {
                sk.arc(&ac, &a2, &a1);
            }
            sk.line(&a2, &aq, false);
            sk.line(&aq, &a1, false);
            let (z0, z1) = (sk.point(0.0, 0.0), sk.point(0.0, 1.0));
            let axis = sk.line(&z0, &z1, true);
            tree.push(sk.json());
            let rid = tree.ids.next();
            tree.push(Obj::new().put("type", "revolve").put("id", rid.as_str()).put("op", "add").put("sketch", sid.as_str()).put("axis", axis.as_str()).put("angle", n(360.0)).build());
        }
        Some(format!("reordered (geometry-equivalent, not the file's order): full-circle round R{radius:.4} (the revolve of its section) of the annulus before its openings are cut"))
    };
    let radius_of = |j: usize| crate::dim(cx, all[j], "D1");
    let mut as_fillet: HashMap<usize, bool> = rounds.iter().map(|r| (r.0, matches!(&r.1, Round::Ring(v) if v.iter().all(|x| x.4.is_some())))).collect();
    for _attempt in 0..2 {
        let mut tr = ck_recog::emit::Tree { ids: ck_recog::emit::Ids(tree.ids.0), feats: tree.feats.clone() };
        let mut st = new_steps.clone();
        let mut rg = new_ranges.clone();
        let mut ring_ids: Vec<(usize, String)> = vec![];
        let mut corner_steps = vec![];
        for (j, r) in &rounds {
            match r {
                Round::Corner => corner_steps.push(*j),
                Round::Ring(v) => {
                    let radius = v[0].1;
                    let at = tr.feats.len();
                    let note = ring_feats(&mut tr, v, radius, as_fillet[j]).or_else(|| ring_feats(&mut tr, v, radius, false))?;
                    if let Some(id) = tr.feats.get(at).and_then(|f| f.get_str("id")) {
                        ring_ids.push((*j, id.to_string()));
                    }
                    rg.push((at, tr.feats.len()));
                    let mut s_ = step(*j, note);
                    if let Some(d) = radius_of(*j) {
                        if (d - radius).abs() > 1e-6 * (1.0 + radius) {
                            s_.note.push_str(&format!(" (the file lists {d})"));
                        }
                    }
                    st.push(s_);
                }
            }
        }
        // the openings cut, through the web and the rings above it
        let at = tr.feats.len();
        let pl = frame_plane(&mut tr, sx, sy);
        let sid = tr.ids.next();
        let mut sk = Sk::new(sid.clone(), pl);
        let mut pts: Vec<(Vec3, String)> = vec![];
        let mut pt = |sk: &mut Sk, p: Vec3| -> String {
            if let Some(x) = pts.iter().find(|x| x.0.dist(p) < 1e-6) {
                return x.1.clone();
            }
            let d = p - o;
            let id = sk.point(d.dot(sx), d.dot(sy));
            pts.push((p, id.clone()));
            id
        };
        for &e in &loop_edges {
            let (p0, p1) = (m.p0(e), m.p1(e));
            match m.curve(e) {
                Curve::Circle { frame, r } if p0.dist(p1) < 1e-9 => {
                    let cid = pt(&mut sk, frame.o);
                    sk.circle(&cid, *r);
                }
                Curve::Line { .. } => {
                    let (a, b) = (pt(&mut sk, p0), pt(&mut sk, p1));
                    sk.line(&a, &b, false);
                }
                Curve::Circle { frame, .. } => {
                    let cc = frame.o;
                    let (a, b, cid) = (pt(&mut sk, p0), pt(&mut sk, p1), pt(&mut sk, cc));
                    let xy = |p: Vec3| ((p - o).dot(sx), (p - o).dot(sy));
                    let (qa, qc, qm) = (xy(p0), xy(cc), xy(m.mid(e)));
                    let ccw = (qa.0 - qc.0) * (qm.1 - qc.1) - (qa.1 - qc.1) * (qm.0 - qc.0) > 0.0;
                    if ccw {
                        sk.arc(&cid, &a, &b);
                    } else {
                        sk.arc(&cid, &b, &a);
                    }
                }
                _ => return None,
            }
        }
        tr.push(sk.json());
        let (b0, b1) = m.b.bbox();
        let reach = hgt(b0).abs() + hgt(b1).abs() + (b1 - b0).len() + 1.0;
        let cid = tr.ids.next();
        tr.push(Obj::new().put("type", "extrude").put("id", cid.as_str()).put("op", "cut").put("sketch", sid.as_str()).put("end", "blind").put("depth", n(reach)).put("flip", true).build());
        for j in corner_steps {
            rg.push((at, tr.feats.len()));
            let r = radius_of(j).map_or(String::new(), |d| format!("R{d:.4} "));
            st.push(step(j, format!("reordered (geometry-equivalent, not the file's order): the {r}corners of the openings, arcs of the openings cut")));
        }
        // the cut is the last feature: the steps are listed in file order
        let order: HashMap<i64, usize> = all.iter().enumerate().map(|(i, f)| (f.id, i)).collect();
        let mut idx: Vec<usize> = (0..st.len()).collect();
        idx.sort_by_key(|&i| order.get(&st[i].sw_id).copied().unwrap_or(usize::MAX));
        let (st, rg): (Vec<Step>, Vec<(usize, usize)>) = idx.into_iter().map(|i| (st[i].clone(), rg[i])).unzip();
        let mut out = Outcome { features: tr.feats, steps: st, body: None, verify: None, errors: vec![], fallback: None, notes: vec![], prefix: 0, remainder: None, checks: vec![], ranges: rg };
        full_regen(t, &mut out);
        if out.fallback.is_none() && (out.exact() || out.close()) {
            out.notes.push("spoked web read back as the full annulus with its openings cut after the fillets (reordered, geometry-equivalent; the whole body verifies)".into());
            return Some(out);
        }
        // a ring whose fillet did not regenerate is a revolve next time
        let mut changed = false;
        for (j, id) in &ring_ids {
            if out.errors.iter().any(|e| e.starts_with(&format!("{id}:"))) && as_fillet[j] {
                as_fillet.insert(*j, false);
                changed = true;
            }
        }
        if !changed {
            return None;
        }
    }
    None
}
