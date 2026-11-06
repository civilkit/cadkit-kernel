//! Steps read back in a geometry-equivalent order when a blend cannot run where the file's history put it.
//!
//! The ck-ops blend stops where a later boss stands in the way of its strips (a chamfer ramp that runs into a lug
//! boss on the floor beside it comes out partial). When the history does not rebuild and its last blend follows a
//! boss, the boss and the cuts after it are read back after that blend (the ramp is then a plain chamfer along the
//! whole edge), and, if that is not enough, a groove (an arc between two collinear lines of the boss's profile) as
//! the straight edge with the groove cut last.
//!
//! The reordered history is kept only when the whole body verifies against the file; the moved steps say so in their
//! notes (reordered, geometry-equivalent, not the file's order).

use crate::{full_regen_early as full_regen_in, Cx, Outcome, Status, Step, Tagged};
use ck_json::{Json, Obj};
use ck_recog::emit::{arr, n, s, vec3};
use ck_geom::Vec3;
use ck_recog::model::{Kind, Model};
use std::collections::HashMap;

type P = (f64, f64);

fn set(j: &mut Json, key: &str, v: Json) {
    if let Json::Obj(m) = j {
        match m.iter_mut().find(|(k, _)| k == key) {
            Some(e) => e.1 = v,
            None => m.push((key.to_string(), v)),
        }
    }
}

/// An arc of the sketch between two collinear lines: (entity index, centre, the two ends, the lines' other ends).
struct Notch {
    ent: usize,
    id: String,
    pts: [String; 3],
    c: P,
    a: P,
    b: P,
}

fn notches(sk: &Json) -> Vec<Notch> {
    let mut out = vec![];
    let Some(pa) = sk.get_arr("points") else { return out };
    let pts: HashMap<&str, P> = pa.iter().filter_map(|p| Some((p.get_str("id")?, (p.get_f64("x")?, p.get_f64("y")?)))).collect();
    let Some(es) = sk.get_arr("entities") else { return out };
    for (i, e) in es.iter().enumerate() {
        if e.get_str("type") != Some("arc") || e.get_bool("construction") == Some(true) {
            continue;
        }
        let (Some(c), Some(p1), Some(p2)) = (e.get_str("c"), e.get_str("p1"), e.get_str("p2")) else { continue };
        let (Some(&cc), Some(&a), Some(&b)) = (pts.get(c), pts.get(p1), pts.get(p2)) else { continue };
        let chord = ((b.0 - a.0), (b.1 - a.1));
        let cl = chord.0.hypot(chord.1);
        if cl < 1e-6 {
            continue;
        }
        let u = (chord.0 / cl, chord.1 / cl);
        // the line leaving each end of the arc runs along the chord (outwards)
        let other = |p: &str, at: P, away: f64| -> bool {
            es.iter().any(|l| {
                if l.get_str("type") != Some("line") || l.get_bool("construction") == Some(true) {
                    return false;
                }
                let (Some(q1), Some(q2)) = (l.get_str("p1"), l.get_str("p2")) else { return false };
                let q = if q1 == p { q2 } else if q2 == p { q1 } else { return false };
                let Some(&o) = pts.get(q) else { return false };
                let d = (o.0 - at.0, o.1 - at.1);
                (d.0 * u.1 - d.1 * u.0).abs() < 1e-6 && (d.0 * u.0 + d.1 * u.1) * away > 1e-6
            })
        };
        if other(p1, a, -1.0) && other(p2, b, 1.0) {
            out.push(Notch { ent: i, id: e.get_str("id").unwrap_or("").into(), pts: [c.into(), p1.into(), p2.into()], c: cc, a, b });
        }
    }
    out
}

/// The arc's mid point (counter-clockwise from its first end to its second).
fn arc_mid(nt: &Notch) -> P {
    let ang = |p: P| (p.1 - nt.c.1).atan2(p.0 - nt.c.0);
    let r = (nt.a.0 - nt.c.0).hypot(nt.a.1 - nt.c.1);
    let mut sweep = ang(nt.b) - ang(nt.a);
    while sweep <= 0.0 {
        sweep += std::f64::consts::TAU;
    }
    let m = ang(nt.a) + sweep / 2.0;
    (nt.c.0 + r * m.cos(), nt.c.1 + r * m.sin())
}

/// The features of the steps in `order` (indices of `steps`), with the ranges they take in the new list.
fn assemble(feats: &[Json], ranges: &[(usize, usize)], order: &[usize]) -> Option<(Vec<Json>, Vec<(usize, usize)>)> {
    let mut out = vec![];
    let mut rg = vec![(0, 0); ranges.len()];
    for &i in order {
        let (a, b) = ranges[i];
        let first = out.len();
        out.extend_from_slice(feats.get(a..b)?);
        rg[i] = (first, out.len());
    }
    let total: usize = ranges.iter().map(|r| r.1 - r.0).sum();
    (out.len() == total && total == feats.len()).then_some((out, rg))
}


fn solve3(rows: [Vec3; 3], rhs: [f64; 3]) -> Option<Vec3> {
    let det = rows[0].dot(rows[1].cross(rows[2]));
    (det.abs() > 1e-9).then(|| (rows[1].cross(rows[2]) * rhs[0] + rows[2].cross(rows[0]) * rhs[1] + rows[0].cross(rows[1]) * rhs[2]) * (1.0 / det))
}

/// A straight chamfer of a concave edge whose strip ends at faces that are not square to the edge, as the file has
/// it (the strip runs on to the plane of each end face), as a ruled loft of the two end triangles: each is the
/// corner of the end plane with the two supports and the strip. The ck-ops chamfer caps such a strip square instead.
fn ramp(t: &Tagged, cx: &mut Cx, sw_id: i64, dist: f64) -> Option<Vec<Json>> {
    let m = Model::new(t.body);
    let has = |f: ck_topo::body::FaceId, id: i64| t.tags.get(f.idx()).map_or(false, |v| v.iter().any(|x| x.0 == id));
    let strip = m.faces.iter().copied().filter(|&f| has(f, sw_id) && matches!(m.kind(f), Kind::Plane { .. })).max_by(|&a, &b| m.area(a).total_cmp(&m.area(b)))?;
    let Kind::Plane { n: ns, d: ds } = m.kind(strip).clone() else { return None };
    let line = |e: ck_topo::body::EdgeId| matches!(m.curve(e), ck_geom::curve::Curve::Line { .. });
    let es: Vec<_> = m.edges(strip).into_iter().filter(|&e| line(e)).collect();
    let len = |e| m.b.edge_length(e);
    let longest = es.iter().copied().max_by(|&a, &b| len(a).total_cmp(&len(b)))?;
    let d = (m.p1(longest) - m.p0(longest)).normalize();
    let par = |e| (m.p1(e) - m.p0(e)).normalize().cross(d).len() < 1e-6;
    let plane_of = |g: ck_topo::body::FaceId| match m.kind(g) {
        Kind::Plane { n, d } => Some((*n, *d)),
        _ => None,
    };
    let same = |p: (Vec3, f64), q: (Vec3, f64)| (p.0 - q.0).len() < 1e-7 && (p.1 - q.1).abs() < 1e-6;
    // the supports: the two planes with the most rail length (a groove through the strip splits a support's rail)
    let mut rails: Vec<((Vec3, f64), f64)> = vec![];
    for &e in es.iter().filter(|&&e| par(e)) {
        let Some(p) = m.across(e, strip).and_then(plane_of) else { continue };
        match rails.iter_mut().find(|r| same(r.0, p)) {
            Some(r) => r.1 += len(e),
            None => rails.push((p, len(e))),
        }
    }
    rails.sort_by(|a, b| b.1.total_cmp(&a.1));
    // the pair of supports the listed distance is measured on: the strip is that far (square) from their corner
    let mut pair = None;
    for i in 0..rails.len() {
        for k in i + 1..rails.len() {
            let ((na, da), (nb, db)) = (rails[i].0, rails[k].0);
            let Some(c) = solve3([na, nb, d], [da, db, 0.0]) else { continue };
            if ((ns.dot(c) - ds).abs() - dist / 2f64.sqrt()).abs() < 1e-4 * (1.0 + dist) {
                pair = pair.or(Some((rails[i].0, rails[k].0)));
            }
        }
    }
    let ((na, da), (nb, db)) = pair?;
    // the end planes: inclined to the edge, meeting a support
    let mut ends: Vec<(Vec3, f64)> = vec![];
    for &e in es.iter().filter(|&&e| !par(e)) {
        let Some(g) = m.across(e, strip) else { continue };
        let Some(p) = plane_of(g) else { continue };
        let meets = m.edges(g).into_iter().any(|x| m.across(x, g).and_then(plane_of).map_or(false, |q| same(q, (na, da)) || same(q, (nb, db))));
        if p.0.dot(d).abs() > 1.0 - 1e-3 || p.0.dot(d).abs() < 1e-3 || !meets || same(p, (na, da)) || same(p, (nb, db)) || ends.iter().any(|q| same(*q, p)) {
            continue;
        }
        ends.push(p);
    }
    if ends.len() != 2 {
        return None;
    }
    let mut out = vec![];
    let mut sks = vec![];
    for (ng, dg) in ends {
        let c = solve3([na, nb, ng], [da, db, dg])?;
        let pa = solve3([na, ns, ng], [da, ds, dg])?;
        let pb = solve3([nb, ns, ng], [db, ds, dg])?;
        let x = (pb - c).normalize();
        let y0 = pa - c;
        let y = (y0 - x * y0.dot(x)).normalize();
        let pid = cx.tree.ids.next();
        let sid = cx.tree.ids.next();
        let pt = |id: &str, p: (f64, f64)| Obj::new().put("id", id).put("x", n(p.0)).put("y", n(p.1)).build();
        let ln = |id: &str, a: &str, b: &str| Obj::new().put("type", "line").put("id", id).put("p1", a).put("p2", b).build();
        out.push(Obj::new().put("type", "plane").put("id", pid.as_str()).put("mode", "frame").put("origin", vec3(c)).put("xDir", vec3(x)).put("yDir", vec3(y)).build());
        out.push(
            Obj::new()
                .put("type", "sketch")
                .put("id", sid.as_str())
                .put("plane", Obj::new().put("kind", "feature").put("id", pid.as_str()).build())
                .put("points", arr(vec![pt("p0", (0.0, 0.0)), pt("p1", ((pb - c).dot(x), 0.0)), pt("p2", (y0.dot(x), y0.dot(y)))]))
                .put("entities", arr(vec![ln("t1", "p0", "p1"), ln("t2", "p1", "p2"), ln("t3", "p2", "p0")]))
                .put("constraints", arr(vec![]))
                .build(),
        );
        sks.push(sid);
    }
    let lid = cx.tree.ids.next();
    out.push(Obj::new().put("type", "loft").put("id", lid.as_str()).put("op", "add").put("sketches", arr(sks.iter().map(|x| s(x)).collect())).build());
    Some(out)
}

/// A copy of a step's features with fresh ids (every string equal to an id of the slice is renamed).
fn copy_fresh(cx: &mut Cx, feats: &[Json]) -> Vec<Json> {
    let mut map: HashMap<String, String> = HashMap::new();
    for f in feats {
        if let Some(id) = f.get_str("id") {
            map.insert(id.to_string(), cx.tree.ids.next());
        }
    }
    fn walk(j: &Json, map: &HashMap<String, String>) -> Json {
        match j {
            Json::Str(x) => Json::Str(map.get(x).cloned().unwrap_or_else(|| x.clone())),
            Json::Arr(a) => Json::Arr(a.iter().map(|v| walk(v, map)).collect()),
            Json::Obj(m) => Json::Obj(m.iter().map(|(k, v)| (k.clone(), if k == "name" { v.clone() } else { walk(v, map) })).collect()),
            other => other.clone(),
        }
    }
    feats.iter().map(|f| walk(f, &map)).collect()
}

const REORDERED: &str = "reordered (geometry-equivalent, not the file's order)";

/// The boss's sketch (`feats[si]`) with the arc of `nt` as its chord, and the features of the groove cut.
fn groove(cx: &mut Cx, feats: &[Json], si: usize, nt: &Notch) -> Option<(Vec<Json>, Vec<Json>)> {
    let sk = &feats[si];
    let mut fs = feats.to_vec();
    let mut sk2 = sk.clone();
    let mut es: Vec<Json> = sk.get_arr("entities")?.to_vec();
    es[nt.ent] = Obj::new().put("type", "line").put("id", nt.id.as_str()).put("p1", nt.pts[1].as_str()).put("p2", nt.pts[2].as_str()).build();
    set(&mut sk2, "entities", Json::Arr(es));
    fs[si] = sk2;
    // the groove: the arc closed by lines out of the body, cut through the whole part both ways
    let mid = arc_mid(nt);
    let cm = ((nt.a.0 + nt.b.0) / 2.0, (nt.a.1 + nt.b.1) / 2.0);
    let bulge = (mid.0 - cm.0, mid.1 - cm.1);
    let bl = bulge.0.hypot(bulge.1);
    if bl < 1e-6 {
        return None;
    }
    let pad = (-bulge.0 / bl, -bulge.1 / bl);
    let (a2, b2) = ((nt.a.0 + pad.0, nt.a.1 + pad.1), (nt.b.0 + pad.0, nt.b.1 + pad.1));
    let pt = |id: &str, p: P| Obj::new().put("id", id).put("x", n(p.0)).put("y", n(p.1)).build();
    let mut arc = sk.get_arr("entities")?[nt.ent].clone();
    set(&mut arc, "id", s("g1"));
    set(&mut arc, "c", s("c"));
    set(&mut arc, "p1", s("a"));
    set(&mut arc, "p2", s("b"));
    let line = |id: &str, p1: &str, p2: &str| Obj::new().put("type", "line").put("id", id).put("p1", p1).put("p2", p2).build();
    let sid = cx.tree.ids.next();
    let eid = cx.tree.ids.next();
    let extra = vec![
        Obj::new()
            .put("type", "sketch")
            .put("id", sid.as_str())
            .put("plane", sk.get("plane")?.clone())
            .put("points", arr(vec![pt("a", nt.a), pt("b", nt.b), pt("c", nt.c), pt("a2", a2), pt("b2", b2)]))
            .put("entities", arr(vec![arc, line("g2", "b", "b2"), line("g3", "b2", "a2"), line("g4", "a2", "a")]))
            .put("constraints", arr(vec![]))
            .build(),
        Obj::new().put("type", "extrude").put("id", eid.as_str()).put("op", "cut").put("sketch", sid.as_str()).put("end", "throughBoth").build(),
    ];
    Some((fs, extra))
}

/// Some face of step `a` borders a face of step `b` on the final body.
pub(crate) fn touches(t: &Tagged, m: &Model, a: i64, b: i64) -> bool {
    let has = |f: ck_topo::body::FaceId, id: i64| t.tags.get(f.idx()).map_or(false, |v| v.iter().any(|x| x.0 == id));
    m.faces.iter().any(|&f| has(f, a) && m.edges(f).into_iter().filter_map(|e| m.across(e, f)).any(|g| has(g, b)))
}

pub(crate) fn reformulate(t: &Tagged, cx: &mut Cx, steps: &[Step], ranges: &[(usize, usize)]) -> Option<Outcome> {
    reformulate_in(t, cx, steps, ranges)
}

fn reformulate_in(t: &Tagged, cx: &mut Cx, steps: &[Step], ranges: &[(usize, usize)]) -> Option<Outcome> {
    if steps.iter().any(|s| s.status == Status::Unsupported) || ranges.len() != steps.len() {
        return None;
    }
    let all: Vec<&ck_sldprt::meta::SwFeature> = t.features.iter().filter(|f| crate::classify(f) != crate::Class::Skip).collect();
    if all.len() != steps.len() {
        return None;
    }
    let class = |k: usize| crate::classify(all[k]);
    let feats = cx.tree.feats.clone();
    let live = |k: usize| steps[k].status == Status::Rebuilt && ranges[k].1 > ranges[k].0;
    // the last blend that was read back
    let j = (0..steps.len()).rev().find(|&k| live(k) && matches!(class(k), crate::Class::Chamfer | crate::Class::Fillet))?;
    // a different order helps where the regeneration failed or the last step read back is the blend itself (it
    // lands on the wrong edges). A body that regenerates cleanly and differs with steps after the last blend is
    // not shown to be an ordering problem, and trying every order on it costs seconds (each wrong order runs the
    // later features on the wrong body: a rib there took ten times the whole regeneration)
    if !cx.regen_failed && (j + 1..steps.len()).any(live) {
        return None;
    }
    let is_boss = |k: usize| live(k) && matches!(class(k), crate::Class::Extrude { cut: false });
    // a boss can stand in the way of a blend only where the two meet on the final body (the blend's round or ramp
    // runs into it); the others are not tried (each try runs the history from the moved step on)
    let model = Model::new(t.body);
    let meets = |k: usize, b: usize| touches(t, &model, steps[b].sw_id, steps[k].sw_id);
    // the variants of the boss profiles: as read, or one groove read as the straight edge (cut last)
    let mut variants: Vec<(Vec<Json>, Vec<Json>, Option<usize>)> = vec![(feats.clone(), vec![], None)];
    for k in (0..j).filter(|&k| is_boss(k)) {
        let (a, b) = ranges[k];
        let Some(si) = (a..b.min(feats.len())).find(|&i| feats[i].get_str("type") == Some("sketch")) else { continue };
        for nt in notches(&feats[si]) {
            if let Some((fs, extra)) = groove(cx, &feats, si, &nt) {
                variants.push((fs, extra, Some(k)));
            }
        }
    }
    // the steps moved after the blend: none, or a boss with the cuts after it (before the blend), or the boss alone
    let mut moves: Vec<Vec<usize>> = vec![vec![]];
    for k in (0..j).filter(|&k| is_boss(k) && meets(k, j)) {
        let cuts: Vec<usize> = (k + 1..j).filter(|&c| live(c) && matches!(class(c), crate::Class::Extrude { cut: true } | crate::Class::Hole)).collect();
        if !cuts.is_empty() {
            let mut v = vec![k];
            v.extend(cuts);
            moves.push(v);
        }
        moves.push(vec![k]);
    }
    for (vf, extra, groove_of) in &variants {
        for mv in &moves {
            if mv.is_empty() && groove_of.is_none() {
                continue;
            }
            let mut order: Vec<usize> = (0..=j).filter(|i| !mv.contains(i)).collect();
            order.extend(mv.iter().copied());
            order.extend(j + 1..steps.len());
            let Some((mut fs, rg)) = assemble(vf, ranges, &order) else { continue };
            fs.extend(extra.iter().cloned());
            let mut st = steps.to_vec();
            for &i in mv {
                st[i].note.push_str(&format!("; {REORDERED}: after {}", steps[j].name));
            }
            if let Some(k) = groove_of {
                st[*k].note.push_str(&format!("; groove read back as the straight edge and cut after the last step ({REORDERED})"));
            }
            let mut out = Outcome { features: fs, steps: st, body: None, verify: None, errors: vec![], fallback: None, notes: vec![], prefix: 0, remainder: None, checks: vec![], ranges: rg };
            full_regen_in(t, &mut out, &cx.sess);
            if out.fallback.is_none() && (out.exact() || out.close()) {
                let names = mv.iter().map(|&i| steps[i].name.as_str()).collect::<Vec<_>>().join(", ");
                out.notes.push(format!(
                    "{}{}{}(reordered, geometry-equivalent; the whole body verifies)",
                    if mv.is_empty() { String::new() } else { format!("{names} read back after {}; ", steps[j].name) },
                    if groove_of.is_some() { "a groove in a boss's edge read back as the straight edge with the groove cut last " } else { "" },
                    ""
                ));
                return Some(out);
            }
        }
    }
    // the blend as a loft of its end triangles, with the groove cut last; then, where a blend of an earlier step
    // overshoots into a boss added before it, that boss read back after the blend
    let (a, b) = ranges[j];
    for (vf, extra, groove_of) in variants.iter().filter(|v| v.2.is_some()).chain(variants.iter().take(1)) {
        let Some(loft) = all[j].dims.iter().find(|x| x.name == "D1").and_then(|x| x.value).and_then(|dist| ramp(t, cx, steps[j].sw_id, dist)) else { break };
        let mut fs: Vec<Json> = vf[..a].to_vec();
        fs.extend(loft);
        let shift = fs.len() as isize - b as isize;
        fs.extend_from_slice(&vf[b..]);
        let rg: Vec<(usize, usize)> = ranges.iter().enumerate().map(|(i, &(x, y))| if i < j { (x, y) } else if i == j { (a, (b as isize + shift) as usize) } else { ((x as isize + shift) as usize, (y as isize + shift) as usize) }).collect();
        let blends: Vec<usize> = (0..j).filter(|&m| live(m) && matches!(class(m), crate::Class::Chamfer | crate::Class::Fillet)).collect();
        let mut tries: Vec<(Option<(usize, usize)>, bool)> = vec![(None, false)];
        for k in (0..j).filter(|&k| is_boss(k)) {
            tries.extend(blends.iter().filter(|&&m| m > k && meets(k, m)).map(|&m| (Some((k, m)), false)));
            // the boss added again after the blend (the blend overshoots into it), the cuts between moved behind
            tries.extend(blends.iter().filter(|&&m| m > k && meets(k, m)).map(|&m| (Some((k, m)), true)));
        }
        for (mvd, refill) in tries {
            let order: Vec<usize> = match mvd {
                None => (0..steps.len()).collect(),
                Some((k, m)) if refill => {
                    let cuts: Vec<usize> = (k + 1..m).filter(|&c| live(c) && matches!(class(c), crate::Class::Extrude { cut: true } | crate::Class::Hole)).collect();
                    let mut o: Vec<usize> = (0..steps.len()).filter(|i| !cuts.contains(i)).collect();
                    let at = o.iter().position(|&i| i == m).unwrap() + 1;
                    for (n_, c) in cuts.into_iter().enumerate() {
                        o.insert(at + n_, c);
                    }
                    o
                }
                Some((k, m)) => {
                    let mut o: Vec<usize> = (0..steps.len()).filter(|&i| i != k).collect();
                    let at = o.iter().position(|&i| i == m).unwrap() + 1;
                    o.insert(at, k);
                    o
                }
            };
            let Some((mut fs2, rg2)) = assemble(&fs, &rg, &order) else { continue };
            if refill {
                let (k, m) = mvd.unwrap();
                // the copy goes in right after the blend, ahead of the cuts moved behind it
                let at = rg2[m].1;
                let copy = copy_fresh(cx, &fs[rg[k].0..rg[k].1]);
                let tail = fs2.split_off(at);
                fs2.extend(copy);
                fs2.extend(tail);
            }
            fs2.extend(extra.iter().cloned());
            let mut st = steps.to_vec();
            st[j].note.push_str(&format!("; strip read back as a loft of its end triangles ({REORDERED})"));
            if let Some((k, m)) = mvd {
                st[k].note.push_str(&format!("; {REORDERED}: after {}", steps[m].name));
            }
            let mut out = Outcome { features: fs2, steps: st, body: None, verify: None, errors: vec![], fallback: None, notes: vec![], prefix: 0, remainder: None, checks: vec![], ranges: rg2 };
            full_regen_in(t, &mut out, &cx.sess);
            if out.fallback.is_none() && (out.exact() || out.close()) {
                out.notes.push(format!(
                    "{} read back as a loft of its end triangles{}{} (reordered, geometry-equivalent; the whole body verifies)",
                    steps[j].name,
                    if groove_of.is_some() { ", the groove of a boss's edge cut last" } else { "" },
                    mvd.map_or(String::new(), |(k, m)| format!(", {} after {}", steps[k].name, steps[m].name))
                ));
                return Some(out);
            }
        }
    }
    None
}
