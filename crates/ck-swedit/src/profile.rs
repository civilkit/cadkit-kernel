//! Extrudes (boss and cut) read back from their faces: direction, extent and the sketch profile.
//!
//! The profile comes from the outline of a cap face when every edge of the cap's rim borders a side face of the
//! same feature (a rim touched by a later feature, a fillet or a merged neighbour, is not the profile any more);
//! otherwise from the cycles of the side faces through their edges parallel to the direction. Loops are drawn as
//! they are: the sketch's even-odd region rule makes windows of nested loops.

use crate::{dim_note, face_points, Cx, Fr, TOL};
use ck_geom::curve::Curve;
use ck_geom::Vec3;
use ck_json::Obj;
use ck_recog::emit::{n, Sk};
use ck_recog::model::{comp, principal, Kind, Model, PlaneSpec};
use ck_sldprt::meta::SwFeature;
use ck_topo::body::{EdgeId, FaceId};
use std::collections::HashSet;
use std::f64::consts::TAU;

type P2 = (f64, f64);

#[derive(Debug, Clone)]
pub enum Seg2 {
    Line { a: P2, b: P2 },
    /// Arc about `c` from `a` to `b`, counter-clockwise when `ccw`.
    Arc { a: P2, b: P2, c: P2, ccw: bool },
    Circle { c: P2, r: f64 },
}

/// One closed loop of the profile; each piece remembers the side faces it came from.
#[derive(Debug, Clone, Default)]
pub struct Loop2 {
    pub segs: Vec<(Seg2, Vec<FaceId>)>,
}

fn near2(p: P2, q: P2) -> bool {
    (p.0 - q.0).abs() < 1e-6 && (p.1 - q.1).abs() < 1e-6
}

fn wrap(a: f64) -> f64 {
    let r = a % TAU;
    if r < 0.0 {
        r + TAU
    } else {
        r
    }
}

fn ang(c: P2, p: P2) -> f64 {
    (p.1 - c.1).atan2(p.0 - c.0)
}

/// Directions of an extrude from its faces: every unit vector such that each face is a cap (plane normal to it) or a
/// side (plane containing it, cylinder about it), with its caps and sides.
pub fn classify_dir(m: &Model, faces: &[FaceId]) -> Result<Vec<(Vec3, Vec<FaceId>, Vec<FaceId>)>, String> {
    let mut cands: Vec<Vec3> = vec![];
    let mut normals: Vec<Vec3> = vec![];
    for &f in faces {
        match m.kind(f) {
            Kind::Plane { n, .. } => {
                cands.push(*n);
                normals.push(*n);
            }
            Kind::Cyl { a, .. } => cands.push(*a),
            Kind::Cone { .. } => return Err("drafted side (cone face)".into()),
            k => return Err(format!("{} face in an extrude", k.name())),
        }
    }
    if normals.len() <= 16 {
        for i in 0..normals.len() {
            for j in i + 1..normals.len() {
                let c = normals[i].cross(normals[j]);
                if c.len() > 1e-6 {
                    cands.push(c.normalize());
                }
            }
        }
    }
    let mut out: Vec<(Vec3, Vec<FaceId>, Vec<FaceId>)> = vec![];
    for d in cands {
        if out.iter().any(|o| o.0.dot(d).abs() > 1.0 - 1e-6) {
            continue;
        }
        let mut caps = vec![];
        let mut sides = vec![];
        let mut ok = true;
        for &f in faces {
            match m.kind(f) {
                Kind::Plane { n, .. } => {
                    let t = n.dot(d).abs();
                    if t > 1.0 - 1e-6 {
                        caps.push(f);
                    } else if t < 1e-6 {
                        sides.push(f);
                    } else {
                        ok = false;
                    }
                }
                Kind::Cyl { a, .. } => {
                    if a.dot(d).abs() > 1.0 - 1e-6 {
                        sides.push(f);
                    } else {
                        ok = false;
                    }
                }
                _ => ok = false,
            }
            if !ok {
                break;
            }
        }
        if ok {
            out.push((d, caps, sides));
        }
    }
    if out.is_empty() {
        let kinds: Vec<String> = faces.iter().map(|&f| m.kind(f).name().to_string()).collect();
        return Err(format!("faces do not share an extrusion direction ({} faces: {})", faces.len(), kinds.join(", ")));
    }
    Ok(out)
}

/// Points along a loop in its order (eight per edge, so a loop of one closed circle has an area).
fn loop_points(b: &ck_topo::body::Body, l: ck_topo::body::LoopId) -> Vec<Vec3> {
    let mut pts = vec![];
    for &c in &b.lp(l).coedges {
        let ce = b.coedge(c);
        let ed = b.edge(ce.edge);
        for k in 0..8 {
            let f = k as f64 / 8.0;
            let t = if ce.rev { ed.t1 + (ed.t0 - ed.t1) * f } else { ed.t0 + (ed.t1 - ed.t0) * f };
            pts.push(ed.curve.eval(t));
        }
    }
    pts
}

/// The cap's outer loop (largest by Newell area) borders only `sides`.
pub fn rim_in(m: &Model, cap: FaceId, sides: &[FaceId]) -> bool {
    let b = m.b;
    let area = |l: ck_topo::body::LoopId| -> f64 {
        let pts = loop_points(b, l);
        let mut a = Vec3::ZERO;
        for i in 0..pts.len() {
            a = a + pts[i].cross(pts[(i + 1) % pts.len()]);
        }
        a.len()
    };
    let Some(outer) = b.face(cap).loops.iter().copied().max_by(|&x, &y| area(x).total_cmp(&area(y))) else { return false };
    b.lp(outer).coedges.iter().all(|&c| m.across(b.coedge(c).edge, cap).map_or(false, |g| sides.contains(&g)))
}

/// Merge consecutive collinear lines and co-circular arcs (pieces of one profile entity split by other features);
/// a loop that is one full arc becomes a circle.
pub fn merge_segs(segs: &mut Vec<(Seg2, Vec<FaceId>)>) {
    let merge = |x: &Seg2, y: &Seg2| -> Option<Seg2> {
        match (x, y) {
            (Seg2::Line { a: a1, b: b1 }, Seg2::Line { a: a2, b: b2 }) if near2(*b1, *a2) => {
                let (u, v) = ((b1.0 - a1.0, b1.1 - a1.1), (b2.0 - a2.0, b2.1 - a2.1));
                let (lu, lv) = (u.0.hypot(u.1), v.0.hypot(v.1));
                if lu < 1e-9 || lv < 1e-9 {
                    return None;
                }
                let cross = (u.0 * v.1 - u.1 * v.0) / (lu * lv);
                let dot = (u.0 * v.0 + u.1 * v.1) / (lu * lv);
                (cross.abs() < 1e-7 && dot > 0.0).then(|| Seg2::Line { a: *a1, b: *b2 })
            }
            (Seg2::Arc { a: a1, b: b1, c: c1, ccw: w1 }, Seg2::Arc { a: a2, b: b2, c: c2, ccw: w2 }) if near2(*b1, *a2) && near2(*c1, *c2) && w1 == w2 => {
                Some(Seg2::Arc { a: *a1, b: *b2, c: *c1, ccw: *w1 })
            }
            _ => None,
        }
    };
    let mut changed = true;
    while changed && segs.len() > 1 {
        changed = false;
        let n = segs.len();
        for i in 0..n {
            let j = (i + 1) % n;
            if i == j {
                break;
            }
            if let Some(s) = merge(&segs[i].0, &segs[j].0) {
                let (_, fj) = segs.remove(j);
                let i = if j < i { i - 1 } else { i };
                segs[i].0 = s;
                for f in fj {
                    if !segs[i].1.contains(&f) {
                        segs[i].1.push(f);
                    }
                }
                changed = true;
                break;
            }
        }
    }
    if segs.len() == 1 {
        if let Seg2::Arc { a, b, c, .. } = segs[0].0 {
            if near2(a, b) {
                segs[0].0 = Seg2::Circle { c, r: (a.0 - c.0).hypot(a.1 - c.1) };
            }
        }
    }
}

/// The piece of the profile a coedge of a planar face traces in the sketch frame.
fn coedge_seg(m: &Model, c: ck_topo::body::CoedgeId, fr: &Fr) -> Result<Seg2, String> {
    let b = m.b;
    let ce = b.coedge(c);
    let (va, vb) = (b.coedge_start(c), b.coedge_end(c));
    let (pa, pb) = (b.vertex(va).p, b.vertex(vb).p);
    match &b.edge(ce.edge).curve {
        Curve::Line { .. } => Ok(Seg2::Line { a: fr.xy(pa), b: fr.xy(pb) }),
        Curve::Circle { frame, r } => {
            if va == vb || pa.dist(pb) < TOL {
                Ok(Seg2::Circle { c: fr.xy(frame.o), r: *r })
            } else {
                Ok(Seg2::Arc { a: fr.xy(pa), b: fr.xy(pb), c: fr.xy(frame.o), ccw: !ce.rev == (frame.z.dot(fr.z) > 0.0) })
            }
        }
        Curve::Ellipse { .. } => Err("profile edge is an ellipse".into()),
        _ => Err("profile edge is a free curve".into()),
    }
}

/// The profile from the outline of a cap face. Loops whose rim borders only `sides` are taken; the outer loop
/// must, inner loops that do not (holes and bosses of later features) are left out.
pub fn cap_outline(m: &Model, cap: FaceId, sides: &HashSet<FaceId>, fr: &Fr) -> Result<Vec<Loop2>, String> {
    cap_outline_later(m, cap, sides, &HashSet::new(), fr)
}

/// As `cap_outline`, but a rim edge shared with a face of `later` (made by a later feature, which cut into this
/// extrude) is taken as it is: the outline is the profile with those cuts already applied, and the later feature
/// then removes nothing more.
pub fn cap_outline_later(m: &Model, cap: FaceId, sides: &HashSet<FaceId>, later: &HashSet<FaceId>, fr: &Fr) -> Result<Vec<Loop2>, String> {
    let b = m.b;
    let face = b.face(cap);
    // the outer loop: largest polygon area of the loop's vertices and edge mid points
    let area_of = |l: ck_topo::body::LoopId| -> f64 {
        let pts: Vec<P2> = loop_points(b, l).into_iter().map(|p| fr.xy(p)).collect();
        let mut a = 0.0;
        for i in 0..pts.len() {
            let (p, q) = (pts[i], pts[(i + 1) % pts.len()]);
            a += p.0 * q.1 - q.0 * p.1;
        }
        a.abs()
    };
    let outer = face.loops.iter().copied().max_by(|&x, &y| area_of(x).total_cmp(&area_of(y))).ok_or("cap without loops")?;
    let mut out = vec![];
    for &l in &face.loops {
        let mut segs = vec![];
        let mut ok = true;
        let mut why = String::new();
        for &c in &b.lp(l).coedges {
            let e = b.coedge(c).edge;
            let Some(other) = m.across(e, cap) else {
                why = format!("edge with {} faces", b.edge_faces(e).len());
                ok = false;
                break;
            };
            let is_later = later.contains(&other);
            if !sides.contains(&other) && !is_later {
                why = format!("rim borders {}", b.face_id_str(other));
                ok = false;
                break;
            }
            match coedge_seg(m, c, fr) {
                Ok(s) => segs.push((s, if is_later { vec![] } else { vec![other] })),
                Err(e) => {
                    if l == outer {
                        return Err(e);
                    }
                    ok = false;
                    break;
                }
            }
        }
        if !ok {
            if l == outer {
                return Err(format!("cap {} rim touched by other features ({why})", b.face_id_str(cap)));
            }
            continue;
        }
        merge_segs(&mut segs);
        if !later.is_empty() {
            resharpen(&mut segs);
        }
        out.push(Loop2 { segs });
    }
    Ok(out)
}

/// An arc made by a later round (a fillet the cap's rim runs along) that lies between two arcs of later rounds
/// (fillets of its ends) is a pulled-in copy of the profile's own arc: the ends' round circles of radius `R` about
/// `cb` are tangent to the profile arc about `o`, so that arc has radius `|cb - o| -/+ R`. Put the arc there and cut the
/// neighbours at the tangent points: the profile then reaches the faces the rounds ran out onto.
fn resharpen(segs: &mut Vec<(Seg2, Vec<FaceId>)>) {
    let n = segs.len();
    if n < 3 {
        return;
    }
    let arc = |s: &Seg2| match s {
        Seg2::Arc { a, b, c, ccw } => Some((*a, *b, *c, *ccw, (a.0 - c.0).hypot(a.1 - c.1))),
        _ => None,
    };
    for i in 0..n {
        let (ib, ic) = ((i + n - 1) % n, (i + 1) % n);
        if !(segs[i].1.is_empty() && segs[ib].1.is_empty() && segs[ic].1.is_empty()) {
            continue;
        }
        let (Some((aa, ab, o, ccw, ra)), Some((ba, bb, cb, bccw, rb)), Some((ca, cbb, cc, cccw, rc))) = (arc(&segs[i].0), arc(&segs[ib].0), arc(&segs[ic].0)) else { continue };
        if !(near2(bb, aa) && near2(ab, ca)) || (cb.0 - o.0).hypot(cb.1 - o.1) < 1e-9 || (cc.0 - o.0).hypot(cc.1 - o.1) < 1e-9 {
            continue;
        }
        let (db, dc) = ((cb.0 - o.0).hypot(cb.1 - o.1), (cc.0 - o.0).hypot(cc.1 - o.1));
        let pick = |d: f64, r: f64| [d - r, d + r].into_iter().filter(|x| *x > 1e-9).min_by(|x, y| (x - ra).abs().total_cmp(&(y - ra).abs()));
        let (Some(r1), Some(r2)) = (pick(db, rb), pick(dc, rc)) else { continue };
        if (r1 - r2).abs() > 1e-6 * (1.0 + r1) || (r1 - ra).abs() < 1e-6 * (1.0 + ra) {
            continue;
        }
        let tb = (o.0 + (cb.0 - o.0) / db * r1, o.1 + (cb.1 - o.1) / db * r1);
        let tc = (o.0 + (cc.0 - o.0) / dc * r1, o.1 + (cc.1 - o.1) / dc * r1);
        // the lines the end rounds are tangent to, run on to meet the arc in a sharp corner (the rounds are later
        // features, which then find their edges)
        let (il, ir) = ((i + n - 2) % n, (i + 2) % n);
        let corner = |l: &Seg2, from: P2, at_start: bool| -> Option<P2> {
            let Seg2::Line { a, b } = l else { return None };
            let (p, q) = if at_start { (*b, *a) } else { (*a, *b) };
            let _ = q;
            let d = (b.0 - a.0, b.1 - a.1);
            let len = d.0.hypot(d.1);
            if len < 1e-9 {
                return None;
            }
            let u = (d.0 / len, d.1 / len);
            // intersections of the line through `p` along `u` with the circle (o, r1)
            let w = (p.0 - o.0, p.1 - o.1);
            let bq = w.0 * u.0 + w.1 * u.1;
            let disc = bq * bq - (w.0 * w.0 + w.1 * w.1 - r1 * r1);
            if disc < 0.0 {
                return None;
            }
            let sq = disc.sqrt();
            [-bq - sq, -bq + sq].into_iter().map(|t| (p.0 + u.0 * t, p.1 + u.1 * t)).min_by(|x, y| ((x.0 - from.0).hypot(x.1 - from.1)).total_cmp(&((y.0 - from.0).hypot(y.1 - from.1))))
        };
        if il != ic && ir != ib && il != i && ir != i && il != ib && ir != ic {
            if let (Seg2::Line { a: la, b: lb }, Seg2::Line { a: ra, b: rb }) = (segs[il].0.clone(), segs[ir].0.clone()) {
                if near2(lb, ba) && near2(cbb, ra) {
                    // (the wall lines are tangent to the round arcs at their ends)
                    let tangent = |end: P2, c: P2, from: P2, to: P2| ((end.0 - c.0) * (to.0 - from.0) + (end.1 - c.1) * (to.1 - from.1)).abs() < 1e-6 * (1.0 + (to.0 - from.0).hypot(to.1 - from.1));
                    if tangent(ba, cb, la, lb) && tangent(cbb, cc, ra, rb) {
                        if let (Some(pb), Some(pc)) = (corner(&segs[il].0, ba, false), corner(&segs[ir].0, cbb, true)) {
                            segs[il].0 = Seg2::Line { a: la, b: pb };
                            segs[i].0 = Seg2::Arc { a: pb, b: pc, c: o, ccw };
                            segs[ir].0 = Seg2::Line { a: pc, b: rb };
                            let (hi, lo) = (ic.max(ib), ic.min(ib));
                            segs.remove(hi);
                            segs.remove(lo);
                            return resharpen(segs);
                        }
                    }
                }
            }
        }
        segs[ib].0 = Seg2::Arc { a: ba, b: tb, c: cb, ccw: bccw };
        segs[i].0 = Seg2::Arc { a: tb, b: tc, c: o, ccw };
        segs[ic].0 = Seg2::Arc { a: tc, b: cbb, c: cc, ccw: cccw };
        return resharpen(segs);
    }
}

/// The profile from the side faces: cycles through the edges parallel to the sketch normal, a lone cylinder
/// being a circle.
pub fn side_cycles(m: &Model, sides: &HashSet<FaceId>, fr: &Fr) -> Result<Vec<Loop2>, String> {
    let z = fr.z;
    let vertical = |e: EdgeId| matches!(m.curve(e), Curve::Line { d, .. } if d.dot(z).abs() > (1.0 - 1e-6) * d.len());
    let pos = |e: EdgeId| fr.xy(m.p0(e));
    let vedges = |w: FaceId| -> Vec<EdgeId> {
        let mut out: Vec<EdgeId> = vec![];
        for e in m.edges(w) {
            if vertical(e) && m.across(e, w).map_or(false, |g| sides.contains(&g)) && !out.iter().any(|&o| near2(pos(o), pos(e))) {
                out.push(e);
            }
        }
        out
    };
    let mut order: Vec<FaceId> = sides.iter().copied().collect();
    order.sort();
    let mut visited: HashSet<FaceId> = HashSet::new();
    let mut loops = vec![];
    for &w in &order {
        if visited.contains(&w) {
            continue;
        }
        let ves = vedges(w);
        if ves.is_empty() {
            let Kind::Cyl { o, r, .. } = m.kind(w) else {
                let nb: Vec<String> = m.edges(w).iter().filter(|&&e| vertical(e)).map(|&e| m.across(e, w).map_or("none".into(), |g| m.b.face_id_str(g))).collect();
                return Err(format!("side face {} without edges along the direction (neighbours along it: {})", m.b.face_id_str(w), nb.join(", ")));
            };
            visited.insert(w);
            loops.push(Loop2 { segs: vec![(Seg2::Circle { c: fr.xy(*o), r: *r }, vec![w])] });
            continue;
        }
        if ves.len() != 2 {
            let nb: Vec<String> = m.edges(w).iter().filter(|&&e| vertical(e)).map(|&e| m.across(e, w).map_or("none".into(), |g| m.b.face_id_str(g))).collect();
            return Err(format!("side face {} with {} edge(s) along the direction (neighbours along it: {})", m.b.face_id_str(w), ves.len(), nb.join(", ")));
        }
        let mut segs = vec![];
        let (mut cur, mut enter) = (w, ves[0]);
        loop {
            visited.insert(cur);
            let v = vedges(cur);
            if v.len() != 2 {
                return Err("side cycle broken".into());
            }
            let exit = if v[0] == enter { v[1] } else { v[0] };
            let (a, b) = (pos(enter), pos(exit));
            let seg = match m.kind(cur) {
                Kind::Plane { .. } => Seg2::Line { a, b },
                Kind::Cyl { o, .. } => {
                    let c = fr.xy(*o);
                    let (ta, tb) = (ang(c, a), ang(c, b));
                    let sep = |t: f64, u: f64| {
                        let d = wrap(t - u);
                        d.min(TAU - d)
                    };
                    let mid = m.edges(cur).into_iter().filter(|&e| !vertical(e)).find_map(|e| {
                        let q = fr.xy(m.mid(e));
                        let t = ang(c, q);
                        (sep(t, ta) > 1e-6 && sep(t, tb) > 1e-6).then_some(q)
                    });
                    let Some(mid) = mid else { return Err("arc side without an edge across the direction".into()) };
                    let tm = ang(c, mid);
                    Seg2::Arc { a, b, c, ccw: wrap(tm - ta) < wrap(tb - ta) }
                }
                _ => return Err("curved side".into()),
            };
            segs.push((seg, vec![cur]));
            let Some(next) = m.across(exit, cur) else { return Err("open side edge".into()) };
            enter = exit;
            cur = next;
            if cur == w {
                break;
            }
            if visited.contains(&cur) || segs.len() > sides.len() {
                return Err("side cycle does not close".into());
            }
        }
        merge_segs(&mut segs);
        loops.push(Loop2 { segs });
    }
    if loops.is_empty() {
        return Err("no side cycle".into());
    }
    Ok(loops)
}

#[derive(Debug, Clone, Copy)]
enum Trace {
    /// Point and unit direction.
    Line { p: P2, d: P2 },
    Circle { c: P2, r: f64 },
}

fn same_trace(a: &Trace, b: &Trace) -> bool {
    match (a, b) {
        (Trace::Line { p: p1, d: d1 }, Trace::Line { p: p2, d: d2 }) => {
            let cross = d1.0 * d2.1 - d1.1 * d2.0;
            let off = (p2.0 - p1.0) * d1.1 - (p2.1 - p1.1) * d1.0;
            cross.abs() < 1e-7 && off.abs() < 1e-6
        }
        (Trace::Circle { c: c1, r: r1 }, Trace::Circle { c: c2, r: r2 }) => near2(*c1, *c2) && (r1 - r2).abs() < 1e-6,
        _ => false,
    }
}

/// Intersections of two traces.
fn meet(a: &Trace, b: &Trace) -> Vec<P2> {
    let circ_line = |c: P2, r: f64, p: P2, d: P2| -> Vec<P2> {
        // p + t d; |p + t d - c| = r
        let f = (p.0 - c.0, p.1 - c.1);
        let bq = f.0 * d.0 + f.1 * d.1;
        let cq = f.0 * f.0 + f.1 * f.1 - r * r;
        let disc = bq * bq - cq;
        if disc < -1e-9 {
            return vec![];
        }
        let sq = disc.max(0.0).sqrt();
        vec![(p.0 + (-bq - sq) * d.0, p.1 + (-bq - sq) * d.1), (p.0 + (-bq + sq) * d.0, p.1 + (-bq + sq) * d.1)]
    };
    match (a, b) {
        (Trace::Line { p: p1, d: d1 }, Trace::Line { p: p2, d: d2 }) => {
            let den = d1.0 * d2.1 - d1.1 * d2.0;
            if den.abs() < 1e-9 {
                return vec![];
            }
            let t = ((p2.0 - p1.0) * d2.1 - (p2.1 - p1.1) * d2.0) / den;
            vec![(p1.0 + t * d1.0, p1.1 + t * d1.1)]
        }
        (Trace::Line { p, d }, Trace::Circle { c, r }) | (Trace::Circle { c, r }, Trace::Line { p, d }) => circ_line(*c, *r, *p, *d),
        (Trace::Circle { c: c1, r: r1 }, Trace::Circle { c: c2, r: r2 }) => {
            let dx = (c2.0 - c1.0, c2.1 - c1.1);
            let dd = dx.0.hypot(dx.1);
            if dd < 1e-9 {
                return vec![];
            }
            let x = (dd * dd + r1 * r1 - r2 * r2) / (2.0 * dd);
            let h2 = r1 * r1 - x * x;
            if h2 < -1e-9 {
                return vec![];
            }
            let h = h2.max(0.0).sqrt();
            let u = (dx.0 / dd, dx.1 / dd);
            let m = (c1.0 + x * u.0, c1.1 + x * u.1);
            vec![(m.0 - h * u.1, m.1 + h * u.0), (m.0 + h * u.1, m.1 - h * u.0)]
        }
    }
}

/// The profile from the side faces by their geometry: faces grouped by the line / circle they trace in the sketch
/// plane; every group has two ends (the extreme vertices along its trace; a cylinder without edges along the
/// direction is a full circle); ends are paired across groups at the intersection of their traces nearest to both
/// (the corner itself, or the sharp corner a later fillet, chamfer or slot removed), cheapest pairs first.
pub fn side_chain(cx: &Cx, sides: &[FaceId], fr: &Fr, span: (f64, f64), cut: bool) -> Result<Vec<Loop2>, String> {
    let m = cx.m;
    let z = fr.z;
    let trace_of = |g: FaceId| -> Option<Trace> {
        match m.kind(g) {
            Kind::Plane { n, .. } => {
                let n2 = (n.dot(fr.x), n.dot(fr.y));
                let l = n2.0.hypot(n2.1);
                if l < 1e-9 {
                    return None;
                }
                let p = fr.xy(m.b.vertex(m.b.edge(m.b.face_edges(g).first().copied()?).v0).p);
                Some(Trace::Line { p, d: (-n2.1 / l, n2.0 / l) })
            }
            Kind::Cyl { o, a, r, .. } if a.dot(z).abs() > 1.0 - 1e-6 => Some(Trace::Circle { c: fr.xy(*o), r: *r }),
            _ => None,
        }
    };
    let vertical = |e: EdgeId| matches!(m.curve(e), Curve::Line { d, .. } if d.dot(z).abs() > (1.0 - 1e-6) * d.len());
    // the sample points and the positions of the edges along the direction of every side face
    let samples = |g: FaceId| -> (Vec<P2>, Vec<P2>) {
        let (mut pts, mut vpos): (Vec<P2>, Vec<P2>) = (vec![], vec![]);
        for e in m.edges(g) {
            pts.push(fr.xy(m.p0(e)));
            pts.push(fr.xy(m.p1(e)));
            pts.push(fr.xy(m.mid(e)));
            if vertical(e) {
                let q = fr.xy(m.p0(e));
                if !vpos.iter().any(|&v| near2(v, q)) {
                    vpos.push(q);
                }
            }
        }
        (pts, vpos)
    };
    // one group per face, except that faces covering the same stretch of one trace (a side a later feature split
    // along the direction) are one group; pieces of one profile entity split across it are joined by continuation
    // pairs and merged afterwards
    let mut groups: Vec<(Trace, Vec<FaceId>)> = vec![];
    let mut gsamp: Vec<(Vec<P2>, Vec<P2>)> = vec![];
    for &g in sides {
        let t = trace_of(g).ok_or_else(|| format!("side {} has no trace", m.b.face_id_str(g)))?;
        let (pts, vpos) = samples(g);
        let overlaps = |t2: &Trace, s2: &(Vec<P2>, Vec<P2>)| -> bool {
            if !same_trace(&t, t2) {
                return false;
            }
            match t {
                Trace::Line { p, d } => {
                    let tt = |q: &P2| (q.0 - p.0) * d.0 + (q.1 - p.1) * d.1;
                    let span = |ps: &[P2]| ps.iter().fold((f64::INFINITY, f64::NEG_INFINITY), |a, q| (a.0.min(tt(q)), a.1.max(tt(q))));
                    let (a, b) = (span(&pts), span(&s2.0));
                    a.1.min(b.1) - a.0.max(b.0) > 1e-6
                }
                Trace::Circle { .. } => !vpos.is_empty() && vpos.len() == s2.1.len() && vpos.iter().all(|&q| s2.1.iter().any(|&v| near2(v, q))),
            }
        };
        match (0..groups.len()).find(|&k| overlaps(&groups[k].0, &gsamp[k])) {
            Some(k) => {
                groups[k].1.push(g);
                gsamp[k].0.extend(pts);
                for q in vpos {
                    if !gsamp[k].1.iter().any(|&v| near2(v, q)) {
                        gsamp[k].1.push(q);
                    }
                }
            }
            None => {
                groups.push((t, vec![g]));
                gsamp.push((pts, vpos));
            }
        }
    }
    // ends: (group, point); `dirs`: the way an end of a line runs on along its trace (out of its side); a group's
    // length for the pairing tolerance
    let mut ends: Vec<(usize, P2)> = vec![];
    let mut dirs: Vec<Option<P2>> = vec![];
    let mut length: Vec<f64> = vec![];
    for (gi, (t, _)) in groups.iter().enumerate() {
        let (pts, vpos) = &gsamp[gi];
        match t {
            Trace::Line { p, d } => {
                let tt = |q: &P2| (q.0 - p.0) * d.0 + (q.1 - p.1) * d.1;
                let (lo, hi) = pts.iter().fold((f64::INFINITY, f64::NEG_INFINITY), |a, q| (a.0.min(tt(q)), a.1.max(tt(q))));
                ends.push((gi, (p.0 + lo * d.0, p.1 + lo * d.1)));
                dirs.push(Some((-d.0, -d.1)));
                ends.push((gi, (p.0 + hi * d.0, p.1 + hi * d.1)));
                dirs.push(Some(*d));
                length.push(hi - lo);
            }
            Trace::Circle { c, r } => {
                length.push(TAU * r);
                if vpos.is_empty() {
                    continue; // a full circle
                }
                // the ends bound the largest angular gap free of sample points (a point a hair outside an end, where a
                // later blend trims the side across the direction, does not count)
                let mut va: Vec<(f64, P2)> = vpos.iter().map(|&q| (wrap(ang(*c, q)), q)).collect();
                va.sort_by(|a, b| a.0.total_cmp(&b.0));
                let sa: Vec<f64> = pts.iter().map(|q| wrap(ang(*c, *q))).collect();
                let mut best: Option<(f64, usize)> = None;
                for i in 0..va.len() {
                    let (a0, a1) = (va[i].0, va[(i + 1) % va.len()].0);
                    let gap = if va.len() == 1 { TAU } else { wrap(a1 - a0) };
                    let eps = (0.01 * gap).max(1e-6);
                    let inside = sa
                        .iter()
                        .filter(|&&t| {
                            let d = wrap(t - a0);
                            d > eps && d < gap - eps
                        })
                        .count();
                    if inside == 0 && best.map_or(true, |b| gap > b.0) {
                        best = Some((gap, i));
                    }
                }
                let Some((_, i)) = best else { return Err("arc side without a free gap".into()) };
                ends.push((gi, va[i].1));
                dirs.push(None);
                ends.push((gi, va[(i + 1) % va.len()].1));
                dirs.push(None);
            }
        }
    }
    // pair ends: both near the traces' intersection, cheapest first
    let mut cands: Vec<(f64, usize, usize, P2)> = vec![];
    for i in 0..ends.len() {
        for j in i + 1..ends.len() {
            let (gi, gj) = (ends[i].0, ends[j].0);
            if gi == gj {
                continue;
            }
            let (ei, ej) = (ends[i].1, ends[j].1);
            if same_trace(&groups[gi].0, &groups[gj].0) {
                // a continuation across a later cut, or a shared edge
                let cost = (ei.0 - ej.0).hypot(ei.1 - ej.1);
                if cost <= length[gi].max(length[gj]) {
                    cands.push((cost, i, j, ((ei.0 + ej.0) / 2.0, (ei.1 + ej.1) / 2.0)));
                }
                continue;
            }
            for p in meet(&groups[gi].0, &groups[gj].0) {
                let cost = (ei.0 - p.0).hypot(ei.1 - p.1) + (ej.0 - p.0).hypot(ej.1 - p.1);
                // a corner removed by a later fillet or chamfer may be far from both remaining ends
                if cost <= 2.0 * (length[gi] + length[gj]) {
                    cands.push((cost, i, j, p));
                }
            }
        }
    }
    cands.sort_by(|a, b| a.0.total_cmp(&b.0));
    let mut taken = vec![false; ends.len()];
    let mut links: Vec<(usize, usize, P2)> = vec![]; // (group, group, corner)
    for (_, i, j, p) in cands {
        if taken[i] || taken[j] {
            continue;
        }
        // two sides meet once at a corner: both ends of one side paired with both ends of another at the same point
        // (a side much longer than its stretch of this profile, a face it shares with other features) is no outline
        let (gi, gj) = (ends[i].0, ends[j].0);
        if links.iter().any(|l| ((l.0 == gi && l.1 == gj) || (l.0 == gj && l.1 == gi)) && near2(l.2, p)) {
            continue;
        }
        taken[i] = true;
        taken[j] = true;
        links.push((ends[i].0, ends[j].0, p));
    }
    // a free end of a line that a later fillet strip trimmed (the strip, a cylinder along the direction, carries no
    // name yet) ran on to the sharp corner with the strip's other support: the profile reaches that corner
    for k in 0..ends.len() {
        if taken[k] {
            continue;
        }
        let (gi, end) = ends[k];
        let (Trace::Line { .. }, Some(dir)) = (&groups[gi].0, dirs[k]) else { continue };
        for &g in &groups[gi].1 {
            for e in m.edges(g) {
                if !vertical(e) || !near2(fr.xy(m.p0(e)), end) {
                    continue;
                }
                let Some(h) = m.across(e, g) else { continue };
                if cx.lookup(h).is_some() || !matches!(m.kind(h), Kind::Cyl { a, .. } if a.dot(z).abs() > 1.0 - 1e-6) {
                    continue;
                }
                // the strip's other long edge, and the support beyond it
                let other = m.edges(h).into_iter().find(|&e2| e2 != e && vertical(e2) && !near2(fr.xy(m.p0(e2)), end));
                let Some(g2) = other.and_then(|e2| m.across(e2, h)) else { continue };
                let Some(t2 @ Trace::Line { .. }) = trace_of(g2) else { continue };
                if let Some(&c) = meet(&groups[gi].0, &t2).first() {
                    if (c.0 - end.0) * dir.0 + (c.1 - end.1) * dir.1 > 1e-6 {
                        ends[k].1 = c;
                    }
                }
            }
        }
    }
    // ends nothing meets: the stretch of the profile between them left no side on the final body
    if (0..ends.len()).filter(|&k| !taken[k]).count() >= 2 {
        close_free(cx, fr, span, cut, &mut groups, &mut length, &ends, &dirs, &mut taken, &mut links);
    }
    // exactly two ends left: the side between them was absorbed by a flush face of another feature; close with a line
    let free: Vec<usize> = (0..ends.len()).filter(|&k| !taken[k]).collect();
    if free.len() == 2 && ends[free[0]].0 != ends[free[1]].0 {
        let (i, j) = (free[0], free[1]);
        let (mut ei, mut ej) = (ends[i].1, ends[j].1);
        if !cut {
            if let (Some(di), Some(dj)) = (dirs[i], dirs[j]) {
                if let Some((pi, pj)) = run_on_to_floor(cx, fr, span, (ei, di), (ej, dj)) {
                    ei = pi;
                    ej = pj;
                }
            }
        }
        let d = (ej.0 - ei.0, ej.1 - ei.1);
        let l = d.0.hypot(d.1);
        if l > 1e-6 {
            groups.push((Trace::Line { p: ei, d: (d.0 / l, d.1 / l) }, vec![]));
            let gi = groups.len() - 1;
            length.push(l);
            taken[i] = true;
            taken[j] = true;
            links.push((ends[i].0, gi, ei));
            links.push((gi, ends[j].0, ej));
        }
    }
    if let Some(k) = (0..ends.len()).find(|&k| !taken[k]) {
        let desc = |gi: usize| match groups[gi].0 {
            Trace::Line { p, d } => format!("line through ({:.3},{:.3}) along ({:.3},{:.3}) len {:.3}", p.0, p.1, d.0, d.1, length[gi]),
            Trace::Circle { c, r } => format!("circle c=({:.3},{:.3}) r={:.3}", c.0, c.1, r),
        };
        let others: Vec<String> = ends.iter().enumerate().filter(|(i, e)| *i != k && e.0 != ends[k].0).map(|(_, e)| format!("[{} end ({:.3},{:.3}) {}]", groups[e.0].1.first().map_or("?".into(), |&g| m.b.face_id_str(g)), e.1 .0, e.1 .1, if taken[ends.iter().position(|x| x.0 == e.0 && near2(x.1, e.1)).unwrap()] { "taken" } else { "free" })).collect();
        return Err(format!("side {} ({}) has an end ({:.3},{:.3}) nothing meets; other ends: {}", groups[ends[k].0].1.first().map_or("?".into(), |&g| m.b.face_id_str(g)), desc(ends[k].0), ends[k].1 .0, ends[k].1 .1, others.join(" ")));
    }
    // cycles
    let mut used = vec![false; groups.len()];
    let mut loops = vec![];
    for start in 0..groups.len() {
        if used[start] {
            continue;
        }
        let d0 = links.iter().filter(|l| l.0 == start || l.1 == start).count();
        if d0 == 0 {
            let Trace::Circle { c, r } = groups[start].0 else { return Err(format!("side {} is linked to nothing", groups[start].1.first().map_or("?".into(), |&g| m.b.face_id_str(g)))) };
            used[start] = true;
            loops.push(Loop2 { segs: vec![(Seg2::Circle { c, r }, groups[start].1.clone())] });
            continue;
        }
        if d0 != 2 {
            return Err(format!("side {} links to {d0} others", groups[start].1.first().map_or("?".into(), |&g| m.b.face_id_str(g))));
        }
        let mut order: Vec<(usize, P2)> = vec![]; // (group, corner at its end)
        let mut prev_link: Option<usize> = None;
        let mut cur = start;
        loop {
            used[cur] = true;
            let Some(li) = (0..links.len()).find(|&li| Some(li) != prev_link && (links[li].0 == cur || links[li].1 == cur)) else { return Err("side chain broken".into()) };
            let l = links[li];
            let next = if l.0 == cur { l.1 } else { l.0 };
            order.push((cur, l.2));
            prev_link = Some(li);
            cur = next;
            if cur == start {
                break;
            }
            if used[cur] || order.len() > groups.len() {
                return Err("side chain does not close".into());
            }
        }
        let n = order.len();
        let mut segs = vec![];
        for k in 0..n {
            let (gi, b) = order[k];
            let a = order[(k + n - 1) % n].1;
            let seg = match groups[gi].0 {
                Trace::Line { .. } => Seg2::Line { a, b },
                Trace::Circle { c, .. } => {
                    if near2(a, b) {
                        return Err("arc side with coincident corners".into());
                    }
                    let (ta, tb) = (ang(c, a), ang(c, b));
                    let mut ccw_votes = 0i32;
                    for &g in &groups[gi].1 {
                        for e in m.edges(g) {
                            for p in [m.p0(e), m.mid(e)] {
                                let t = ang(c, fr.xy(p));
                                let sa = wrap(t - ta);
                                let sb = wrap(tb - ta);
                                // points at the corners (a hair to either side of them numerically) do not vote
                                if sa > 1e-6 && sa < TAU - 1e-6 && (sb - sa).abs() > 1e-6 {
                                    ccw_votes += if sa < sb { 1 } else { -1 };
                                }
                            }
                        }
                    }
                    Seg2::Arc { a, b, c, ccw: ccw_votes >= 0 }
                }
            };
            segs.push((seg, groups[gi].1.clone()));
        }
        merge_segs(&mut segs);
        loops.push(Loop2 { segs });
    }
    // a side whose two corners coincide (both its ends met other sides at one point) is no piece of the outline
    for l in loops.iter_mut() {
        l.segs.retain(|(s, _)| !matches!(s, Seg2::Line { a, b } if dist2(*a, *b) < 1e-6));
        merge_segs(&mut l.segs);
    }
    loops.retain(|l| !l.segs.is_empty());
    if loops.is_empty() {
        let desc: Vec<String> = groups
            .iter()
            .map(|(t, fs)| match t {
                Trace::Line { p, d } => format!("{}: line ({:.3},{:.3}) d ({:.3},{:.3})", fs.first().map_or("closure".into(), |&g| m.b.face_id_str(g)), p.0, p.1, d.0, d.1),
                Trace::Circle { c, r } => format!("{}: circle ({:.3},{:.3}) r {:.3}", fs.first().map_or("closure".into(), |&g| m.b.face_id_str(g)), c.0, c.1, r),
            })
            .collect();
        let ls: Vec<String> = links.iter().map(|l| format!("{}-{} at ({:.3},{:.3})", l.0, l.1, l.2 .0, l.2 .1)).collect();
        return Err(format!("the side traces close no loop with any extent (groups: {}; links: {})", desc.join(" | "), ls.join(" ")));
    }
    if let Some(why) = not_simple(&loops) {
        let desc: Vec<String> = loops
            .iter()
            .flat_map(|l| l.segs.iter())
            .take(40)
            .map(|(s, _)| match s {
                Seg2::Line { a, b } => format!("L({:.3},{:.3})-({:.3},{:.3})", a.0, a.1, b.0, b.1),
                Seg2::Arc { a, b, c, ccw } => format!("A({:.3},{:.3})-({:.3},{:.3}) c({:.3},{:.3}){}", a.0, a.1, b.0, b.1, c.0, c.1, if *ccw { "+" } else { "-" }),
                Seg2::Circle { c, r } => format!("C({:.3},{:.3}) r{:.3}", c.0, c.1, r),
            })
            .collect();
        return Err(format!("the side traces give no simple outline ({why}: {})", desc.join(" ")));
    }
    Ok(loops)
}

/// The body the features rebuilt so far make (the material before this feature).
fn prior_body(cx: &Cx) -> Option<ck_topo::body::Body> {
    ck_recog::rebuild::rebuild(&cx.tree.feats).ok().and_then(|(b, _)| b)
}

fn lerp2(a: P2, b: P2, t: f64) -> P2 {
    (a.0 + (b.0 - a.0) * t, a.1 + (b.1 - a.1) * t)
}

fn dist2(a: P2, b: P2) -> f64 {
    (a.0 - b.0).hypot(a.1 - b.1)
}

/// The closing chord of a boss, whose stretch left no side on the final body, may lie short of where the profile
/// really ended: the two ends of the sides run on (along their traces) to the nearest plane of the final body that
/// faces back at them (the floor of a later feature the boss sits on), when the strip between the old chord and that
/// plane lies in the final body throughout. The union is the same either way, but the faces of the boss's edges the
/// later features then name (a chamfer of the edge down to that floor) exist in the rebuilt body.
fn run_on_to_floor(cx: &Cx, fr: &Fr, span: (f64, f64), (ei, di): (P2, P2), (ej, dj): (P2, P2)) -> Option<(P2, P2)> {
    use ck_ops::contain::{Containment, Loc};
    let m = cx.m;
    let mut cands: Vec<(f64, P2, P2)> = vec![];
    for &h in &m.faces {
        let Kind::Plane { n, d } = m.kind(h) else { continue };
        let n2 = (n.dot(fr.x), n.dot(fr.y));
        if (n2.0.hypot(n2.1) - 1.0).abs() > 1e-6 {
            continue;
        }
        let d2 = d - n.dot(fr.o);
        let hit = |e: P2, dir: P2| -> Option<P2> {
            let c = n2.0 * dir.0 + n2.1 * dir.1;
            let s = (d2 - (n2.0 * e.0 + n2.1 * e.1)) / c;
            (c < -1e-6 && s > 1e-6).then(|| (e.0 + dir.0 * s, e.1 + dir.1 * s))
        };
        let (Some(pi), Some(pj)) = (hit(ei, di), hit(ej, dj)) else { continue };
        let far = dist2(ei, pi).max(dist2(ej, pj));
        if !cands.iter().any(|c| near2(c.1, pi) && near2(c.2, pj)) {
            cands.push((far, pi, pj));
        }
    }
    cands.sort_by(|a, b| a.0.total_cmp(&b.0));
    let fc = Containment::new(m.b, TOL);
    let levels: Vec<f64> = [0.02, 0.5, 0.98].iter().map(|f| span.0 + f * (span.1 - span.0)).collect();
    let at = |p: P2, t: f64| fr.o + fr.x * p.0 + fr.y * p.1 + fr.z * t;
    for (_, pi, pj) in cands {
        let mut ok = true;
        'strip: for u in [0.25, 0.5, 0.75] {
            for v in [0.25, 0.5, 0.75] {
                let q = lerp2(lerp2(ei, ej, u), lerp2(pi, pj, u), v);
                for &t in &levels {
                    if fc.classify_point(at(q, t), TOL) == Loc::Out {
                        ok = false;
                        break 'strip;
                    }
                }
            }
        }
        if ok {
            return Some((pi, pj));
        }
    }
    None
}

/// Close the ends of the side traces that nothing meets (the stretch of the profile between them left no side on
/// the final body): first with chords the material says are safe, then by continuing one trace across a gap.
///
/// A boss's chord is safe when it runs through material the earlier features made (any closing there gives the
/// same union) or nowhere through the final body (a later cut removed that stretch, so the boss's shape there does
/// not matter); a cut's chord when it runs nowhere through the earlier material (the cut breaks out of the body
/// there). The ends of line sides may first run on along their traces (a boss into earlier material, a cut out of
/// it; the strip that adds to a cut must stay out of the earlier material too).
/// Cheapest pairs first; an end no safe chord reaches is left for the caller. `span`: the extrude's extent along
/// the sketch normal, from the sketch plane.
#[allow(clippy::too_many_arguments)]
fn close_free(
    cx: &Cx,
    fr: &Fr,
    span: (f64, f64),
    cut: bool,
    groups: &mut Vec<(Trace, Vec<FaceId>)>,
    length: &mut Vec<f64>,
    ends: &[(usize, P2)],
    dirs: &[Option<P2>],
    taken: &mut [bool],
    links: &mut Vec<(usize, usize, P2)>,
) {
    use ck_ops::contain::{Containment, Loc};
    let prior = prior_body(cx);
    let pc = prior.as_ref().map(|b| Containment::new(b, TOL));
    let fc = Containment::new(cx.m.b, TOL);
    let in_prior = |q: Vec3| pc.as_ref().map_or(Loc::Out, |c| c.classify_point(q, TOL));
    let in_final = |q: Vec3| fc.classify_point(q, TOL);
    let levels: Vec<f64> = [0.15, 0.5, 0.85].iter().map(|f| span.0 + f * (span.1 - span.0)).collect();
    let at = |p: P2, t: f64| fr.o + fr.x * p.0 + fr.y * p.1 + fr.z * t;
    let chord_pts = |a: P2, b: P2| -> Vec<Vec3> { (1..6).flat_map(|k| levels.iter().map(move |&t| (k, t))).map(|(k, t)| at(lerp2(a, b, k as f64 / 6.0), t)).collect() };
    let strip_pts = |a: P2, b: P2, a2: P2, b2: P2| -> Vec<Vec3> {
        let mut out = vec![];
        for u in [0.25, 0.5, 0.75] {
            for v in [0.25, 0.5, 0.75] {
                let q = lerp2(lerp2(a, b, u), lerp2(a2, b2, u), v);
                out.extend(levels.iter().map(|&t| at(q, t)));
            }
        }
        out
    };
    // the open chains the links make: the two free ends of one chain close it into a loop (tried first), ends of
    // two chains join them
    let mut chain: Vec<usize> = (0..groups.len()).collect();
    fn root(c: &mut [usize], i: usize) -> usize {
        let mut r = i;
        while c[r] != r {
            r = c[r];
        }
        c[i] = r;
        r
    }
    for l in links.iter() {
        let (a, b) = (root(&mut chain, l.0), root(&mut chain, l.1));
        chain[a] = b;
    }
    let free: Vec<usize> = (0..ends.len()).filter(|&k| !taken[k]).collect();
    let mut pairs: Vec<(bool, f64, usize, usize)> = vec![];
    for (x, &i) in free.iter().enumerate() {
        for &j in &free[x + 1..] {
            if ends[i].0 != ends[j].0 {
                let other_chain = root(&mut chain, ends[i].0) != root(&mut chain, ends[j].0);
                pairs.push((other_chain, dist2(ends[i].1, ends[j].1), i, j));
            }
        }
    }
    pairs.sort_by(|a, b| a.0.cmp(&b.0).then(a.1.total_cmp(&b.1)));
    let steps: &[f64] = if cut { &[0.05, 0.25, 0.0] } else { &[0.0, 0.05, 0.25, 0.5, 1.0, 2.0] };
    for &(_, _, i, j) in &pairs {
        if taken[i] || taken[j] {
            continue;
        }
        let (ei, ej) = (ends[i].1, ends[j].1);
        let lc = [ends[i].0, ends[j].0].iter().filter(|&&g| matches!(groups[g].0, Trace::Line { .. })).map(|&g| length[g]).fold(0.0, f64::max).max(dist2(ei, ej));
        let mut found = None;
        for &s in steps {
            if s > 0.0 && dirs[i].is_none() && dirs[j].is_none() {
                continue;
            }
            let ext = |e: P2, d: Option<P2>| d.map_or(e, |d| (e.0 + d.0 * s * lc, e.1 + d.1 * s * lc));
            let (pi, pj) = (ext(ei, dirs[i]), ext(ej, dirs[j]));
            let chord = chord_pts(pi, pj);
            let strip = if s > 0.0 { strip_pts(ei, ej, pi, pj) } else { vec![] };
            let ok = if cut {
                chord.iter().chain(&strip).all(|&q| in_prior(q) != Loc::In)
            } else {
                chord.iter().all(|&q| in_prior(q) != Loc::Out) || (s == 0.0 && chord.iter().all(|&q| in_final(q) != Loc::In))
            };
            if ok {
                found = Some((pi, pj));
                break;
            }
        }
        let Some((pi, pj)) = found else { continue };
        taken[i] = true;
        taken[j] = true;
        let l = dist2(pi, pj);
        if l < 1e-6 {
            links.push((ends[i].0, ends[j].0, pi));
        } else {
            groups.push((Trace::Line { p: pi, d: ((pj.0 - pi.0) / l, (pj.1 - pi.1) / l) }, vec![]));
            length.push(l);
            let g = groups.len() - 1;
            links.push((ends[i].0, g, pi));
            links.push((g, ends[j].0, pj));
        }
    }
    // a side whose middle stretch left no face (interrupted by a later feature, or flush with an earlier one)
    // continues along its own trace
    for &(_, _, i, j) in &pairs {
        if taken[i] || taken[j] || !same_trace(&groups[ends[i].0].0, &groups[ends[j].0].0) {
            continue;
        }
        taken[i] = true;
        taken[j] = true;
        links.push((ends[i].0, ends[j].0, lerp2(ends[i].1, ends[j].1, 0.5)));
    }
}

fn orient2(p: P2, q: P2) -> f64 {
    p.0 * q.1 - q.0 * p.1
}

/// Why the loops are no simple outline (a polygon through the line ends and arc samples crosses itself, or a side
/// runs back over its neighbour), if they are not.
fn not_simple(loops: &[Loop2]) -> Option<String> {
    let mut edges: Vec<(usize, usize, usize, P2, P2)> = vec![]; // (loop, index, count in loop, a, b)
    for (li, l) in loops.iter().enumerate() {
        if l.segs.is_empty() {
            return Some("a loop without sides".into());
        }
        let mut poly: Vec<P2> = vec![];
        for (s, _) in &l.segs {
            match s {
                Seg2::Line { a, .. } => poly.push(*a),
                Seg2::Arc { a, b, c, ccw } => {
                    let r = dist2(*a, *c);
                    let t0 = ang(*c, *a);
                    let mut sweep = if *ccw { wrap(ang(*c, *b) - t0) } else { -wrap(t0 - ang(*c, *b)) };
                    if sweep.abs() < 1e-9 {
                        sweep = if *ccw { TAU } else { -TAU };
                    }
                    for k in 0..8 {
                        let t = t0 + sweep * k as f64 / 8.0;
                        poly.push((c.0 + r * t.cos(), c.1 + r * t.sin()));
                    }
                }
                Seg2::Circle { .. } => {}
            }
        }
        let n = poly.len();
        if n < 2 {
            continue;
        }
        let area: f64 = (0..n).map(|k| orient2(poly[k], poly[(k + 1) % n])).sum::<f64>() / 2.0;
        let size = poly.iter().fold(0.0f64, |a, p| a.max(dist2(*p, poly[0])));
        if area.abs() < 1e-9 * (1.0 + size * size) {
            return Some("a loop without area".into());
        }
        for k in 0..n {
            edges.push((li, k, n, poly[k], poly[(k + 1) % n]));
        }
    }
    let orient = |a: P2, b: P2, c: P2| (b.0 - a.0) * (c.1 - a.1) - (b.1 - a.1) * (c.0 - a.0);
    for x in 0..edges.len() {
        for y in x + 1..edges.len() {
            let (l1, k1, n1, p1, p2) = edges[x];
            let (l2, k2, _, q1, q2) = edges[y];
            let (lp, lq) = (dist2(p1, p2), dist2(q1, q2));
            if lp < 1e-9 || lq < 1e-9 {
                continue;
            }
            let adjacent = l1 == l2 && (k2 == (k1 + 1) % n1 || k1 == (k2 + 1) % n1);
            if adjacent {
                let u = ((p2.0 - p1.0) / lp, (p2.1 - p1.1) / lp);
                let v = ((q2.0 - q1.0) / lq, (q2.1 - q1.1) / lq);
                if (u.0 * v.1 - u.1 * v.0).abs() < 1e-7 && u.0 * v.0 + u.1 * v.1 < 0.0 {
                    return Some(format!("a side runs back over its neighbour at ({:.3},{:.3})", p2.0, p2.1));
                }
                continue;
            }
            let eps = 1e-9 * lp * lq;
            let (d1, d2) = (orient(q1, q2, p1), orient(q1, q2, p2));
            let (d3, d4) = (orient(p1, p2, q1), orient(p1, p2, q2));
            if d1.abs() <= eps * 1e3 && d2.abs() <= eps * 1e3 {
                // collinear: the two stretches must not overlap
                let u = ((p2.0 - p1.0) / lp, (p2.1 - p1.1) / lp);
                let t = |q: P2| (q.0 - p1.0) * u.0 + (q.1 - p1.1) * u.1;
                let (a, b) = (t(q1).min(t(q2)), t(q1).max(t(q2)));
                if b.min(lp) - a.max(0.0) > 1e-6 {
                    return Some(format!("two sides overlap near ({:.3},{:.3})", p1.0, p1.1));
                }
                continue;
            }
            if ((d1 > eps && d2 < -eps) || (d1 < -eps && d2 > eps)) && ((d3 > eps && d4 < -eps) || (d3 < -eps && d4 > eps)) {
                return Some(format!("the outline crosses itself near ({:.3},{:.3})", p1.0, p1.1));
            }
        }
    }
    None
}

/// Draw the loops into a sketch and name the side faces after the entities.
pub fn emit_loops(cx: &mut Cx, sk: &mut Sk, loops: &[Loop2], eid: &str) {
    emit_loops_ids(cx, sk, loops, eid);
}

/// [`emit_loops`], returning every piece with the id of its sketch entity.
pub fn emit_loops_ids(cx: &mut Cx, sk: &mut Sk, loops: &[Loop2], eid: &str) -> Vec<(Seg2, String)> {
    let mut out = vec![];
    let mut pts: Vec<(P2, String)> = vec![];
    let mut point = |sk: &mut Sk, p: P2| -> String {
        if let Some((_, id)) = pts.iter().find(|(q, _)| near2(*q, p)) {
            return id.clone();
        }
        let id = sk.point(p.0, p.1);
        pts.push((p, id.clone()));
        id
    };
    for l in loops {
        for (seg, faces) in &l.segs {
            let e = match seg {
                Seg2::Line { a, b } => {
                    let (pa, pb) = (point(sk, *a), point(sk, *b));
                    sk.line(&pa, &pb, false)
                }
                Seg2::Arc { a, b, c, ccw } => {
                    let (pa, pb, pc) = (point(sk, *a), point(sk, *b), point(sk, *c));
                    if *ccw {
                        sk.arc(&pc, &pa, &pb)
                    } else {
                        sk.arc(&pc, &pb, &pa)
                    }
                }
                Seg2::Circle { c, r } => {
                    let pc = point(sk, *c);
                    sk.circle(&pc, *r)
                }
            };
            // a face an earlier feature named keeps that name: a side flush with an earlier face is one face of
            // the rebuilt body, which carries the earlier feature's name
            for &f in faces {
                cx.names.entry(f).or_insert_with(|| format!("{eid}.S.{e}"));
            }
            out.push((seg.clone(), e));
        }
    }
    out
}

/// `FACE_ID_2001` kind code of a face a split line cut out of an earlier face (sample census: the split line's own
/// id, no other tag).
const KIND_SPLIT: i64 = 62;

/// Faces no feature claims yet that lie on a side of this extrude: the surface of a piece of the profile, within
/// the piece's stretch and the extrude's extent (a side a split line cut into faces tagged with the split line
/// only). Named after the piece, so a later blend can pick them.
fn name_flush(cx: &mut Cx, pieces: &[(Seg2, String)], fr: &Fr, span: (f64, f64), eid: &str) {
    let m = cx.m;
    let tol = 1e-6;
    let on = |p: P2, seg: &Seg2| -> bool {
        match seg {
            Seg2::Line { a, b } => {
                let l = dist2(*a, *b);
                if l < 1e-9 {
                    return false;
                }
                let u = ((b.0 - a.0) / l, (b.1 - a.1) / l);
                let (dx, dy) = (p.0 - a.0, p.1 - a.1);
                let t = dx * u.0 + dy * u.1;
                (dx * u.1 - dy * u.0).abs() < tol * (1.0 + l) && t > -tol * l - 1e-6 && t < l * (1.0 + tol) + 1e-6
            }
            Seg2::Arc { a, c, .. } => (dist2(p, *c) - dist2(*a, *c)).abs() < tol * (1.0 + dist2(*a, *c)),
            Seg2::Circle { c, r } => (dist2(p, *c) - r).abs() < tol * (1.0 + r),
        }
    };
    let mut found: Vec<(FaceId, String)> = vec![];
    for &g in &m.faces {
        // only faces no geometry feature claims: a later feature's flush wall is named when that feature is read
        if cx.names.contains_key(&g) || !cx.tags.get(g.idx()).map_or(true, |v| v.iter().all(|t| t.1 == KIND_SPLIT)) {
            continue;
        }
        match m.kind(g) {
            Kind::Plane { n, .. } if n.dot(fr.z).abs() < 1e-9 => {}
            Kind::Cyl { a, .. } if a.dot(fr.z).abs() > 1.0 - 1e-9 => {}
            _ => continue,
        }
        let pts = face_points(m, g);
        if pts.iter().any(|&p| {
            let t = (p - fr.o).dot(fr.z);
            t < span.0 - 1e-6 || t > span.1 + 1e-6
        }) {
            continue;
        }
        if let Some((_, e)) = pieces.iter().find(|(seg, _)| pts.iter().all(|&p| on(fr.xy(p), seg))) {
            found.push((g, format!("{eid}.S.{e}")));
        }
    }
    for (g, n) in found {
        cx.names.insert(g, n);
    }
}

/// Read an extrude back from its faces and emit plane, sketch and extrude.
/// Extent of the body along `z`.
fn body_extent(m: &Model, z: Vec3) -> (f64, f64) {
    let (mut lo, mut hi) = (f64::INFINITY, f64::NEG_INFINITY);
    for v in m.b.vertex_ids() {
        let t = m.b.vertex(v).p.dot(z);
        lo = lo.min(t);
        hi = hi.max(t);
    }
    (lo, hi)
}

/// The rim of `sides` at level `z` (along `fz`) borders a face that no earlier feature named: a later feature
/// (a blend) cut the walls short there.
fn mouth_trimmed(cx: &Cx, sides: &[FaceId], fz: Vec3, z: f64, tol: f64) -> bool {
    let m = cx.m;
    let mut any = false;
    for &g in sides {
        for e in m.edges(g) {
            let (p0, p1, mid) = (m.p0(e).dot(fz), m.p1(e).dot(fz), m.mid(e).dot(fz));
            if (p0 - z).abs() > tol || (p1 - z).abs() > tol || (mid - z).abs() > tol {
                continue;
            }
            let Some(h) = m.across(e, g) else { continue };
            if sides.contains(&h) {
                continue;
            }
            if cx.names.contains_key(&h) {
                return false;
            }
            any = true;
        }
    }
    any
}

/// The listed depth of a boss reaches past the end of its sides (`(lo, hi)` to `(nlo, nhi)`): a later blend or cut
/// consumed the stretch, and the boss really ran on, unless the stretch lies nowhere in the final body. The side
/// faces' points at an end, carried through the stretch, are then outside the final body (a boss on a plate, its
/// listed depth counted from a plane below the part: nothing of it was ever there).
fn listed_stretch_has_material(cx: &Cx, sides: &[FaceId], fz: Vec3, (lo, hi): (f64, f64), (nlo, nhi): (f64, f64)) -> bool {
    use ck_ops::contain::{Containment, Loc};
    let fc = Containment::new(cx.m.b, TOL);
    let tol = TOL * (1.0 + lo.abs().max(hi.abs()));
    let mut pts: Vec<(Vec3, f64)> = vec![];
    for &g in sides {
        for p in face_points(cx.m, g) {
            let z = p.dot(fz);
            if (z - lo).abs() < tol && nlo < lo - tol {
                pts.extend([0.25, 0.75].iter().map(|f| (p - fz * (f * (lo - nlo)), *f)));
            }
            if (z - hi).abs() < tol && nhi > hi + tol {
                pts.extend([0.25, 0.75].iter().map(|f| (p + fz * (f * (nhi - hi)), *f)));
            }
        }
    }
    pts.is_empty() || pts.iter().any(|&(q, _)| fc.classify_point(q, TOL) != Loc::Out)
}

pub fn extrude(cx: &mut Cx, f: &SwFeature, faces: &[FaceId], cut: bool) -> Result<String, String> {
    let m = cx.m;
    let extent = |d: Vec3| -> (f64, f64) {
        // along a principal axis the true extent of the faces (a saddle edge reaches further than its vertices and
        // mid points: a small hole breaking into a bore)
        if let Some((i, sg)) = principal(d) {
            if (d.len() - 1.0).abs() < 1e-9 && sg.abs() == 1.0 {
                let (a, b) = m.b.bbox_of_faces(faces);
                let (x, y) = (comp(a, i), comp(b, i));
                return if sg > 0.0 { (x, y) } else { (-y, -x) };
            }
        }
        let (mut lo, mut hi) = (f64::INFINITY, f64::NEG_INFINITY);
        for &g in faces {
            for p in face_points(m, g) {
                lo = lo.min(p.dot(d));
                hi = hi.max(p.dot(d));
            }
        }
        (lo, hi)
    };
    // the direction: the file's cap tags decide (kind 40 = end cap), then the extent that equals the listed depth,
    // then the most caps
    let d1 = crate::dim(cx, f, "D1");
    let mut cands = classify_dir(m, faces)?;
    cands.retain(|c| !c.2.is_empty());
    if cands.is_empty() {
        return Err("no side faces".into());
    }
    let score = |c: &(Vec3, Vec<FaceId>, Vec<FaceId>)| {
        let tagged_caps = c.1.iter().filter(|&&g| cx.is_cap(g, f.id)).count() as i64 - c.2.iter().filter(|&&g| cx.is_cap(g, f.id)).count() as i64;
        let (lo, hi) = extent(c.0);
        let d1_match = d1.map_or(false, |v| (hi - lo - v).abs() <= 1e-3 * (1.0 + v)) as i64;
        // a cap whose rim is not the feature's own sides counts against the direction (a wall of a through slot
        // reads as a cap of the cross direction)
        let quality: i64 = c.1.iter().map(|&g| if rim_in(m, g, &c.2) { 1 } else { -1 }).sum();
        (tagged_caps, d1_match, quality)
    };
    cands.sort_by_key(|c| std::cmp::Reverse(score(c)));
    let (d, caps, mut sides) = cands.remove(0);
    // faces a split line cut out of this extrude's sides carry the split line's tag only: the pieces along the
    // direction that border the feature's faces (or such pieces) are sides too
    loop {
        let more: Vec<FaceId> = m
            .faces
            .iter()
            .copied()
            .filter(|&g| {
                !sides.contains(&g)
                    && cx.tags.get(g.idx()).map_or(false, |v| !v.is_empty() && v.iter().all(|t| t.1 == KIND_SPLIT))
                    && match m.kind(g) {
                        Kind::Plane { n, .. } => n.dot(d).abs() < 1e-9,
                        Kind::Cyl { a, .. } => a.dot(d).abs() > 1.0 - 1e-9,
                        _ => false,
                    }
                    && m.edges(g).iter().any(|&e| m.across(e, g).map_or(false, |h| sides.contains(&h) || caps.contains(&h)))
            })
            .collect();
        if more.is_empty() {
            break;
        }
        sides.extend(more);
    }
    let fz = match principal(d) {
        Some((i, _)) => PlaneSpec::for_axis(i).zs,
        None => d,
    };
    let (mut lo, mut hi) = extent(fz);
    // an end that lies beyond every vertex and edge mid point (a small hole breaking into a bore: the saddle edge
    // reaches further than the points on it) is a cut ending in a void: it runs a little past the extreme, so that
    // the tool does not touch the far surface tangentially
    {
        let (mut slo, mut shi) = (f64::INFINITY, f64::NEG_INFINITY);
        for &g in faces {
            for p in face_points(m, g) {
                slo = slo.min(p.dot(fz));
                shi = shi.max(p.dot(fz));
            }
        }
        let (t, pad) = (TOL * (1.0 + lo.abs().max(hi.abs())), 0.02 * (hi - lo));
        if cut && principal(fz).is_some() {
            if hi > shi + t {
                hi += pad;
            }
            if lo < slo - t {
                lo -= pad;
            }
        }
    }
    let level_of = |g: FaceId| -> f64 {
        match m.kind(g) {
            Kind::Plane { n, d } => d * n.dot(fz),
            _ => f64::NAN,
        }
    };
    // caps at two levels bound the extrude; side faces running beyond them were extended by later features
    let mut levels: Vec<f64> = caps.iter().map(|&g| level_of(g)).collect();
    levels.sort_by(|a, b| a.total_cmp(b));
    levels.dedup_by(|a, b| (*a - *b).abs() < TOL * (1.0 + a.abs()));
    let mut extended = false;
    if levels.len() == 2 {
        extended = levels[0] > lo + TOL || levels[1] < hi - TOL;
        lo = levels[0];
        hi = levels[1];
    }
    // one cap between the ends of the faces (the sides run on into the faces of other features they share): the
    // listed depth from the cap, towards the boss's material (away from its outward normal) or into the cut (along
    // the floor's outward normal)
    // the same for a boss whose sides end short of the listed depth (a later blend or cut consumed the stretch at
    // the far end), when the listed depth from the cap covers all its faces
    let mut from_d1 = false;
    if let (1, Some(v)) = (levels.len(), d1) {
        let lv = levels[0];
        let t = TOL * (1.0 + lv.abs());
        let up = matches!(m.kind(caps[0]), Kind::Plane { n, .. } if n.dot(fz) > 0.0);
        let (nlo, nhi) = if up != cut { (lv - v, lv) } else { (lv, lv + v) };
        let short = !cut && v > hi - lo + t && nlo <= lo + t && nhi >= hi - t && listed_stretch_has_material(cx, &sides, fz, (lo, hi), (nlo, nhi));
        if v > 1e-6 && ((lv > lo + t && lv < hi - t) || short) {
            let up = matches!(m.kind(caps[0]), Kind::Plane { n, .. } if n.dot(fz) > 0.0);
            if up != cut {
                lo = lv - v;
                hi = lv;
            } else {
                lo = lv;
                hi = lv + v;
            }
            from_d1 = true;
        }
    }
    let depth = hi - lo;
    if !(depth > 1e-6) {
        return Err("no extent along the direction".into());
    }
    let tol = TOL * (1.0 + lo.abs().max(hi.abs()));
    let (mut cap_lo, mut cap_hi) = (vec![], vec![]);
    for &g in &caps {
        let z = level_of(g);
        if (z - lo).abs() < tol {
            cap_lo.push(g);
        } else if (z - hi).abs() < tol {
            cap_hi.push(g);
        } else {
            return Err("cap at an intermediate level".into());
        }
    }
    // the sketch sits at the bottom of a boss, at the mouth of a cut; `down`: the extrude runs along -z
    let (level, down, c0, c1): (f64, bool, Vec<FaceId>, Vec<FaceId>) = if !cut {
        (lo, false, cap_lo.clone(), cap_hi.clone())
    } else if !cap_lo.is_empty() {
        (hi, true, vec![], cap_lo.clone())
    } else if !cap_hi.is_empty() {
        (lo, false, vec![], cap_hi.clone())
    } else {
        // a through cut whose walls end short of the body's extent: a later blend or chamfer rounded the mouth off
        // (the rim borders a face no earlier feature named), so the sketch sits at the body's face, not at the wall's end
        let (bmin, bmax) = body_extent(m, fz);
        let btol = TOL * (1.0 + bmin.abs().max(bmax.abs()));
        if (lo - bmin).abs() < btol && bmax > hi + btol && mouth_trimmed(cx, &sides, fz, hi, btol) {
            (bmax, true, vec![], vec![])
        } else if (hi - bmax).abs() < btol && bmin < lo - btol && mouth_trimmed(cx, &sides, fz, lo, btol) {
            (bmin, false, vec![], vec![])
        } else {
            (hi, true, vec![], vec![])
        }
    };
    // a cut runs against the sketch normal unless flipped (ck-model extrude)
    let flip = down != cut;
    let fr = cx.frame(fz, fz * level);
    let mut why: Vec<String> = vec![];
    let mut loops: Option<(Vec<Loop2>, &str)> = None;
    // the feature's side faces; when they give no profile, the ones it made itself (a face it shares with an
    // earlier feature, a side it split, may not be one of its walls)
    let own: Vec<FaceId> = sides.iter().copied().filter(|&g| cx.tag_of(g) == Some(f.id)).collect();
    let mut attempts = vec![sides.clone()];
    if !own.is_empty() && own.len() < sides.len() {
        attempts.push(own);
    }
    let span = (lo - level, hi - level);
    for (k, ss) in attempts.iter().enumerate() {
        if loops.is_some() {
            break;
        }
        let tag = if k == 0 { "" } else { "own sides: " };
        let side_set: HashSet<FaceId> = ss.iter().copied().collect();
        // the caps of one level: one per region of the profile (a cut of two holes has two floors), each giving
        // its own loops; when one of them fails, the first that does not (as for a single region)
        let mut first: Option<Vec<Loop2>> = None;
        // a level whose caps are the feature's own first: a cap flush with an earlier feature's face is that face's
        // outline (the whole body's end, a hub the ring sits on), not this profile's
        let flush = |cs: &Vec<FaceId>| cs.iter().any(|&g| cx.tags.get(g.idx()).map_or(false, |v| v.iter().any(|t| t.0 < f.id)));
        let mut order = [&cap_lo, &cap_hi];
        if flush(order[0]) && !order[1].is_empty() && !flush(order[1]) {
            order.swap(0, 1);
        }
        for level_caps in order {
            let mut acc: Vec<Loop2> = vec![];
            let mut all = !level_caps.is_empty();
            for &g in level_caps.iter() {
                match cap_outline(m, g, &side_set, &fr) {
                    Ok(l) => {
                        if first.is_none() {
                            first = Some(l.clone());
                        }
                        acc.extend(l);
                    }
                    Err(e) => {
                        all = false;
                        why.push(format!("{tag}cap: {e}"));
                    }
                }
            }
            if all {
                loops = Some((acc, "cap outline"));
                break;
            }
        }
        if loops.is_none() {
            if let Some(l) = first {
                loops = Some((l, "cap outline"));
            }
        }
        if loops.is_none() {
            match side_cycles(m, &side_set, &fr) {
                Ok(l) => loops = Some((l, "side cycles")),
                Err(e) => why.push(format!("{tag}sides: {e}")),
            }
        }
        if loops.is_none() {
            match side_chain(cx, ss, &fr, span, cut) {
                Ok(l) => loops = Some((l, if k == 0 { "side traces" } else { "side traces of its own sides" })),
                Err(e) => why.push(format!("{tag}traces: {e}")),
            }
        }
    }
    // last resort: the cap's outline as the later features left it
    if loops.is_none() {
        let later: HashSet<FaceId> = m
            .faces
            .iter()
            .copied()
            .filter(|&g| cx.tags.get(g.idx()).map_or(false, |v| !v.is_empty() && v.iter().all(|t| t.0 > f.id)))
            .collect();
        let side_set: HashSet<FaceId> = sides.iter().copied().collect();
        for level_caps in [&cap_lo, &cap_hi] {
            let mut acc: Vec<Loop2> = vec![];
            // a cap flush with an earlier feature's face is that face's outline, not this profile's
            let own_caps: Vec<FaceId> = level_caps.iter().copied().filter(|&g| cx.tags.get(g.idx()).map_or(true, |v| v.iter().all(|t| t.0 >= f.id))).collect();
            let mut all = !own_caps.is_empty();
            for &g in own_caps.iter() {
                match cap_outline_later(m, g, &side_set, &later, &fr) {
                    Ok(l) => acc.extend(l),
                    Err(e) => {
                        all = false;
                        why.push(format!("cap with later cuts: {e}"));
                    }
                }
            }
            if all {
                loops = Some((acc, "cap outline with later cuts"));
                break;
            }
        }
    }
    let Some((loops, source)) = loops else { return Err(format!("no profile ({})", why.join("; "))) };
    // a cut without caps that reaches the body's extent is a through cut
    let through = cut && caps.is_empty() && {
        let (mut bmin, mut bmax) = (f64::INFINITY, f64::NEG_INFINITY);
        for v in m.b.vertex_ids() {
            let z = m.b.vertex(v).p.dot(fz);
            bmin = bmin.min(z);
            bmax = bmax.max(z);
        }
        if down {
            (lo - bmin).abs() < tol
        } else {
            (hi - bmax).abs() < tol
        }
    };
    let sid = cx.tree.ids.next();
    let mut sk = Sk::new(sid.clone(), fr.plane.clone());
    let eid = format!("F{}", cx.tree.ids.0 + 1);
    let pieces = emit_loops_ids(cx, &mut sk, &loops, &eid);
    name_flush(cx, &pieces, &fr, span, &eid);
    cx.tree.push(sk.json());
    let eid2 = cx.tree.ids.next();
    debug_assert_eq!(eid, eid2);
    // a boss whose cap lies against an earlier feature's face (the same plane, facing the other way, a point of the
    // cap on it) and stayed a face of the file: the file keeps the two as separate bodies (merge result off)
    let separate = !cut && {
        let own: Vec<FaceId> = c0.iter().chain(c1.iter()).copied().collect();
        own.iter().any(|&cap| {
            let Kind::Plane { n: nc, d: dc } = m.kind(cap).clone() else { return false };
            let pc = face_points(m, cap);
            m.faces.iter().any(|&h| {
                h != cap
                    && cx.tags.get(h.idx()).map_or(false, |v| !v.is_empty() && v.iter().all(|t| t.0 < f.id))
                    && matches!(m.kind(h), Kind::Plane { n, d } if (*n + nc).len() < 1e-7 && (d + dc).abs() < 1e-6 * (1.0 + dc.abs()))
                    && pc.iter().any(|&p| !matches!(ck_ops::boolean::classify::point_in_face(m.b, h, p, 1e-6), ck_ops::boolean::classify::Where::Out))
            })
        })
    };
    let mut o = Obj::new().put("type", "extrude").put("id", eid.as_str()).put("op", if cut { "cut" } else if separate { "newBody" } else { "add" }).put("sketch", sid.as_str());
    o = if through { o.put("end", "through") } else { o.put("end", "blind").put("depth", n(depth)) };
    if flip {
        o = o.put("flip", true);
    }
    cx.tree.push(o.build());
    // sides the profile did not touch (a cap outline names only the sides at its rim) still belong to this extrude:
    // a pick on them resolves by feature and position
    for &g in &sides {
        cx.names.entry(g).or_insert_with(|| format!("{eid}.S.e1"));
    }
    // (a cap flush with an earlier face keeps that face's name, as the sides do)
    for g in c0 {
        cx.names.entry(g).or_insert_with(|| format!("{eid}.C0"));
    }
    for g in c1 {
        cx.names.entry(g).or_insert_with(|| format!("{eid}.C1"));
    }
    let nseg: usize = loops.iter().map(|l| l.segs.len()).sum();
    Ok(format!("{}; {} loop(s), {} segment(s) from the {}{}{}", dim_note(cx, f, "D1", "depth", depth), loops.len(), nseg, source, if through { "; through" } else { "" }, if extended { "; sides extended by later features" } else { "" }) + if from_d1 { "; far end from the listed depth" } else { "" })
}
