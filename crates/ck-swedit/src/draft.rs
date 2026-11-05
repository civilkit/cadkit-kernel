//! Drafted extrudes (boss and cut with a draft angle) read back from their faces.
//!
//! A drafted extrude's sides are planes tilted against the direction and cones about it, all by the same angle
//! (`|n . d| = sin(angle)` for every side); its caps are planes normal to the direction. The draft is about the
//! sketch plane (the neutral plane: the profile there is the sketch's), so the profile is read at the sketch level:
//!
//! * from the outline of a cap at the sketch level whose rim borders only the feature's sides (a base boss's
//!   bottom), else
//! * from the cycle of the side faces through their shared edges: every side's trace at the sketch level (a plane
//!   gives a line, a cone a circle about its axis), consecutive traces meeting at the corner nearest to the shared
//!   edge's end at that level.
//!
//! The sign: a positive angle (ck-ops draft) tilts the tool's side normals towards its travel; `draft` tapers the
//! tool inward, `draftOut` outward. For a boss the tool's normals are the body's, for a cut the reverse.

use crate::profile::{cap_outline, emit_loops, Loop2, Seg2};
use crate::{dim_note, face_points, Cx, Fr, TOL};
use ck_geom::Vec3;
use ck_json::Obj;
use ck_recog::emit::{n, Sk};
use ck_recog::model::{principal, Kind, Model, PlaneSpec};
use ck_sldprt::meta::SwFeature;
use ck_topo::body::FaceId;
use std::collections::HashSet;
use std::f64::consts::TAU;

type P2 = (f64, f64);

/// Smallest draft taken as a draft (radians); below it the side is straight and the plain extrude applies.
const MIN_DRAFT: f64 = 1e-6;

/// Outward normal of a side face at a point of it.
fn normal_at(m: &Model, f: FaceId, p: Vec3) -> Option<Vec3> {
    match m.kind(f) {
        Kind::Plane { n, .. } => Some(*n),
        Kind::Cyl { o, a, convex, .. } => {
            let r = (p - *o) - *a * (p - *o).dot(*a);
            (r.len() > 1e-9).then(|| r.normalize() * if *convex { 1.0 } else { -1.0 })
        }
        Kind::Cone { o, a, half, convex, .. } => {
            let r = (p - *o) - *a * (p - *o).dot(*a);
            if r.len() < 1e-9 {
                return None;
            }
            let s = r.normalize() * half.cos() - *a * half.sin();
            Some(s * if *convex { 1.0 } else { -1.0 })
        }
        _ => None,
    }
}

/// A point inside a face (the mid point of its longest edge), for normals.
fn sample(m: &Model, f: FaceId) -> Vec3 {
    let es = m.edges(f);
    es.iter().copied().max_by(|&x, &y| m.p0(x).dist(m.p1(x)).total_cmp(&m.p0(y).dist(m.p1(y)))).map_or(Vec3::ZERO, |e| m.mid(e))
}

/// One reading of the faces as a drafted extrude along `d`: caps, sides and `n . d` of the sides (signed, the
/// same for all).
struct Reading {
    d: Vec3,
    caps: Vec<FaceId>,
    sides: Vec<FaceId>,
    /// `n . d` of every side's outward normal.
    t: f64,
}

fn read_dir(m: &Model, faces: &[FaceId], d: Vec3) -> Option<Reading> {
    let (mut caps, mut sides) = (vec![], vec![]);
    let mut t: Option<f64> = None;
    for &f in faces {
        match m.kind(f) {
            Kind::Plane { n, .. } if n.dot(d).abs() > 1.0 - 1e-9 => {
                caps.push(f);
                continue;
            }
            Kind::Plane { .. } => {}
            Kind::Cone { a, .. } | Kind::Cyl { a, .. } if a.dot(d).abs() > 1.0 - 1e-9 => {}
            _ => return None,
        }
        let nf = normal_at(m, f, sample(m, f))?;
        let tf = nf.dot(d);
        match t {
            None => t = Some(tf),
            Some(t0) if (t0 - tf).abs() < 1e-7 => {}
            _ => return None,
        }
        sides.push(f);
    }
    let t = t?;
    (t.abs() > MIN_DRAFT.sin()).then_some(Reading { d, caps, sides, t })
}

/// The side's trace at the sketch level: a line (point, unit direction) or a circle.
#[derive(Debug, Clone, Copy)]
enum Trace {
    Line { p: P2, d: P2 },
    Circle { c: P2, r: f64 },
}

fn trace_at(m: &Model, f: FaceId, fr: &Fr) -> Result<Trace, String> {
    match m.kind(f) {
        Kind::Plane { n, d } => {
            let n2 = (n.dot(fr.x), n.dot(fr.y));
            let l2 = n2.0 * n2.0 + n2.1 * n2.1;
            if l2 < 1e-12 {
                return Err("side parallel to the sketch".into());
            }
            let c = d - n.dot(fr.o);
            let l = l2.sqrt();
            Ok(Trace::Line { p: (n2.0 * c / l2, n2.1 * c / l2), d: (-n2.1 / l, n2.0 / l) })
        }
        Kind::Cone { o, a, r0, half, .. } => {
            let az = a.dot(fr.z);
            // the axis' point at the sketch level
            let v = (fr.o - *o).dot(fr.z) / az;
            let c = *o + *a * v;
            let r = r0 + v * half.tan();
            if !(r > 1e-9) {
                return Err("cone side without a radius at the sketch level".into());
            }
            Ok(Trace::Circle { c: fr.xy(c), r })
        }
        Kind::Cyl { o, r, .. } => Ok(Trace::Circle { c: fr.xy(*o), r: *r }),
        k => Err(format!("{} side", k.name())),
    }
}

/// Intersections of two traces; a near miss of tangent traces (rounding) gives the touch point.
fn meet(a: &Trace, b: &Trace) -> Vec<P2> {
    let circ_line = |c: P2, r: f64, p: P2, d: P2| -> Vec<P2> {
        let f = (p.0 - c.0, p.1 - c.1);
        let bq = f.0 * d.0 + f.1 * d.1;
        let cq = f.0 * f.0 + f.1 * f.1 - r * r;
        let disc = bq * bq - cq;
        if disc < -1e-6 * r * r {
            return vec![];
        }
        let sq = disc.max(0.0).sqrt();
        vec![(p.0 + (-bq - sq) * d.0, p.1 + (-bq - sq) * d.1), (p.0 + (-bq + sq) * d.0, p.1 + (-bq + sq) * d.1)]
    };
    match (a, b) {
        (Trace::Line { p: p1, d: d1 }, Trace::Line { p: p2, d: d2 }) => {
            let den = d1.0 * d2.1 - d1.1 * d2.0;
            if den.abs() < 1e-12 {
                return vec![];
            }
            let t = ((p2.0 - p1.0) * d2.1 - (p2.1 - p1.1) * d2.0) / den;
            vec![(p1.0 + t * d1.0, p1.1 + t * d1.1)]
        }
        (Trace::Line { p, d }, Trace::Circle { c, r }) | (Trace::Circle { c, r }, Trace::Line { p, d }) => circ_line(*c, *r, *p, *d),
        (Trace::Circle { c: c1, r: r1 }, Trace::Circle { c: c2, r: r2 }) => {
            let dx = (c2.0 - c1.0, c2.1 - c1.1);
            let dd = dx.0.hypot(dx.1);
            if dd < 1e-12 {
                return vec![];
            }
            let x = (dd * dd + r1 * r1 - r2 * r2) / (2.0 * dd);
            let h2 = r1 * r1 - x * x;
            if h2 < -1e-6 * r1 * r1 {
                return vec![];
            }
            let h = h2.max(0.0).sqrt();
            let u = (dx.0 / dd, dx.1 / dd);
            let q = (c1.0 + x * u.0, c1.1 + x * u.1);
            vec![(q.0 - h * u.1, q.1 + h * u.0), (q.0 + h * u.1, q.1 - h * u.0)]
        }
    }
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

fn dist2(a: P2, b: P2) -> f64 {
    (a.0 - b.0).hypot(a.1 - b.1)
}

/// The profile at the sketch level from the cycles of the side faces through their shared edges.
fn side_cycles(m: &Model, sides: &[FaceId], fr: &Fr) -> Result<Vec<Loop2>, String> {
    let set: HashSet<FaceId> = sides.iter().copied().collect();
    let level = fr.o.dot(fr.z);
    // neighbours of a side among the sides, with the shared edges' points nearest to the sketch level
    let nbrs = |w: FaceId| -> Vec<(FaceId, Vec3)> {
        let mut out: Vec<(FaceId, Vec3)> = vec![];
        for e in m.edges(w) {
            let Some(g) = m.across(e, w).filter(|g| set.contains(g)) else { continue };
            let (p, q) = (m.p0(e), m.p1(e));
            let near = if (p.dot(fr.z) - level).abs() <= (q.dot(fr.z) - level).abs() { p } else { q };
            match out.iter_mut().find(|o| o.0 == g) {
                Some(o) if (near.dot(fr.z) - level).abs() < (o.1.dot(fr.z) - level).abs() => o.1 = near,
                Some(_) => {}
                None => out.push((g, near)),
            }
        }
        out
    };
    let mut order: Vec<FaceId> = sides.to_vec();
    order.sort();
    let mut visited: HashSet<FaceId> = HashSet::new();
    let mut loops = vec![];
    for &w in &order {
        if visited.contains(&w) {
            continue;
        }
        let nw = nbrs(w);
        if nw.is_empty() {
            let Trace::Circle { c, r } = trace_at(m, w, fr)? else {
                return Err(format!("side {} borders no other side", m.b.face_id_str(w)));
            };
            visited.insert(w);
            loops.push(Loop2 { segs: vec![(Seg2::Circle { c, r }, vec![w])] });
            continue;
        }
        // the cycle: faces in order and the corner between each and the next
        let mut cyc: Vec<FaceId> = vec![w];
        let mut prev: Option<FaceId> = None;
        let mut cur = w;
        let mut corners: Vec<P2> = vec![];
        loop {
            visited.insert(cur);
            let nc = nbrs(cur);
            if nc.len() != 2 {
                return Err(format!("side {} borders {} other side(s)", m.b.face_id_str(cur), nc.len()));
            }
            let (next, near) = if Some(nc[0].0) == prev { nc[1] } else { nc[0] };
            let (ta, tb) = (trace_at(m, cur, fr)?, trace_at(m, next, fr)?);
            let est = fr.xy(near);
            let corner = meet(&ta, &tb)
                .into_iter()
                .min_by(|x, y| dist2(*x, est).total_cmp(&dist2(*y, est)))
                .ok_or_else(|| format!("sides {} and {} do not meet at the sketch level", m.b.face_id_str(cur), m.b.face_id_str(next)))?;
            corners.push(corner);
            prev = Some(cur);
            cur = next;
            if cur == w {
                break;
            }
            if visited.contains(&cur) || cyc.len() > sides.len() {
                return Err("side cycle does not close".into());
            }
            cyc.push(cur);
        }
        // side k runs from corner k-1 to corner k
        let k_n = cyc.len();
        let mut segs = vec![];
        for k in 0..k_n {
            let g = cyc[k];
            let (a, b) = (corners[(k + k_n - 1) % k_n], corners[k]);
            let seg = match trace_at(m, g, fr)? {
                Trace::Line { .. } => Seg2::Line { a, b },
                Trace::Circle { c, .. } => {
                    if dist2(a, b) < 1e-9 {
                        return Err("arc side with coincident corners".into());
                    }
                    // angles about the axis do not change with the level: vote with the face's points
                    let (ta, tb) = (ang(c, a), ang(c, b));
                    let sb = wrap(tb - ta);
                    let mut votes = 0i32;
                    for p in face_points(m, g) {
                        let sa = wrap(ang(c, fr.xy(p)) - ta);
                        if sa > 1e-6 && (sb - sa).abs() > 1e-6 && TAU - sa > 1e-6 {
                            votes += if sa < sb { 1 } else { -1 };
                        }
                    }
                    Seg2::Arc { a, b, c, ccw: votes >= 0 }
                }
            };
            segs.push((seg, vec![g]));
        }
        crate::profile::merge_segs(&mut segs);
        loops.push(Loop2 { segs });
    }
    if loops.is_empty() {
        return Err("no side cycle".into());
    }
    Ok(loops)
}

/// A point of a side carried along the side's ruling to the sketch level (in the sketch frame): a plane's ruling is
/// its line of steepest slope, a cone's runs through the apex (its angle about the axis is kept).
fn to_level(m: &Model, f: FaceId, p: Vec3, fr: &Fr, tr: &Trace) -> Option<P2> {
    match (m.kind(f), tr) {
        (Kind::Plane { n, .. }, _) => {
            let l = fr.z.cross(*n);
            if l.len() < 1e-9 {
                return None;
            }
            let u = n.cross(l.normalize());
            let uz = u.dot(fr.z);
            if uz.abs() < 1e-9 {
                return None;
            }
            Some(fr.xy(p - u * ((p - fr.o).dot(fr.z) / uz)))
        }
        (_, Trace::Circle { c, r }) => {
            let t = ang(*c, fr.xy(p));
            Some((c.0 + r * t.cos(), c.1 + r * t.sin()))
        }
        _ => None,
    }
}

/// The profile at the sketch level from the sides' traces when their shared edges are gone (later fillets or
/// chamfers on the drafted corners): every side has two ends (its points carried to the sketch level, extreme
/// along a line, the bounds of the largest free gap on a circle); ends are paired across sides at the
/// intersection of their traces nearest to both, cheapest first, which recovers the corners the later features
/// removed.
fn side_chain(m: &Model, sides: &[FaceId], fr: &Fr) -> Result<Vec<Loop2>, String> {
    let mut traces: Vec<Trace> = vec![];
    let mut ends: Vec<(usize, P2)> = vec![];
    let mut length: Vec<f64> = vec![];
    for (gi, &g) in sides.iter().enumerate() {
        let tr = trace_at(m, g, fr)?;
        let pts: Vec<P2> = face_points(m, g).into_iter().map(|p| to_level(m, g, p, fr, &tr)).collect::<Option<_>>().ok_or("side without a ruling")?;
        match tr {
            Trace::Line { p, d } => {
                let tt = |q: &P2| (q.0 - p.0) * d.0 + (q.1 - p.1) * d.1;
                let (lo, hi) = pts.iter().fold((f64::INFINITY, f64::NEG_INFINITY), |a, q| (a.0.min(tt(q)), a.1.max(tt(q))));
                ends.push((gi, (p.0 + lo * d.0, p.1 + lo * d.1)));
                ends.push((gi, (p.0 + hi * d.0, p.1 + hi * d.1)));
                length.push(hi - lo);
            }
            Trace::Circle { c, r } => {
                length.push(TAU * r);
                let mut a: Vec<f64> = pts.iter().map(|&q| wrap(ang(c, q))).collect();
                a.sort_by(|x, y| x.total_cmp(y));
                a.dedup_by(|x, y| (*x - *y).abs() < 1e-9);
                if a.len() < 2 {
                    return Err("arc side with one point".into());
                }
                // the arc's span is the complement of the largest gap between its points
                let (gap, i) = (0..a.len()).map(|i| (wrap(a[(i + 1) % a.len()] - a[i]), i)).fold((0.0, 0), |b, x| if x.0 > b.0 { x } else { b });
                if gap < 1e-6 {
                    traces.push(tr);
                    continue; // a full circle
                }
                let (t0, t1) = (a[i], a[(i + 1) % a.len()]);
                ends.push((gi, (c.0 + r * t0.cos(), c.1 + r * t0.sin())));
                ends.push((gi, (c.0 + r * t1.cos(), c.1 + r * t1.sin())));
            }
        }
        traces.push(tr);
    }
    let same = |a: &Trace, b: &Trace| match (a, b) {
        (Trace::Line { p: p1, d: d1 }, Trace::Line { p: p2, d: d2 }) => {
            (d1.0 * d2.1 - d1.1 * d2.0).abs() < 1e-7 && ((p2.0 - p1.0) * d1.1 - (p2.1 - p1.1) * d1.0).abs() < 1e-6
        }
        (Trace::Circle { c: c1, r: r1 }, Trace::Circle { c: c2, r: r2 }) => dist2(*c1, *c2) < 1e-6 && (r1 - r2).abs() < 1e-6,
        _ => false,
    };
    let mut cands: Vec<(f64, usize, usize, P2)> = vec![];
    for i in 0..ends.len() {
        for j in i + 1..ends.len() {
            let (gi, gj) = (ends[i].0, ends[j].0);
            if gi == gj {
                continue;
            }
            let (ei, ej) = (ends[i].1, ends[j].1);
            if same(&traces[gi], &traces[gj]) {
                // one side split by a later feature
                let cost = dist2(ei, ej);
                if cost <= length[gi].max(length[gj]) {
                    cands.push((cost, i, j, ((ei.0 + ej.0) / 2.0, (ei.1 + ej.1) / 2.0)));
                }
                continue;
            }
            for p in meet(&traces[gi], &traces[gj]) {
                let cost = dist2(ei, p) + dist2(ej, p);
                if cost <= 2.0 * (length[gi] + length[gj]) {
                    cands.push((cost, i, j, p));
                }
            }
        }
    }
    cands.sort_by(|a, b| a.0.total_cmp(&b.0));
    let mut taken = vec![false; ends.len()];
    let mut links: Vec<(usize, usize, P2)> = vec![];
    for (_, i, j, p) in cands {
        if !taken[i] && !taken[j] {
            taken[i] = true;
            taken[j] = true;
            links.push((ends[i].0, ends[j].0, p));
        }
    }
    if let Some(k) = (0..ends.len()).find(|&k| !taken[k]) {
        return Err(format!("side {} has an end ({:.3},{:.3}) nothing meets", m.b.face_id_str(sides[ends[k].0]), ends[k].1 .0, ends[k].1 .1));
    }
    let mut used = vec![false; sides.len()];
    let mut loops = vec![];
    for start in 0..sides.len() {
        if used[start] {
            continue;
        }
        let deg = links.iter().filter(|l| l.0 == start || l.1 == start).count();
        if deg == 0 {
            let Trace::Circle { c, r } = traces[start] else { return Err(format!("side {} is linked to nothing", m.b.face_id_str(sides[start]))) };
            used[start] = true;
            loops.push(Loop2 { segs: vec![(Seg2::Circle { c, r }, vec![sides[start]])] });
            continue;
        }
        if deg != 2 {
            return Err(format!("side {} links to {deg} others", m.b.face_id_str(sides[start])));
        }
        let mut order: Vec<(usize, P2)> = vec![];
        let (mut prev, mut cur) = (None, start);
        loop {
            used[cur] = true;
            let li = (0..links.len()).find(|&li| Some(li) != prev && (links[li].0 == cur || links[li].1 == cur)).ok_or("side chain broken")?;
            let l = links[li];
            order.push((cur, l.2));
            prev = Some(li);
            cur = if l.0 == cur { l.1 } else { l.0 };
            if cur == start {
                break;
            }
            if used[cur] || order.len() > sides.len() {
                return Err("side chain does not close".into());
            }
        }
        let k_n = order.len();
        let mut segs: Vec<(Seg2, Vec<FaceId>)> = vec![];
        for k in 0..k_n {
            let (gi, b) = order[k];
            let a = order[(k + k_n - 1) % k_n].1;
            let seg = match traces[gi] {
                Trace::Line { .. } => Seg2::Line { a, b },
                Trace::Circle { c, .. } => {
                    if dist2(a, b) < 1e-9 {
                        return Err("arc side with coincident corners".into());
                    }
                    let (ta, sb) = (ang(c, a), wrap(ang(c, b) - ang(c, a)));
                    let mut votes = 0i32;
                    for p in face_points(m, sides[gi]) {
                        let sa = wrap(ang(c, fr.xy(p)) - ta);
                        if sa > 1e-6 && (sb - sa).abs() > 1e-6 && TAU - sa > 1e-6 {
                            votes += if sa < sb { 1 } else { -1 };
                        }
                    }
                    Seg2::Arc { a, b, c, ccw: votes >= 0 }
                }
            };
            segs.push((seg, vec![sides[gi]]));
        }
        crate::profile::merge_segs(&mut segs);
        loops.push(Loop2 { segs });
    }
    Ok(loops)
}

/// Read a drafted extrude back from its faces and emit plane, sketch and extrude with `draft`.
pub fn extrude(cx: &mut Cx, f: &SwFeature, faces: &[FaceId], cut: bool) -> Result<String, String> {
    let m = cx.m;
    // candidate directions: the caps' normals (tagged caps first), the cones' axes
    let mut cands: Vec<Vec3> = vec![];
    let push = |v: Vec3, cands: &mut Vec<Vec3>| {
        if !cands.iter().any(|c| c.dot(v).abs() > 1.0 - 1e-9) {
            cands.push(v);
        }
    };
    for pass in 0..2 {
        for &g in faces {
            match m.kind(g) {
                Kind::Plane { n, .. } if (pass == 0) == cx.is_cap(g, f.id) => push(*n, &mut cands),
                Kind::Cone { a, .. } if pass == 1 => push(*a, &mut cands),
                _ => {}
            }
        }
    }
    let r = cands
        .into_iter()
        .filter_map(|d| read_dir(m, faces, d))
        .filter(|r| !r.sides.is_empty())
        .next()
        .ok_or_else(|| "faces are no drafted extrusion (sides not tilted by one angle about a cap normal or cone axis)".to_string())?;
    let fz = match principal(r.d) {
        Some((i, _)) => PlaneSpec::for_axis(i).zs,
        None => r.d,
    };
    // `t` along fz
    let t = r.t * r.d.dot(fz).signum();
    let (mut lo, mut hi) = (f64::INFINITY, f64::NEG_INFINITY);
    for &g in faces {
        for p in face_points(m, g) {
            lo = lo.min(p.dot(fz));
            hi = hi.max(p.dot(fz));
        }
    }
    let level_of = |g: FaceId| -> f64 {
        match m.kind(g) {
            Kind::Plane { n, d } => d * n.dot(fz),
            _ => f64::NAN,
        }
    };
    let mut levels: Vec<f64> = r.caps.iter().map(|&g| level_of(g)).collect();
    levels.sort_by(|a, b| a.total_cmp(b));
    levels.dedup_by(|a, b| (*a - *b).abs() < TOL * (1.0 + a.abs()));
    if levels.len() > 2 {
        return Err("caps at more than two levels".into());
    }
    if levels.len() == 2 {
        if levels[0] > lo + TOL || levels[1] < hi - TOL {
            return Err("drafted sides extended beyond the caps by later features".into());
        }
        lo = levels[0];
        hi = levels[1];
    }
    // one cap: the far end's faces may be gone (later features consumed the sides' ends); the listed depth reaches
    // further than the faces do, take it from the cap
    let mut from_dim = false;
    if levels.len() == 1 {
        if let Some(d1) = crate::dim(cx, f, "D1") {
            if d1 > hi - lo + TOL * (1.0 + d1) {
                if (levels[0] - hi).abs() < TOL * (1.0 + hi.abs()) {
                    lo = hi - d1;
                } else if (levels[0] - lo).abs() < TOL * (1.0 + lo.abs()) {
                    hi = lo + d1;
                }
                from_dim = true;
            }
        }
    }
    let depth = hi - lo;
    if !(depth > 1e-6) {
        return Err("no extent along the direction".into());
    }
    let tol = TOL * (1.0 + lo.abs().max(hi.abs()));
    let (mut cap_lo, mut cap_hi) = (vec![], vec![]);
    for &g in &r.caps {
        let z = level_of(g);
        if (z - lo).abs() < tol {
            cap_lo.push(g);
        } else if (z - hi).abs() < tol {
            cap_hi.push(g);
        } else {
            return Err("cap at an intermediate level".into());
        }
    }
    // the sketch at the bottom of a boss, at the mouth of a cut (as the plain extrude)
    let (level, down, c0, c1): (f64, bool, Vec<FaceId>, Vec<FaceId>) = if !cut {
        (lo, false, cap_lo.clone(), cap_hi.clone())
    } else if !cap_lo.is_empty() {
        (hi, true, vec![], cap_lo.clone())
    } else if !cap_hi.is_empty() {
        (lo, false, vec![], cap_hi.clone())
    } else {
        (hi, true, vec![], vec![])
    };
    let flip = down != cut;
    // the tool travels along +fz unless `down`; its side normals are the body's for a boss, reversed for a cut
    let s = t * if down { -1.0 } else { 1.0 } * if cut { -1.0 } else { 1.0 };
    let angle = s.abs().asin().to_degrees();
    let outward = s < 0.0;
    let fr = cx.frame(fz, fz * level);
    let side_set: HashSet<FaceId> = r.sides.iter().copied().collect();
    let mut why: Vec<String> = vec![];
    let mut loops: Option<(Vec<Loop2>, &str)> = None;
    // a cap at the sketch level carries the sketch's own profile
    for &g in if level == lo { &cap_lo } else { &cap_hi } {
        match cap_outline(m, g, &side_set, &fr) {
            Ok(l) => {
                loops = Some((l, "cap outline"));
                break;
            }
            Err(e) => why.push(format!("cap: {e}")),
        }
    }
    if loops.is_none() {
        match side_cycles(m, &r.sides, &fr) {
            Ok(l) => loops = Some((l, "side traces at the sketch level")),
            Err(e) => why.push(format!("sides: {e}")),
        }
    }
    if loops.is_none() {
        match side_chain(m, &r.sides, &fr) {
            Ok(l) => loops = Some((l, "side traces at the sketch level (corners re-met)")),
            Err(e) => why.push(format!("traces: {e}")),
        }
    }
    let Some((loops, source)) = loops else { return Err(format!("no drafted profile ({})", why.join("; "))) };
    // a cut without caps reaching the body's extent goes through: blind a little past the body (a drafted tool
    // of the body's full length may close up)
    let through = cut && r.caps.is_empty() && {
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
    let len = if through { depth + (0.01 * depth).max(0.01) } else { depth };
    let sid = cx.tree.ids.next();
    let mut sk = Sk::new(sid.clone(), fr.plane.clone());
    let eid = format!("F{}", cx.tree.ids.0 + 1);
    emit_loops(cx, &mut sk, &loops, &eid);
    cx.tree.push(sk.json());
    let eid2 = cx.tree.ids.next();
    debug_assert_eq!(eid, eid2);
    let mut o = Obj::new()
        .put("type", "extrude")
        .put("id", eid.as_str())
        .put("op", if cut { "cut" } else { "add" })
        .put("sketch", sid.as_str())
        .put("end", "blind")
        .put("depth", n(len))
        .put("draft", n(angle));
    if outward {
        o = o.put("draftOut", true);
    }
    if flip {
        o = o.put("flip", true);
    }
    cx.tree.push(o.build());
    for &g in &r.sides {
        cx.names.entry(g).or_insert_with(|| format!("{eid}.S.e1"));
    }
    for g in c0 {
        cx.names.insert(g, format!("{eid}.C0"));
    }
    for g in c1 {
        cx.names.insert(g, format!("{eid}.C1"));
    }
    let nseg: usize = loops.iter().map(|l| l.segs.len()).sum();
    Ok(format!(
        "{}; draft {angle:.4} deg {}; {} loop(s), {} segment(s) from the {source}{}{}",
        dim_note(cx, f, "D1", "depth", depth),
        if outward { "outward" } else { "inward" },
        loops.len(),
        nseg,
        if through { "; through" } else { "" },
        if from_dim { "; far end from the listed depth (its faces are gone)" } else { "" }
    ))
}

/// The plain extrude, else (its sides are tilted) the drafted one; a failed plain attempt leaves nothing behind.
pub fn plain_or_drafted(cx: &mut Cx, f: &SwFeature, faces: &[FaceId], cut: bool) -> Result<String, String> {
    let (ids, nfeat, nnotes, names) = (cx.tree.ids.0, cx.tree.feats.len(), cx.notes.len(), cx.names.clone());
    match crate::profile::extrude(cx, f, faces, cut) {
        Ok(note) => Ok(note),
        Err(plain) => {
            cx.tree.ids.0 = ids;
            cx.tree.feats.truncate(nfeat);
            cx.notes.truncate(nnotes);
            cx.names = names;
            let tilted = faces.iter().any(|&g| match cx.m.kind(g) {
                Kind::Cone { .. } => true,
                Kind::Plane { n, .. } => faces.iter().any(|&h| matches!(cx.m.kind(h), Kind::Plane { n: m2, .. } if { let c = n.dot(*m2).abs(); c > 1e-6 && c < 1.0 - 1e-6 })),
                _ => false,
            });
            if !tilted {
                return Err(plain);
            }
            extrude(cx, f, faces, cut).map_err(|d| format!("{plain}; as a drafted extrude: {d}"))
        }
    }
}
