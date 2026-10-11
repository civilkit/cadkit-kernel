//! The `convert` request: model edges / faces / sketch curves projected into
//! a sketch frame as new fixed sketch entities.
//!
//! `{sketch, frame:{o,x,y,z}, ref}` -> `{sketch, ids, skipped}` (`null` when
//! the reference does not resolve). Per edge, in order (faces: loop order of
//! the face's edges, seam edges dropped):
//!
//! * line -> `line` (skipped when both ends project to one point);
//! * circle parallel to the sketch plane -> `circle` (full) / `arc` (centre,
//!   start, end; ends swapped when the circle normal opposes the frame z so
//!   the arc stays counter-clockwise);
//! * circle edge-on -> `line` (full: leftmost -> rightmost; arc: start ->
//!   end), skipped when degenerate;
//! * full circle seen obliquely -> `ellipse` (centre, major-axis end, minor
//!   radius);
//! * everything else -> `spline` through 129 curve samples (128 intervals).
//!
//! Points are looked up by position (< 1e-9) among the sketch's points
//! (a hit is marked `fixed`), else appended as `fixed` points; ids and the
//! `seq` counter share one numbering (`p<seq>` / `e<seq>`; `seq` defaults to
//! `1 + points + entities`). Entities carry
//! `from:{kind:"edge",faces:[sorted],at:<edge mid>,shape:<curve kind>}`.

use crate::measure::{resolve_ref, Ref};
use ck_core::linalg::{Frame, Vec3};
use ck_geom::curve::Curve;
use ck_json::{Json, Obj};
use ck_topo::body::{Body, EdgeId};

type P2 = [f64; 2];

/// A projected entity.
#[derive(Debug, Clone, PartialEq)]
pub enum Ent {
    Line(P2, P2),
    Circle(P2, f64),
    /// centre, start, end
    Arc(P2, P2, P2),
    /// centre, major-axis end, minor radius
    Ellipse(P2, P2, f64),
    Spline(Vec<P2>),
}

/// An entity with its `from` object.
pub struct Item {
    pub ent: Ent,
    pub from: Json,
}

fn proj(f: &Frame, p: Vec3) -> P2 {
    let d = p - f.o;
    [d.dot(f.x), d.dot(f.y)]
}

fn near(a: P2, b: P2) -> bool {
    (a[0] - b[0]).hypot(a[1] - b[1]) < 1e-9
}

fn hyp(a: P2, b: P2) -> f64 {
    ((a[0] - b[0]) * (a[0] - b[0]) + (a[1] - b[1]) * (a[1] - b[1])).sqrt()
}

/// Project the curve segment `[t0,t1]` of `c` into frame `f`.
/// `None` = skipped. Wasm f1163 (shared with the convert feature).
pub fn project_curve(c: &Curve, t0: f64, t1: f64, closed: bool, f: &Frame) -> Option<Ent> {
    const TAU_EPS: f64 = 6.283185306179586; // 2 pi - 1e-9
    let sweep = (t1 - t0).abs();
    match c {
        Curve::Line { .. } => {
            let (a, b) = (proj(f, c.eval(t0)), proj(f, c.eval(t1)));
            return (!near(a, b)).then_some(Ent::Line(a, b));
        }
        Curve::Circle { frame, r } => {
            let nrm = frame.x.cross(frame.y).normalize();
            let nz = nrm.dot(f.z);
            let ctr = proj(f, frame.o);
            if (nz.abs() - 1.0).abs() < 1e-9 {
                if sweep <= TAU_EPS {
                    let (s, e) = (proj(f, c.eval(t0)), proj(f, c.eval(t1)));
                    return Some(if (t0 < t1) != (nz > 0.0) { Ent::Arc(ctr, e, s) } else { Ent::Arc(ctr, s, e) });
                }
                return Some(Ent::Circle(ctr, *r));
            }
            if sweep > TAU_EPS {
                // full circle seen obliquely: the ellipse of its projected
                // conjugate radii (None when edge-on)
                let a = [frame.x.dot(f.x) * *r, frame.x.dot(f.y) * *r];
                let b = [frame.y.dot(f.x) * *r, frame.y.dot(f.y) * *r];
                if let Some(e) = ellipse_of(ctr, a, b) {
                    return Some(e);
                }
            }
        }
        _ => {}
    }
    generic(c, t0, t1, closed, f)
}

/// Ellipse `ctr + a cos t + b sin t` as (centre, major-axis end, minor
/// radius); `None` when degenerate (minor radius ~ 0).
fn ellipse_of(ctr: P2, a: P2, b: P2) -> Option<Ent> {
    let (aa, bb, ab) = (a[0] * a[0] + a[1] * a[1], b[0] * b[0] + b[1] * b[1], a[0] * b[0] + a[1] * b[1]);
    let t = 0.5 * ab.atan2(0.5 * (aa - bb)) ;
    let (c, s) = (t.cos(), t.sin());
    let u = [a[0] * c + b[0] * s, a[1] * c + b[1] * s];
    let v = [-a[0] * s + b[0] * c, -a[1] * s + b[1] * c];
    let (lu, lv) = (u[0].hypot(u[1]), v[0].hypot(v[1]));
    if lv < 1e-9 || lu < lv {
        return None;
    }
    Some(Ent::Ellipse(ctr, [ctr[0] + u[0], ctr[1] + u[1]], lv))
}

/// Sampled projection: 2049 samples; a line when all of them lie within
/// 1e-5 of the farthest pair's line, else a spline.
fn generic(c: &Curve, t0: f64, t1: f64, closed: bool, f: &Frame) -> Option<Ent> {
    let len = t1 - t0;
    let q: Vec<P2> = (0..=2048)
        .map(|i| proj(f, c.eval(if i == 2048 { t0 + len } else { t0 + (len * i as f64) * 0.00048828125 })))
        .collect();
    let (mut lo, mut hi) = ([f64::INFINITY; 2], [f64::NEG_INFINITY; 2]);
    for p in &q {
        for k in 0..2 {
            lo[k] = lo[k].min(p[k]);
            hi[k] = hi[k].max(p[k]);
        }
    }
    let diag = ((hi[0] - lo[0]).powi(2) + (hi[1] - lo[1]).powi(2)).sqrt();
    if diag < 1e-9 {
        return None;
    }
    let tol = diag * 1e-5;
    // farthest from the first sample, then farthest from that (strictly
    // farther wins, so ties keep the earliest sample)
    let far = |from: P2| {
        let mut best = from;
        for p in &q {
            if hyp(*p, from) > hyp(best, from) {
                best = *p;
            }
        }
        best
    };
    let a = far(q[0]);
    let b = far(a);
    let l = hyp(a, b).max(1e-300);
    let (ux, uy) = ((b[1] - a[1]) * (1.0 / l), (b[0] - a[0]) * (1.0 / l));
    if q.iter().all(|p| (ux * (p[0] - a[0]) - uy * (p[1] - a[1])).abs() <= tol) {
        let (mut a, mut b) = (a, b);
        if !closed && l - tol <= hyp(q[0], q[2048]) {
            a = q[0];
            b = q[2048];
        }
        return Some(Ent::Line(a, b));
    }
    // spline: n = 16, 32, .. 512 uniform curve samples, fitted by a natural
    // chord-length cubic spline, until that spline (sampled 16x per interval)
    // stays within `tol` of every 4th dense sample (or n reaches 512). The
    // sketch spline entity keeps the n+1 fit points.
    let at = |n: usize| -> Vec<P2> { (0..=n).map(|i| proj(f, c.eval(t0 + (len * i as f64) / n as f64))).collect() };
    let mut n = 16;
    loop {
        let pts = at(n);
        let poly = natural_spline_polyline(&pts, 16);
        let dev = q
            .iter()
            .step_by(4)
            .map(|p| poly.windows(2).map(|w| seg_dist(*p, w[0], w[1])).fold(f64::INFINITY, f64::min))
            .fold(0.0, f64::max);
        if n >= 512 || dev <= tol {
            return Some(Ent::Spline(pts));
        }
        n *= 2;
    }
}

/// Natural cubic spline through `pts` parameterised by chord length,
/// sampled `per` times per interval (+ the end point).
fn natural_spline_polyline(pts: &[P2], per: usize) -> Vec<P2> {
    let n = pts.len();
    let mut s = vec![0.0; n];
    for i in 1..n {
        s[i] = s[i - 1] + hyp(pts[i], pts[i - 1]);
    }
    // second derivatives per coordinate (tridiagonal, natural ends)
    let moments = |k: usize| -> Vec<f64> {
        let mut m = vec![0.0; n];
        if n < 3 {
            return m;
        }
        let (mut cp, mut dp) = (vec![0.0; n], vec![0.0; n]);
        for i in 1..n - 1 {
            let (h0, h1) = (s[i] - s[i - 1], s[i + 1] - s[i]);
            let (a, b, c) = (h0, 2.0 * (h0 + h1), h1);
            let d = 6.0 * ((pts[i + 1][k] - pts[i][k]) / h1 - (pts[i][k] - pts[i - 1][k]) / h0);
            let den = b - a * cp[i - 1];
            cp[i] = c / den;
            dp[i] = (d - a * dp[i - 1]) / den;
        }
        for i in (1..n - 1).rev() {
            m[i] = dp[i] - cp[i] * m[i + 1];
        }
        m
    };
    let (mx, my) = (moments(0), moments(1));
    let mut out = Vec::with_capacity((n - 1) * per + 1);
    for i in 0..n - 1 {
        let h = s[i + 1] - s[i];
        for j in 0..per {
            let t = j as f64 / per as f64;
            let (a, b) = (1.0 - t, t);
            let ev = |k: usize, m: &Vec<f64>| {
                a * pts[i][k] + b * pts[i + 1][k] + ((a * a * a - a) * m[i] + (b * b * b - b) * m[i + 1]) * h * h / 6.0
            };
            out.push([ev(0, &mx), ev(1, &my)]);
        }
    }
    out.push(pts[n - 1]);
    out
}

fn seg_dist(p: P2, a: P2, b: P2) -> f64 {
    let (dx, dy) = (b[0] - a[0], b[1] - a[1]);
    let l2 = dx * dx + dy * dy;
    let t = if l2 > 0.0 { (((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / l2).clamp(0.0, 1.0) } else { 0.0 };
    hyp(p, [a[0] + dx * t, a[1] + dy * t])
}

/// Edges of a reference in conversion order.
fn ref_edges(body: &Body, r: Ref) -> Vec<EdgeId> {
    match r {
        Ref::Edge(e) => vec![e],
        Ref::Face(face) => {
            let mut out: Vec<EdgeId> = vec![];
            for &l in &body.face(face).loops {
                for &c in &body.lp(l).coedges {
                    let e = body.coedge(c).edge;
                    if !body.is_seam(e) && !out.contains(&e) {
                        out.push(e);
                    }
                }
            }
            out
        }
    }
}

/// `from` object of a projected edge.
pub fn edge_from(body: &Body, e: EdgeId, shape: &str) -> Json {
    let ed = body.edge(e);
    let mut fs: Vec<String> = body.edge_faces(e).iter().map(|&f| body.face_id_str(f)).collect();
    fs.sort();
    let at = ed.curve.eval(0.5 * (ed.t0 + ed.t1));
    Obj::new()
        .put("kind", "edge")
        .put("faces", Json::Arr(fs.into_iter().map(Json::Str).collect()))
        .put("at", Json::Arr(vec![Json::Num(at.x), Json::Num(at.y), Json::Num(at.z)]))
        .put("shape", shape)
        .build()
}

/// Project every edge of `r`; `skipped` counts degenerate ones.
pub fn project_ref(body: &Body, r: Ref, f: &Frame) -> (Vec<Item>, usize) {
    let mut items = vec![];
    let mut skipped = 0;
    for e in ref_edges(body, r) {
        let ed = body.edge(e);
        let closed = ed.v0 == ed.v1;
        match project_curve(&ed.curve, ed.t0, ed.t1, closed, f) {
            Some(ent) => items.push(Item { ent, from: edge_from(body, e, ed.curve.kind()) }),
            None => skipped += 1,
        }
    }
    (items, skipped)
}

fn num_pair(p: P2) -> (f64, f64) {
    (p[0], p[1])
}

/// Add `items` to the sketch JSON; returns (sketch, ids).
pub fn apply(sketch: &Json, items: Vec<Item>) -> (Json, Vec<String>) {
    let Json::Obj(kv) = sketch else {
        // the oracle still numbers the entities, but there is nothing to edit
        let ids = (0..items.len()).map(|i| format!("e{}", 3 + i)).collect();
        return (Json::Null, ids);
    };
    let mut kv = kv.clone();
    let slot = |kv: &mut Vec<(String, Json)>, k: &str, default: Json| -> usize {
        match kv.iter().position(|(n, _)| n == k) {
            Some(i) => i,
            None => {
                kv.push((k.to_string(), default));
                kv.len() - 1
            }
        }
    };
    let ip = slot(&mut kv, "points", Json::Arr(vec![]));
    let ie = slot(&mut kv, "entities", Json::Arr(vec![]));
    slot(&mut kv, "constraints", Json::Arr(vec![]));
    let is = slot(&mut kv, "seq", Json::Null);
    let mut points = kv[ip].1.as_arr().map(|a| a.to_vec()).unwrap_or_default();
    let mut ents = kv[ie].1.as_arr().map(|a| a.to_vec()).unwrap_or_default();
    let mut seq = match kv[is].1.as_f64() {
        Some(s) => s as i64,
        None => 1 + points.len() as i64 + ents.len() as i64,
    };
    let mut ids = vec![];
    for it in items {
        let pid = |p: P2, points: &mut Vec<Json>, seq: &mut i64| -> String {
            for q in points.iter_mut() {
                if let (Some(x), Some(y)) = (q.get_f64("x"), q.get_f64("y")) {
                    if (x - p[0]).hypot(y - p[1]) < 1e-7 {
                        q.set("fixed", true);
                        return q.get_str("id").unwrap_or("").to_string();
                    }
                }
            }
            let id = format!("p{}", *seq);
            *seq += 1;
            points.push(Obj::new().put("id", id.as_str()).put("x", p[0]).put("y", p[1]).put("fixed", true).build());
            id
        };
        let mut o = Obj::new();
        let (ty, body): (&str, Vec<(&str, Json)>) = match &it.ent {
            Ent::Line(a, b) => {
                let (a, b) = (pid(*a, &mut points, &mut seq), pid(*b, &mut points, &mut seq));
                ("line", vec![("p1", a.into()), ("p2", b.into())])
            }
            Ent::Circle(c, r) => {
                let c = pid(*c, &mut points, &mut seq);
                ("circle", vec![("c", c.into()), ("r", (*r).into())])
            }
            Ent::Arc(c, s, e) => {
                let c = pid(*c, &mut points, &mut seq);
                let (s, e) = (pid(*s, &mut points, &mut seq), pid(*e, &mut points, &mut seq));
                ("arc", vec![("c", c.into()), ("p1", s.into()), ("p2", e.into())])
            }
            Ent::Ellipse(c, p, r) => {
                let c = pid(*c, &mut points, &mut seq);
                let p = pid(*p, &mut points, &mut seq);
                ("ellipse", vec![("c", c.into()), ("p", p.into()), ("r", (*r).into())])
            }
            Ent::Spline(ps) => {
                let ids: Vec<Json> = ps.iter().map(|&p| Json::Str(pid(p, &mut points, &mut seq))).collect();
                ("spline", vec![("pts", Json::Arr(ids))])
            }
        };
        let id = format!("e{seq}");
        seq += 1;
        o = o.put("id", id.as_str()).put("type", ty);
        for (k, v) in body {
            o = o.put(k, v);
        }
        o = o.put("from", it.from);
        ents.push(o.build());
        ids.push(id);
    }
    kv[ip].1 = Json::Arr(points);
    kv[ie].1 = Json::Arr(ents);
    kv[is].1 = Json::Num(seq as f64);
    let _ = num_pair;
    (Json::Obj(kv), ids)
}

/// `{frame}` of the request.
pub fn frame_of(j: &Json) -> Option<Frame> {
    let v = |k: &str| j.get(k).and_then(Json::as_vec3).map(Vec3::from_array);
    Some(Frame { o: v("o")?, x: v("x")?, y: v("y")?, z: v("z")? })
}

/// The `convert` reply for body-resolved refs (edge / face).
pub fn reply(body: &Body, msg: &Json) -> Json {
    let (Some(r), Some(f)) = (msg.get("ref").and_then(|r| resolve_ref(body, r)), msg.get("frame").and_then(frame_of)) else {
        return Json::Null;
    };
    let (items, skipped) = project_ref(body, r, &f);
    finish(msg.get("sketch").unwrap_or(&Json::Null), items, skipped)
}

/// Assemble `{sketch, ids, skipped}`.
pub fn finish(sketch: &Json, items: Vec<Item>, skipped: usize) -> Json {
    let (sk, ids) = apply(sketch, items);
    Obj::new()
        .put("sketch", sk)
        .put("ids", Json::Arr(ids.into_iter().map(Json::Str).collect()))
        .put("skipped", skipped)
        .build()
}
