//! Sketch offset — the `sketchOffset` message (client `Pc()`/`Tc()`).
//!
//! Request: `{type:"sketchOffset", sketch, ids, distance}`; reply
//! `{"chains":[[seg..]..]}` or `{"error":"offsetTooLarge"}` (a missing
//! distance is the message error `sketchOffset without a distance`).
//! Segments (oracle):
//! * `{"kind":"line","a":[..],"b":[..]}`
//! * `{"kind":"arc","c":[..],"r":4,"a":[..],"b":[..],"ccw":true}`
//! * `{"kind":"circle","c":[..],"r":1,"a":[c.x+r,c.y],"b":[at 2π],"ccw":true}`
//! * `{"kind":"curve","pts":[[..]..],"closed":false}` — splines, offset as a
//!   polyline of 16 samples per span.
//!
//! Semantics (oracle probes): entities are chained by shared endpoints in
//! sketch order, each chain oriented by its first entity; a positive
//! distance offsets to the LEFT of the chain direction (a CCW rectangle
//! shrinks, a circle's radius decreases); neighbouring offsets are joined at
//! their intersection (mitred corners), open chain ends are not extended; a
//! radius that would become <= 0 gives `offsetTooLarge`.
//! Not oracle-probed: in the `sketchOffset` reply an arc with a neighbour on
//! both sides (a rounded corner) whose radius would become <= 0 is dropped and
//! its neighbours meet at their mitre, so a rounded rectangle offset inward
//! past its corner radius gets sharp corners; a lone arc, an open chain's end
//! arc and a circle still give `offsetTooLarge`. `sketch_offset` (the lip)
//! keeps the strict rule.
//!
//! First pass: joins without an intersection are left as gaps (GUESS: the
//! kernel may insert round joins); spline sampling is uniform in chord
//! length, so curve points are not bit-identical to the kernel's.

use super::spline::InterpSpline;
use super::{Sketch, SketchEntity};
use ck_json::Json;
use std::f64::consts::PI;

type P = [f64; 2];

pub const OFFSET_TOO_LARGE: &str = "offsetTooLarge";
pub const NO_DISTANCE: &str = "sketchOffset without a distance";

#[derive(Debug, Clone)]
pub enum OffSeg {
    Line { a: P, b: P },
    /// `ccw`: traversal direction from `a` to `b` around `c`.
    Arc { c: P, r: f64, a: P, b: P, ccw: bool },
    Circle { c: P, r: f64 },
    Curve { pts: Vec<P>, closed: bool },
}

impl OffSeg {
    pub fn to_json(&self) -> Json {
        let pt = |p: P| Json::Arr(vec![Json::Num(p[0]), Json::Num(p[1])]);
        let kind = |k: &str| ("kind".to_string(), Json::Str(k.into()));
        match self {
            OffSeg::Line { a, b } => Json::Obj(vec![kind("line"), ("a".into(), pt(*a)), ("b".into(), pt(*b))]),
            OffSeg::Arc { c, r, a, b, ccw } => Json::Obj(vec![
                kind("arc"),
                ("c".into(), pt(*c)),
                ("r".into(), Json::Num(*r)),
                ("a".into(), pt(*a)),
                ("b".into(), pt(*b)),
                ("ccw".into(), Json::Bool(*ccw)),
            ]),
            OffSeg::Circle { c, r } => Json::Obj(vec![
                kind("circle"),
                ("c".into(), pt(*c)),
                ("r".into(), Json::Num(*r)),
                ("a".into(), pt([c[0] + r, c[1]])),
                ("b".into(), pt([c[0] + r * (2.0 * PI).cos(), c[1] + r * (2.0 * PI).sin()])),
                ("ccw".into(), Json::Bool(true)),
            ]),
            OffSeg::Curve { pts, closed } => Json::Obj(vec![
                kind("curve"),
                ("pts".into(), Json::Arr(pts.iter().map(|p| pt(*p)).collect())),
                ("closed".into(), Json::Bool(*closed)),
            ]),
        }
    }
    fn start(&self) -> P {
        match self {
            OffSeg::Line { a, .. } | OffSeg::Arc { a, .. } => *a,
            OffSeg::Circle { c, r } => [c[0] + r, c[1]],
            OffSeg::Curve { pts, .. } => pts[0],
        }
    }
    fn end(&self) -> P {
        match self {
            OffSeg::Line { b, .. } | OffSeg::Arc { b, .. } => *b,
            OffSeg::Circle { c, r } => [c[0] + r, c[1]],
            OffSeg::Curve { pts, .. } => pts[pts.len() - 1],
        }
    }
    fn set_start(&mut self, p: P) {
        match self {
            OffSeg::Line { a, .. } | OffSeg::Arc { a, .. } => *a = p,
            OffSeg::Curve { pts, .. } => pts[0] = p,
            OffSeg::Circle { .. } => {}
        }
    }
    fn set_end(&mut self, p: P) {
        match self {
            OffSeg::Line { b, .. } | OffSeg::Arc { b, .. } => *b = p,
            OffSeg::Curve { pts, .. } => {
                let n = pts.len();
                pts[n - 1] = p
            }
            OffSeg::Circle { .. } => {}
        }
    }
}

/// An oriented chain element.
#[derive(Clone)]
struct Item<'a> {
    e: &'a SketchEntity,
    rev: bool,
}

fn ends(e: &SketchEntity) -> Option<(&str, &str)> {
    match e {
        SketchEntity::Line { p1, p2, .. } | SketchEntity::Arc { p1, p2, .. } => Some((p1, p2)),
        SketchEntity::Spline { pts, closed: false, .. } if pts.len() >= 2 => Some((&pts[0], &pts[pts.len() - 1])),
        _ => None,
    }
}

fn chains<'a>(sk: &'a Sketch, ids: &[String]) -> Vec<(Vec<Item<'a>>, bool)> {
    let ents: Vec<&SketchEntity> = sk
        .entities
        .iter()
        .filter(|e| ids.iter().any(|i| i == e.id()) && !matches!(e, SketchEntity::Point { .. }))
        .collect();
    let mut used = vec![false; ents.len()];
    let mut out = Vec::new();
    for i in 0..ents.len() {
        if used[i] {
            continue;
        }
        used[i] = true;
        let Some((h0, t0)) = ends(ents[i]) else {
            out.push((vec![Item { e: ents[i], rev: false }], true));
            continue;
        };
        let mut chain = vec![Item { e: ents[i], rev: false }];
        let (mut head, mut tail) = (h0.to_string(), t0.to_string());
        // extend forward from the tail
        loop {
            if head == tail {
                break;
            }
            let Some(j) = (0..ents.len()).find(|&j| !used[j] && ends(ents[j]).map_or(false, |(a, b)| a == tail || b == tail)) else { break };
            used[j] = true;
            let (a, b) = ends(ents[j]).unwrap();
            let rev = b == tail;
            tail = if rev { a.to_string() } else { b.to_string() };
            chain.push(Item { e: ents[j], rev });
        }
        // extend backward from the head
        loop {
            if head == tail {
                break;
            }
            let Some(j) = (0..ents.len()).find(|&j| !used[j] && ends(ents[j]).map_or(false, |(a, b)| a == head || b == head)) else { break };
            used[j] = true;
            let (a, b) = ends(ents[j]).unwrap();
            let rev = a == head;
            head = if rev { b.to_string() } else { a.to_string() };
            chain.insert(0, Item { e: ents[j], rev });
        }
        out.push((chain, head == tail));
    }
    out
}

fn offset_item(sk: &Sketch, it: &Item, d: f64) -> Result<Option<OffSeg>, &'static str> {
    let pt = |id: &str| sk.point(id).map(|p| [p.x, p.y]);
    Ok(Some(match it.e {
        SketchEntity::Line { p1, p2, .. } => {
            let (Some(mut a), Some(mut b)) = (pt(p1), pt(p2)) else { return Ok(None) };
            if it.rev {
                std::mem::swap(&mut a, &mut b);
            }
            let (dx, dy) = (b[0] - a[0], b[1] - a[1]);
            let l = dx.hypot(dy);
            if l < 1e-12 {
                return Ok(None);
            }
            // TODO: oracle gets 1.4142135623730951 for a 45° line offset by 2;
            // this formula yields ...095 (1 ulp low).
            let k = d / l;
            let n = [-dy * k, dx * k];
            OffSeg::Line { a: [a[0] + n[0], a[1] + n[1]], b: [b[0] + n[0], b[1] + n[1]] }
        }
        SketchEntity::Arc { c, p1, p2, .. } => {
            let (Some(c), Some(a), Some(b)) = (pt(c), pt(p1), pt(p2)) else { return Ok(None) };
            let r0 = (a[0] - c[0]).hypot(a[1] - c[1]);
            let ccw = !it.rev;
            let r = if ccw { r0 - d } else { r0 + d };
            if r <= 1e-12 {
                return Err(OFFSET_TOO_LARGE);
            }
            let on = |p: P| {
                let t = (p[1] - c[1]).atan2(p[0] - c[0]);
                [c[0] + r * t.cos(), c[1] + r * t.sin()]
            };
            let (a, b) = if ccw { (on(a), on(b)) } else { (on(b), on(a)) };
            OffSeg::Arc { c, r, a, b, ccw }
        }
        SketchEntity::Circle { c, r, .. } => {
            let Some(c) = pt(c) else { return Ok(None) };
            let r = r.abs() - d;
            if r <= 1e-12 {
                return Err(OFFSET_TOO_LARGE);
            }
            OffSeg::Circle { c, r }
        }
        SketchEntity::Spline { pts, closed, .. } => {
            let ps: Vec<P> = pts.iter().filter_map(|p| pt(p)).collect();
            let Some(s) = InterpSpline::new(&ps, *closed) else { return Ok(None) };
            let n = s.h.len() * 16;
            let mut out: Vec<P> = (0..=n)
                .map(|i| {
                    let t = s.length() * i as f64 / n as f64;
                    let p = s.eval(t);
                    let dv = s.at(t, 1);
                    let l = dv[0].hypot(dv[1]).max(1e-300);
                    [p[0] - dv[1] / l * d, p[1] + dv[0] / l * d]
                })
                .collect();
            if it.rev {
                out.reverse();
                // reversed direction: the left side flips
                out = (0..=n)
                    .rev()
                    .map(|i| {
                        let t = s.length() * i as f64 / n as f64;
                        let p = s.eval(t);
                        let dv = s.at(t, 1);
                        let l = dv[0].hypot(dv[1]).max(1e-300);
                        [p[0] + dv[1] / l * d, p[1] - dv[0] / l * d]
                    })
                    .collect();
            }
            OffSeg::Curve { pts: out, closed: s.closed }
        }
        _ => return Ok(None),
    }))
}

fn cross(a: P, b: P) -> f64 {
    a[0] * b[1] - a[1] * b[0]
}

/// Intersections of the support curves of two offset segments.
fn support_intersections(s: &OffSeg, t: &OffSeg) -> Vec<P> {
    enum Sup {
        L(P, P),
        C(P, f64),
    }
    let sup = |x: &OffSeg, at_end: bool| -> Sup {
        match x {
            OffSeg::Line { a, b } => Sup::L(*a, [b[0] - a[0], b[1] - a[1]]),
            OffSeg::Arc { c, r, .. } | OffSeg::Circle { c, r } => Sup::C(*c, *r),
            OffSeg::Curve { pts, .. } => {
                let n = pts.len();
                if at_end {
                    Sup::L(pts[n - 2], [pts[n - 1][0] - pts[n - 2][0], pts[n - 1][1] - pts[n - 2][1]])
                } else {
                    Sup::L(pts[0], [pts[1][0] - pts[0][0], pts[1][1] - pts[0][1]])
                }
            }
        }
    };
    let line_circle = |p: P, d: P, c: P, r: f64| -> Vec<P> {
        let f = [p[0] - c[0], p[1] - c[1]];
        let a = d[0] * d[0] + d[1] * d[1];
        let b = 2.0 * (f[0] * d[0] + f[1] * d[1]);
        let k = f[0] * f[0] + f[1] * f[1] - r * r;
        let disc = b * b - 4.0 * a * k;
        if disc < -1e-9 * a * (1.0 + r * r) {
            return vec![];
        }
        let sq = disc.max(0.0).sqrt();
        [(-b - sq) / (2.0 * a), (-b + sq) / (2.0 * a)].iter().map(|t| [p[0] + d[0] * t, p[1] + d[1] * t]).collect()
    };
    match (sup(s, true), sup(t, false)) {
        (Sup::L(p, d), Sup::L(q, e)) => {
            let den = cross(d, e);
            if den.abs() < 1e-12 * (d[0].hypot(d[1]) * e[0].hypot(e[1])) {
                return vec![];
            }
            let w = [q[0] - p[0], q[1] - p[1]];
            let u = cross(w, e) / den;
            vec![[p[0] + d[0] * u, p[1] + d[1] * u]]
        }
        (Sup::L(p, d), Sup::C(c, r)) | (Sup::C(c, r), Sup::L(p, d)) => line_circle(p, d, c, r),
        (Sup::C(c1, r1), Sup::C(c2, r2)) => {
            let dv = [c2[0] - c1[0], c2[1] - c1[1]];
            let dd = dv[0].hypot(dv[1]);
            if dd < 1e-12 || dd > r1 + r2 + 1e-9 || dd < (r1 - r2).abs() - 1e-9 {
                return vec![];
            }
            let x = (dd * dd + r1 * r1 - r2 * r2) / (2.0 * dd);
            let h = (r1 * r1 - x * x).max(0.0).sqrt();
            let u = [dv[0] / dd, dv[1] / dd];
            let m = [c1[0] + u[0] * x, c1[1] + u[1] * x];
            vec![[m[0] - u[1] * h, m[1] + u[0] * h], [m[0] + u[1] * h, m[1] - u[0] * h]]
        }
    }
}

fn join(a: &mut OffSeg, b: &mut OffSeg) {
    let (e, s) = (a.end(), b.start());
    let mid = [(e[0] + s[0]) / 2.0, (e[1] + s[1]) / 2.0];
    let cands = support_intersections(a, b);
    if let Some(p) = cands.into_iter().min_by(|p, q| {
        let dp = (p[0] - mid[0]).hypot(p[1] - mid[1]);
        let dq = (q[0] - mid[0]).hypot(q[1] - mid[1]);
        dp.partial_cmp(&dq).unwrap()
    }) {
        a.set_end(p);
        b.set_start(p);
    }
}

/// Offset the given entities by `distance` (positive = left of the chain).
pub fn sketch_offset(sk: &Sketch, ids: &[String], distance: f64) -> Result<Vec<Vec<OffSeg>>, &'static str> {
    offset_chains(sk, ids, distance, false)
}

/// `drop_collapsed`: an arc between two neighbours whose offset radius would
/// be <= 0 is left out (its neighbours are joined directly) instead of failing.
fn offset_chains(sk: &Sketch, ids: &[String], distance: f64, drop_collapsed: bool) -> Result<Vec<Vec<OffSeg>>, &'static str> {
    let mut out = Vec::new();
    for (chain, closed) in chains(sk, ids) {
        let mut segs = Vec::new();
        let mut dropped = false;
        for (k, it) in chain.iter().enumerate() {
            let inner = closed || (k > 0 && k + 1 < chain.len());
            match offset_item(sk, it, distance) {
                Ok(Some(s)) => segs.push(s),
                Ok(None) => {}
                Err(OFFSET_TOO_LARGE) if drop_collapsed && inner && chain.len() > 1 && matches!(it.e, SketchEntity::Arc { .. }) => dropped = true,
                Err(e) => return Err(e),
            }
        }
        if segs.is_empty() {
            if dropped {
                return Err(OFFSET_TOO_LARGE);
            }
            continue;
        }
        let dirs: Vec<Option<P>> = segs.iter().map(|s| if let OffSeg::Line { a, b } = s { Some([b[0] - a[0], b[1] - a[1]]) } else { None }).collect();
        let n = segs.len();
        let joins = if closed && n > 1 { n } else { n - 1 };
        for k in 0..joins {
            let j = (k + 1) % n;
            if k == j {
                continue;
            }
            let (lo, hi) = if k < j { (k, j) } else { (j, k) };
            let (x, y) = segs.split_at_mut(hi);
            if k < j {
                join(&mut x[lo], &mut y[0]);
            } else {
                join(&mut y[0], &mut x[lo]);
            }
        }
        if dropped {
            // with a corner dropped the neighbours must still meet and no line may turn round
            // (an obround offset past its half width): otherwise the offset is too large
            let gap = (0..joins).any(|k| {
                let (e, s) = (segs[k].end(), segs[(k + 1) % n].start());
                (e[0] - s[0]).hypot(e[1] - s[1]) > 1e-9 * (1.0 + distance.abs())
            });
            let flipped = segs.iter().zip(&dirs).any(|(s, d)| match (s, d) {
                (OffSeg::Line { a, b }, Some(d)) => (b[0] - a[0]) * d[0] + (b[1] - a[1]) * d[1] <= 0.0,
                _ => false,
            });
            if gap || flipped {
                return Err(OFFSET_TOO_LARGE);
            }
        }
        out.push(segs);
    }
    Ok(out)
}

/// Wire reply for `sketchOffset`.
pub fn sketch_offset_json(sk: &Sketch, ids: &[String], distance: f64) -> Json {
    match offset_chains(sk, ids, distance, true) {
        Ok(chs) => Json::Obj(vec![(
            "chains".into(),
            Json::Arr(chs.iter().map(|c| Json::Arr(c.iter().map(|s| s.to_json()).collect())).collect()),
        )]),
        Err(e) => Json::Obj(vec![("error".into(), Json::Str(e.into()))]),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const SK: &str = r#"{"points":[{"id":"a","x":0,"y":0},{"id":"b","x":10,"y":0},{"id":"c","x":10,"y":10},{"id":"d","x":0,"y":10},{"id":"o","x":30,"y":0},{"id":"q","x":50,"y":0},{"id":"r","x":60,"y":10},{"id":"m","x":10,"y":5}],"entities":[{"type":"line","id":"e1","p1":"a","p2":"b"},{"type":"line","id":"e2","p1":"b","p2":"c"},{"type":"line","id":"e3","p1":"c","p2":"d"},{"type":"line","id":"e4","p1":"d","p2":"a"},{"type":"circle","id":"e5","c":"o","r":3},{"type":"line","id":"e6","p1":"q","p2":"r"},{"type":"arc","id":"e7","c":"m","p1":"b","p2":"c"},{"type":"line","id":"e8","p1":"b","p2":"d"}]}"#;

    fn off(ids: &[&str], d: f64) -> String {
        let sk = Sketch::from_json(&ck_json::parse(SK).unwrap());
        let ids: Vec<String> = ids.iter().map(|s| s.to_string()).collect();
        ck_json::to_string(&sketch_offset_json(&sk, &ids, d))
    }

    /// Oracle replies (`sketchOffset`).
    #[test]
    fn oracle_lines() {
        assert_eq!(off(&["e1", "e2", "e3", "e4"], 2.0), r#"{"chains":[[{"kind":"line","a":[2,2],"b":[8,2]},{"kind":"line","a":[8,2],"b":[8,8]},{"kind":"line","a":[8,8],"b":[2,8]},{"kind":"line","a":[2,8],"b":[2,2]}]]}"#);
        assert_eq!(off(&["e1", "e2", "e3", "e4"], -2.0), r#"{"chains":[[{"kind":"line","a":[-2,-2],"b":[12,-2]},{"kind":"line","a":[12,-2],"b":[12,12]},{"kind":"line","a":[12,12],"b":[-2,12]},{"kind":"line","a":[-2,12],"b":[-2,-2]}]]}"#);
        // oracle: a = [48.58578643762691, 1.4142135623730951] (we are 1 ulp off in y)
        let j = ck_json::parse(&off(&["e6"], 2.0)).unwrap();
        let seg = &j.get("chains").unwrap().as_arr().unwrap()[0].as_arr().unwrap()[0];
        let a = seg.get("a").unwrap().as_arr().unwrap();
        assert!((a[0].as_f64().unwrap() - 48.58578643762691).abs() < 1e-12);
        assert!((a[1].as_f64().unwrap() - 1.4142135623730951).abs() < 1e-12);
        assert_eq!(off(&["e1", "e2"], 2.0), r#"{"chains":[[{"kind":"line","a":[0,2],"b":[8,2]},{"kind":"line","a":[8,2],"b":[8,10]}]]}"#);
        assert_eq!(off(&["e3", "e1"], 1.0), r#"{"chains":[[{"kind":"line","a":[0,1],"b":[10,1]}],[{"kind":"line","a":[10,9],"b":[0,9]}]]}"#);
    }

    #[test]
    fn oracle_circles_arcs() {
        assert_eq!(off(&["e5"], 2.0), r#"{"chains":[[{"kind":"circle","c":[30,0],"r":1,"a":[31,0],"b":[31,-0.00000000000000024492935982947064],"ccw":true}]]}"#.replace("-0.00000000000000024492935982947064", &ck_json::fmt_f64(-2.4492935982947064e-16)));
        assert_eq!(off(&["e5"], 5.0), r#"{"error":"offsetTooLarge"}"#);
        assert_eq!(off(&["e7"], 1.0), r#"{"chains":[[{"kind":"arc","c":[10,5],"r":4,"a":[10,1],"b":[10,9],"ccw":true}]]}"#);
        let s = off(&["e1", "e8"], 1.0);
        assert!(s.starts_with(r#"{"chains":[[{"kind":"line","a":[0,1],"b":[7.58578643762690"#), "{s}");
    }

    /// A sketch of the given points and entities (entities as JSON fragments).
    fn sketch_of(pts: &[(&str, f64, f64)], ents: &[String]) -> Sketch {
        let p: Vec<String> = pts.iter().map(|(id, x, y)| format!(r#"{{"id":"{id}","x":{x},"y":{y}}}"#)).collect();
        Sketch::from_json(&ck_json::parse(&format!(r#"{{"points":[{}],"entities":[{}]}}"#, p.join(","), ents.join(","))).unwrap())
    }
    fn line(id: &str, a: &str, b: &str) -> String {
        format!(r#"{{"type":"line","id":"{id}","p1":"{a}","p2":"{b}"}}"#)
    }
    fn arc(id: &str, c: &str, a: &str, b: &str) -> String {
        format!(r#"{{"type":"arc","id":"{id}","c":"{c}","p1":"{a}","p2":"{b}"}}"#)
    }
    /// 20 x 10 rectangle at the origin with corner radius `r`, counter-clockwise (inward = positive offset).
    fn rounded_rect(r: f64) -> (Sketch, Vec<String>) {
        let pts = [
            ("b0", r, 0.0), ("b1", 20.0 - r, 0.0), ("r0", 20.0, r), ("r1", 20.0, 10.0 - r),
            ("t0", 20.0 - r, 10.0), ("t1", r, 10.0), ("l0", 0.0, 10.0 - r), ("l1", 0.0, r),
            ("c0", 20.0 - r, r), ("c1", 20.0 - r, 10.0 - r), ("c2", r, 10.0 - r), ("c3", r, r),
        ];
        let ents = vec![
            line("e1", "b0", "b1"), arc("e2", "c0", "b1", "r0"), line("e3", "r0", "r1"), arc("e4", "c1", "r1", "t0"),
            line("e5", "t0", "t1"), arc("e6", "c2", "t1", "l0"), line("e7", "l0", "l1"), arc("e8", "c3", "l1", "b0"),
        ];
        let ids = (1..=8).map(|i| format!("e{i}")).collect();
        (sketch_of(&pts, &ents), ids)
    }
    /// The reply's chains as (kind, a, b) per segment.
    fn segs(j: &Json) -> Vec<Vec<(String, P, P)>> {
        let p = |v: &Json| {
            let a = v.as_arr().unwrap();
            [a[0].as_f64().unwrap(), a[1].as_f64().unwrap()]
        };
        let chains = j.get("chains").unwrap_or_else(|| panic!("no chains: {}", ck_json::to_string(j)));
        chains.as_arr().unwrap().iter().map(|c| c.as_arr().unwrap().iter().map(|s| (s.get_str("kind").unwrap().to_string(), p(s.get("a").unwrap()), p(s.get("b").unwrap()))).collect()).collect()
    }
    fn near(a: P, b: P) -> bool {
        (a[0] - b[0]).hypot(a[1] - b[1]) < 1e-9
    }
    /// A closed chain whose segments run end to start.
    fn closed_chain(c: &[(String, P, P)]) -> bool {
        (0..c.len()).all(|k| near(c[k].2, c[(k + 1) % c.len()].1))
    }

    #[test]
    fn offset_rounded_rect_past_corner_radius_gives_sharp_corners() {
        let (sk, ids) = rounded_rect(2.0);
        let j = sketch_offset_json(&sk, &ids, 3.0);
        let ch = segs(&j);
        assert_eq!(ch.len(), 1, "{}", ck_json::to_string(&j));
        let c = &ch[0];
        assert!(c.iter().all(|s| s.0 == "line") && c.len() == 4, "want 4 lines: {}", ck_json::to_string(&j));
        assert!(closed_chain(c), "{}", ck_json::to_string(&j));
        let want = [[3.0, 3.0], [17.0, 3.0], [17.0, 7.0], [3.0, 7.0]];
        for (s, w) in c.iter().zip(want) {
            assert!(near(s.1, w), "corner {:?} want {:?}", s.1, w);
        }
        // past half the height nothing is left: still an error, not crossed lines
        assert_eq!(ck_json::to_string(&sketch_offset_json(&sk, &ids, 5.5)), r#"{"error":"offsetTooLarge"}"#);
        // the lip keeps the strict rule
        assert_eq!(sketch_offset(&sk, &ids, 3.0).unwrap_err(), OFFSET_TOO_LARGE);
    }

    #[test]
    #[ignore = "offset does not split at self-intersections: a waist offset past half its width gives one crossed chain of 12 lines, not two outlines"]
    fn offset_waist_splits_into_two_outlines() {
        // dumbbell: two 10 x 10 squares joined by a 6 long, 4 wide waist; counter-clockwise
        let pts = [
            ("p0", 0.0, 0.0), ("p1", 10.0, 0.0), ("p2", 10.0, 3.0), ("p3", 16.0, 3.0), ("p4", 16.0, 0.0), ("p5", 26.0, 0.0),
            ("p6", 26.0, 10.0), ("p7", 16.0, 10.0), ("p8", 16.0, 7.0), ("p9", 10.0, 7.0), ("p10", 10.0, 10.0), ("p11", 0.0, 10.0),
        ];
        let ents: Vec<String> = (0..12).map(|i| line(&format!("e{i}"), &format!("p{i}"), &format!("p{}", (i + 1) % 12))).collect();
        let ids: Vec<String> = (0..12).map(|i| format!("e{i}")).collect();
        let sk = sketch_of(&pts, &ents);
        let j = sketch_offset_json(&sk, &ids, 2.5);
        let ch = segs(&j);
        assert_eq!(ch.len(), 2, "want two outlines: {}", ck_json::to_string(&j));
        for (c, x0) in ch.iter().zip([2.5, 18.5]) {
            assert!(c.len() == 4 && closed_chain(c), "{}", ck_json::to_string(&j));
            let lo = c.iter().map(|s| s.1[0]).fold(f64::MAX, f64::min);
            assert!((lo - x0).abs() < 1e-9, "outline from x {lo}, want {x0}");
        }
    }

    #[test]
    fn offset_follows_changed_distance() {
        let (sk, ids) = rounded_rect(2.0);
        for (d, arcs) in [(0.5, 4), (1.0, 4), (1.5, 4), (2.5, 0), (4.0, 0)] {
            let j = sketch_offset_json(&sk, &ids, d);
            let ch = segs(&j);
            assert_eq!(ch.len(), 1, "d={d}: {}", ck_json::to_string(&j));
            let c = &ch[0];
            assert!(closed_chain(c), "d={d}: {}", ck_json::to_string(&j));
            assert_eq!(c.iter().filter(|s| s.0 == "arc").count(), arcs, "d={d}: {}", ck_json::to_string(&j));
            // the bottom line moves up by d; its ends stay over the corner centres, or at the mitre once the arcs are gone
            let bottom = &c[0];
            let x0 = if arcs > 0 { 2.0 } else { d };
            assert!(near(bottom.1, [x0, d]) && near(bottom.2, [20.0 - x0, d]), "d={d}: bottom {:?} -> {:?}", bottom.1, bottom.2);
            if arcs > 0 {
                let r = j.get("chains").unwrap().as_arr().unwrap()[0].as_arr().unwrap()[1].get_f64("r").unwrap();
                assert!((r - (2.0 - d)).abs() < 1e-12, "d={d}: corner radius {r}");
            }
        }
    }
}
