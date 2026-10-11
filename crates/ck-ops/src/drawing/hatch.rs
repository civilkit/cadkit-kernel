//! Section hatch: 45° lines (`u−v = m·spacing·√2`) clipped to the projected
//! `SEC.*` faces. Pinned by the box section probe: descending `m`, endpoints
//! exactly on the section outline, `k:"h"`, no `e` index.
use ck_core::linalg::Vec3;
use ck_json::{Json, Obj};
use ck_topo::body::{Body, FaceId};

pub fn hatch_lines(body: &Body, faces: &[FaceId], x: Vec3, y: Vec3, _z: Vec3, h: f64) -> Vec<Json> {
    if !(h > 0.0) {
        return vec![];
    }
    // projected polygons of the section faces (loops sampled; cut faces are
    // planar so this is exact)
    let mut polys: Vec<Vec<[f64; 2]>> = vec![];
    for &f in faces {
        for &lp in &body.face(f).loops {
            let mut poly = vec![];
            for &c in body.lp(lp).coedges.iter() {
                let co = body.coedge(c);
                if !co.alive {
                    continue;
                }
                let e = body.edge(co.edge);
                let n = if e.curve.kind() == "line" { 1 } else { 24 };
                for i in 0..n {
                    let t = e.t0 + (e.t1 - e.t0) * i as f64 / n as f64;
                    let p = e.curve.eval(t);
                    poly.push([p.dot(x), p.dot(y)]);
                }
            }
            if poly.len() >= 3 {
                polys.push(poly);
            }
        }
    }
    if polys.is_empty() {
        return vec![];
    }
    let d = h * 2.0f64.sqrt();
    let (mut lo, mut hi) = (f64::INFINITY, f64::NEG_INFINITY);
    for p in &polys {
        for q in p {
            let c = q[0] - q[1];
            lo = lo.min(c);
            hi = hi.max(c);
        }
    }
    let m0 = (lo / d).floor() as i64 - 1;
    let m1 = (hi / d).ceil() as i64 + 1;
    let mut out = vec![];
    // pinned: descending m
    for m in (m0..=m1).rev() {
        let c = m as f64 * d;
        let mut ts: Vec<f64> = vec![];
        for p in &polys {
            let n = p.len();
            for i in 0..n {
                let (a, b) = (p[i], p[(i + 1) % n]);
                let f0 = a[0] - a[1] - c;
                let f1 = b[0] - b[1] - c;
                if (f0 > 0.0) == (f1 > 0.0) || f0 == f1 {
                    continue;
                }
                let t = f0 / (f0 - f1);
                if !(0.0..=1.0).contains(&t) {
                    continue;
                }
                ts.push(a[0] + (b[0] - a[0]) * t);
            }
        }
        if ts.len() < 2 {
            continue;
        }
        ts.sort_by(|a, b| a.partial_cmp(b).unwrap());
        ts.dedup_by(|a, b| (*a - *b).abs() < 1e-12);
        if ts.len() % 2 == 1 {
            ts.pop(); // GUESS: drop an unpaired crossing
        }
        for pair in ts.chunks(2) {
            if pair.len() < 2 {
                break;
            }
            let (u0, u1) = (pair[0], pair[1]);
            if u1 - u0 < 1e-9 {
                continue;
            }
            let um = (u0 + u1) * 0.5;
            let vm = um - c;
            let mut inside = false;
            for p in &polys {
                if winding(p, [um, vm]) {
                    inside = !inside;
                }
            }
            if !inside {
                continue;
            }
            out.push(
                Obj::new()
                    .put("k", Json::Str("h".into()))
                    .put("v", Json::Bool(true))
                    .put("p", Json::Arr(vec![Json::Num(u0), Json::Num(u0 - c), Json::Num(u1), Json::Num(u1 - c)]))
                    .build(),
            );
        }
    }
    out
}

fn winding(poly: &[[f64; 2]], p: [f64; 2]) -> bool {
    let mut inside = false;
    let n = poly.len();
    let mut j = n - 1;
    for i in 0..n {
        let (a, b) = (poly[j], poly[i]);
        if (a[1] > p[1]) != (b[1] > p[1]) && p[0] < (b[0] - a[0]) * (p[1] - a[1]) / (b[1] - a[1]) + a[0] {
            inside = !inside;
        }
        j = i;
    }
    inside
}
