// Adapted from brepkit v2.129.15 (MIT OR Apache-2.0), crates/check/src/validate/wire.rs
//! Loop closure / chaining and uv self-intersection of planar faces.

use crate::{face_ent, Issue};
use ck_geom::surface::Surface;
use ck_geom::Vec2;
use ck_topo::body::Body;

pub fn check(b: &Body, tol: f64, crossings: bool, ctl: &crate::Ctl, out: &mut Vec<Issue>) {
    for f in b.face_ids() {
        if ctl.expired() {
            return;
        }
        let face = b.face(f);
        let planar = matches!(face.surface, Surface::Plane { .. });
        if face.loops.is_empty() {
            // A closed analytic face (full sphere/torus) may legitimately have none.
            if planar {
                out.push(Issue { code: "FACE_NO_LOOP", entity: face_ent(b, f), detail: "planar face without a loop".into() });
            }
            continue;
        }
        for &l in &face.loops {
            if !b.lp(l).alive {
                continue;
            }
            let cs: &Vec<_> = &b.lp(l).coedges.iter().copied().filter(|&c| b.coedge(c).alive).collect();
            if cs.is_empty() {
                out.push(Issue { code: "LOOP_EMPTY", entity: face_ent(b, f), detail: "loop has no edges".into() });
                continue;
            }
            for i in 0..cs.len() {
                let (c, n) = (cs[i], cs[(i + 1) % cs.len()]);
                let (ve, vs) = (b.coedge_end(c), b.coedge_start(n));
                if ve != vs && b.vertex(ve).p.dist(b.vertex(vs).p) > tol.max(b.vertex(ve).tol).max(b.vertex(vs).tol) {
                    out.push(Issue {
                        code: "LOOP_OPEN",
                        entity: face_ent(b, f),
                        detail: format!("coedge {i} end != coedge {} start", (i + 1) % cs.len()),
                    });
                }
            }
        }
        if planar && crossings {
            self_intersection(b, f, ctl, out);
        }
    }
}

/// Adaptive uv polyline of one coedge: subdivide (min 2 levels, max 12) until
/// the parametric midpoint lies within `sag` of the chord. Pushes the start
/// point and interior points, not the end point (the next coedge's start).
#[allow(clippy::too_many_arguments)]
fn sample(b: &Body, c: ck_topo::body::CoedgeId, t0: f64, p0: Vec2, t1: f64, p1: Vec2, depth: u32, sag: f64, out: &mut Vec<Vec2>) {
    let tm = 0.5 * (t0 + t1);
    let pm = b.coedge_uv(c, tm, Some(p0));
    let d = p1 - p0;
    let l = d.len();
    let dev = if l < 1e-14 { (pm - p0).len() } else { d.cross(pm - p0).abs() / l };
    // a midpoint beyond the chord ends (cusp / overshoot) also needs refining
    let along = if l < 1e-14 { 0.0 } else { (pm - p0).dot(d) / (l * l) };
    let flat = dev <= sag && (0.0..=1.0).contains(&along);
    if depth >= 12 || (depth >= 2 && flat) {
        out.push(p0);
        out.push(pm);
        return;
    }
    sample(b, c, t0, p0, tm, pm, depth + 1, sag, out);
    sample(b, c, tm, pm, t1, p1, depth + 1, sag, out);
}

/// Polylines (uv) of each loop of a planar face. Within one loop, report
/// crossings between non-adjacent segments (`LOOP_SELF_INTERSECT`); between
/// different loops of the face (outer vs hole) report `LOOP_CROSS` only for a
/// true transversal crossing. Sampling is adaptive (chord sag below a tiny
/// fraction of the face size) and the crossing tolerance is scaled to that
/// sag, so thin annuli / near-concentric loops are not flagged.
fn self_intersection(b: &Body, f: ck_topo::body::FaceId, ctl: &crate::Ctl, out: &mut Vec<Issue>) {
    // size of the face in uv, from the vertices (sets the sag tolerance)
    let (mut lo, mut hi) = (Vec2::new(f64::MAX, f64::MAX), Vec2::new(f64::MIN, f64::MIN));
    let mut curved = false;
    for &l in &b.face(f).loops {
        if !b.lp(l).alive {
            continue;
        }
        for &c in b.lp(l).coedges.iter().filter(|&&c| b.coedge(c).alive) {
            let (t0, t1) = b.coedge_params(c);
            for k in 0..=4 {
                let p = b.coedge_uv(c, t0 + (t1 - t0) * k as f64 / 4.0, None);
                lo = Vec2::new(lo.x.min(p.x), lo.y.min(p.y));
                hi = Vec2::new(hi.x.max(p.x), hi.y.max(p.y));
            }
            curved |= !matches!(b.edge(b.coedge(c).edge).curve, ck_geom::curve::Curve::Line { .. });
        }
    }
    let size = (hi - lo).len().max(1e-9);
    let sag = if curved { (size * 1e-5).max(1e-9) } else { f64::INFINITY };
    let eps = if curved { sag.max(1e-6) } else { 1e-6 };
    let mut polys: Vec<Vec<Vec2>> = vec![];
    for &l in &b.face(f).loops {
        if !b.lp(l).alive {
            continue;
        }
        let mut pts: Vec<Vec2> = vec![];
        let cs: Vec<_> = b.lp(l).coedges.iter().copied().filter(|&c| b.coedge(c).alive).collect();
        for &c in &cs {
            let (t0, t1) = b.coedge_params(c);
            let p0 = b.coedge_uv(c, t0, pts.last().copied());
            let p1 = b.coedge_uv(c, t1, Some(p0));
            if sag.is_infinite() {
                // straight edges: the end points are the whole polyline
                pts.push(p0);
            } else {
                sample(b, c, t0, p0, t1, p1, 0, sag, &mut pts);
            }
        }
        polys.push(pts);
    }
    if let Some((a, c, i, j)) = first_crossing(&polys, eps, ctl) {
        out.push(Issue {
            code: if a == c { "LOOP_SELF_INTERSECT" } else { "LOOP_CROSS" },
            entity: face_ent(b, f),
            detail: format!("uv loops {a}/{c} cross at segments {i}/{j}"),
        });
    }
}

/// The first crossing between the segments of `polys` (closed polylines) in the order of the plain double loop: loop `a`,
/// then loop `c >= a`, then segment `i` of `a`, then segment `j` of `c`; `None` when none cross or `ctl` ran out.
fn first_crossing(polys: &[Vec<Vec2>], eps: f64, ctl: &crate::Ctl) -> Option<(usize, usize, usize, usize)> {
    // Every segment of every loop, looked up through a uniform grid over the uv box (a plate with thousands of holes has
    // millions of segment pairs, nearly all far apart). The answer is the first crossing in the order of the plain
    // double loop: loop `a`, then loop `c >= a`, then segment `i` of `a`, then segment `j` of `c`.
    let segs: Vec<(usize, usize, Vec2, Vec2)> = polys
        .iter()
        .enumerate()
        .flat_map(|(a, p)| (0..p.len()).map(move |i| (a, i, p[i], p[(i + 1) % p.len()])))
        .collect();
    let bx = |s: &(usize, usize, Vec2, Vec2)| (s.2.x.min(s.3.x), s.2.y.min(s.3.y), s.2.x.max(s.3.x), s.2.y.max(s.3.y));
    let (mut glo, mut ghi) = (Vec2::new(f64::MAX, f64::MAX), Vec2::new(f64::MIN, f64::MIN));
    let mut ext = 0.0f64;
    let mut finite = 0usize;
    for s in &segs {
        let (x0, y0, x1, y1) = bx(s);
        if [x0, y0, x1, y1].iter().all(|v| v.is_finite()) {
            glo = Vec2::new(glo.x.min(x0), glo.y.min(y0));
            ghi = Vec2::new(ghi.x.max(x1), ghi.y.max(y1));
            ext += (x1 - x0).max(y1 - y0);
            finite += 1;
        }
    }
    let cs = if finite > 0 { (ext / finite as f64).max((ghi.x - glo.x).max(ghi.y - glo.y) / 1024.0).max(eps).max(1e-12) } else { 1.0 };
    let cell = |x: f64, y: f64| (((x - glo.x) / cs).floor() as i64, ((y - glo.y) / cs).floor() as i64);
    const BIG_SPAN: i64 = 64;
    let mut grid: ck_core::fxhash::FxHashMap<(i64, i64), Vec<u32>> = ck_core::fxhash::FxHashMap::default();
    // segments spanning many cells (a long bridge) or without a finite box are tested against everything instead
    let mut big: Vec<u32> = vec![];
    let mut is_big = vec![false; segs.len()];
    for (k, s) in segs.iter().enumerate() {
        let (x0, y0, x1, y1) = bx(s);
        let (c0, c1) = (cell(x0, y0), cell(x1, y1));
        if ![x0, y0, x1, y1].iter().all(|v| v.is_finite()) || c1.0 - c0.0 >= BIG_SPAN || c1.1 - c0.1 >= BIG_SPAN {
            big.push(k as u32);
            is_big[k] = true;
            continue;
        }
        for x in c0.0..=c1.0 {
            for y in c0.1..=c1.1 {
                grid.entry((x, y)).or_default().push(k as u32);
            }
        }
    }
    let mut stamp = vec![0u32; segs.len()];
    let mut found: Option<(usize, usize, usize, usize)> = None;
    for (k, s) in segs.iter().enumerate() {
        if ctl.poll() {
            return None;
        }
        let (a, i) = (s.0, s.1);
        let (sl, sh) = (Vec2::new(s.2.x.min(s.3.x) - eps, s.2.y.min(s.3.y) - eps), Vec2::new(s.2.x.max(s.3.x) + eps, s.2.y.max(s.3.y) + eps));
        let mut cand: Vec<u32> = vec![];
        if is_big[k] {
            cand.extend(0..segs.len() as u32);
        } else {
            let (c0, c1) = (cell(sl.x, sl.y), cell(sh.x, sh.y));
            for x in c0.0..=c1.0 {
                for y in c0.1..=c1.1 {
                    if let Some(v) = grid.get(&(x, y)) {
                        for &m in v {
                            if stamp[m as usize] != k as u32 + 1 {
                                stamp[m as usize] = k as u32 + 1;
                                cand.push(m);
                            }
                        }
                    }
                }
            }
            cand.extend(big.iter().copied());
        }
        for m in cand {
            let t = &segs[m as usize];
            let (c, j) = (t.0, t.1);
            // each pair once, with the segment of the earlier loop (or the earlier segment of one loop) first
            if c < a || (c == a && (j <= i || j == i + 1 || (i == 0 && j == polys[a].len() - 1))) {
                continue;
            }
            if t.2.x.min(t.3.x) > sh.x || t.2.x.max(t.3.x) < sl.x || t.2.y.min(t.3.y) > sh.y || t.2.y.max(t.3.y) < sl.y {
                continue;
            }
            if proper_cross(s.2, s.3, t.2, t.3, eps) && found.map_or(true, |f| (a, c, i, j) < f) {
                found = Some((a, c, i, j));
            }
        }
        // the plain loop is ordered by loop `a` first: once a later loop is reached no crossing can beat the one found
        if found.is_some_and(|f| a > f.0) {
            break;
        }
    }
    found
}

/// True when the segments cross strictly inside both. Endpoints that touch
/// the other segment (shared vertices, keyhole bridges that run back over each
/// other, pinch points where a loop revisits a vertex) are not crossings: the
/// test is on signed *distances* so float noise at a shared vertex cannot
/// flip a sign.
fn proper_cross(p: Vec2, q: Vec2, r: Vec2, s: Vec2, eps: f64) -> bool {
    let dist = |a: Vec2, b: Vec2, c: Vec2| {
        let l = (b - a).len();
        if l < 1e-12 { 0.0 } else { (b - a).cross(c - a) / l }
    };
    let (d1, d2, d3, d4) = (dist(r, s, p), dist(r, s, q), dist(p, q, r), dist(p, q, s));
    (d1 < -eps && d2 > eps || d1 > eps && d2 < -eps) && (d3 < -eps && d4 > eps || d3 > eps && d4 < -eps)
}

#[cfg(test)]
mod tests {
    use super::{first_crossing, proper_cross};
    use ck_geom::Vec2;
    fn v(x: f64, y: f64) -> Vec2 {
        Vec2::new(x, y)
    }
    #[test]
    fn real_crossing() {
        assert!(proper_cross(v(0., 0.), v(2., 2.), v(0., 2.), v(2., 0.), 1e-6));
    }
    #[test]
    fn shared_vertex_with_noise_is_not_a_crossing() {
        // two segments meeting at (42, ~1e-15) from opposite sides of the x axis
        assert!(!proper_cross(v(42.0, 1e-15), v(41.657, 1.722), v(41.657, -1.722), v(42.0, 0.0), 1e-6));
    }
    #[test]
    fn overlapping_bridge_is_not_a_crossing() {
        assert!(!proper_cross(v(0., 0.), v(5., 0.), v(5., 0.), v(0., 0.), 1e-6));
    }

    /// The plain double loop over every segment pair that `first_crossing` replaces.
    fn brute(polys: &[Vec<Vec2>], eps: f64) -> Option<(usize, usize, usize, usize)> {
        let seg = |p: &Vec<Vec2>, i: usize| (p[i], p[(i + 1) % p.len()]);
        for (a, pa) in polys.iter().enumerate() {
            for (c, pc) in polys.iter().enumerate().skip(a) {
                for i in 0..pa.len() {
                    let s = seg(pa, i);
                    for j in 0..pc.len() {
                        if a == c && (j <= i || j == i + 1 || (i == 0 && j == pa.len() - 1)) {
                            continue;
                        }
                        let t = seg(pc, j);
                        if proper_cross(s.0, s.1, t.0, t.1, eps) {
                            return Some((a, c, i, j));
                        }
                    }
                }
            }
        }
        None
    }

    #[test]
    fn grid_search_finds_the_same_first_crossing_as_the_plain_loop() {
        let mut seed = 0x2545F4914F6CDD1Du64;
        let mut rnd = move || {
            seed ^= seed << 13;
            seed ^= seed >> 7;
            seed ^= seed << 17;
            (seed >> 11) as f64 / (1u64 << 53) as f64
        };
        let ctl = crate::Ctl::unlimited();
        let (mut with, mut without) = (0, 0);
        for case in 0..400 {
            let nloops = 1 + (rnd() * 6.0) as usize;
            let spread = if case % 4 == 0 { 100.0 } else { 10.0 };
            let polys: Vec<Vec<Vec2>> = (0..nloops)
                .map(|_| {
                    let (cx, cy, r) = (rnd() * spread, rnd() * spread, 0.5 + rnd() * 4.0);
                    let n = 3 + (rnd() * 30.0) as usize;
                    // a jittered ring, now and then with a long bridge point far away
                    (0..n)
                        .map(|k| {
                            let t = k as f64 / n as f64 * std::f64::consts::TAU;
                            let rr = if case % 7 == 0 && k == 1 { r * 60.0 } else { r * (0.8 + 0.4 * rnd()) };
                            v(cx + rr * t.cos(), cy + rr * t.sin())
                        })
                        .collect()
                })
                .collect();
            let want = brute(&polys, 1e-6);
            assert_eq!(first_crossing(&polys, 1e-6, &ctl), want, "case {case}");
            if want.is_some() {
                with += 1;
            } else {
                without += 1;
            }
        }
        assert!(with > 20 && without > 20, "the cases should cover both outcomes: {with} / {without}");
    }
}
