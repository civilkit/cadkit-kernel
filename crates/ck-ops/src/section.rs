//! Planar cross-sections of a body and point / ray probes (extension requests `section`, `sections`, `probe`).
//!
//! Method. The display tessellation of every face is cut by the plane (a triangle with vertices on both sides yields one
//! oriented segment, interior on its left in the plane frame), the segments are chained into closed loops (vertex positions
//! are bit-identical across faces: edges are sampled once), and then the polyline is made exact:
//! - every loop vertex is Newton-snapped in the plane onto the exact surface(s) of the faces meeting there (two faces: the
//!   exact corner of the two surfaces),
//! - each segment on a curved face is bisected (new midpoints snapped to the surface) until the chord sagitta is below
//!   `SAG * diagonal`,
//! - a run of one curved face whose points lie on one circle (plane perpendicular to a cylinder / cone / torus axis, any
//!   plane through a sphere centre or off it, ...) becomes an exact `arc`; the area uses the exact circular segment,
//! - a run on a planar face becomes one `line`; everything else (ellipses from oblique cuts, spline surfaces) stays a
//!   `polyline` whose vertices are on the exact surface and whose sagitta is below the tolerance.
//!
//! Orientation gives the classification: loops run counter-clockwise (positive signed area) around material and clockwise
//! around holes, so `outer` = positive, `inner` = negative, and the section area is the signed sum (Green).
//! A plane exactly through a face is resolved by treating on-plane vertices as lying on the positive side.

use crate::contain::Containment;
use ck_core::linalg::Vec3;
use ck_geom::surface::Surface;
use ck_json::{Json, Obj};
use ck_tess::mesh::{tessellate_memo, Tess};
use ck_topo::body::{Body, FaceId};
use std::collections::HashMap;

/// Sagitta tolerance of refined curved sections, relative to the body diagonal.
pub const SAG: f64 = 2e-5;
/// Largest number of points / rays per probe request, and of positions per `sections` request.
pub const MAX_PROBES: usize = 2000;
pub const MAX_SECTIONS: usize = 1000;

#[derive(Clone, Copy)]
pub struct Frame {
    pub o: Vec3,
    pub x: Vec3,
    pub y: Vec3,
    pub z: Vec3,
}

impl Frame {
    pub fn new(o: Vec3, n: Vec3) -> Frame {
        let z = n.normalize();
        let x = if z.z.abs() > 0.999 { Vec3::X } else { Vec3::Z.cross(z).normalize() };
        let y = z.cross(x).normalize();
        let x = y.cross(z).normalize();
        Frame { o, x, y, z }
    }
    fn p2(&self, p: Vec3) -> [f64; 2] {
        let d = p - self.o;
        [d.dot(self.x), d.dot(self.y)]
    }
}

type Uv = (f64, f64);

struct Seg {
    a: Vec3,
    b: Vec3,
    face: usize,
    /// face parameters at a and b (strip coordinates of the mesh: only used as Newton hints)
    ua: Uv,
    ub: Uv,
}

/// A chain point: position, face of the outgoing segment, hint of that face at the point and hint of the same
/// segment's face at its far end.
type CPt = (Vec3, usize, Uv, Uv);

/// One run of a loop: consecutive points on one face (the last point is the next run's first).
struct Run {
    face: usize,
    pts: Vec<Vec3>,
}

pub struct Loop {
    runs: Vec<Run>,
}

pub struct Section {
    pub frame: Frame,
    loops: Vec<LoopOut>,
    pub area: f64,
    pub open: Vec<[Vec3; 2]>,
    pub bbox2d: [f64; 4],
}

struct LoopOut {
    outer: bool,
    area: f64,
    edges: Vec<Json>,
    pts: Vec<[f64; 2]>,
}

fn key(p: Vec3) -> [u64; 3] {
    let f = |v: f64| if v == 0.0 { 0u64 } else { v.to_bits() };
    [f(p.x), f(p.y), f(p.z)]
}

fn lex_less(a: Vec3, b: Vec3) -> bool {
    (a.x, a.y, a.z) < (b.x, b.y, b.z)
}

/// Crossing of edge (a, b) with the plane, computed from the canonically ordered endpoints so that neighbours agree bitwise.
fn cross_pt(a: Vec3, da: f64, ua: Uv, b: Vec3, db: f64, ub: Uv) -> (Vec3, Uv) {
    let (a, da, ua, b, db, ub) = if lex_less(b, a) { (b, db, ub, a, da, ua) } else { (a, da, ua, b, db, ub) };
    let t = da / (da - db);
    (a + (b - a) * t, (ua.0 + (ub.0 - ua.0) * t, ua.1 + (ub.1 - ua.1) * t))
}

fn cut_segments(body: &Body, tess: &Tess, fr: &Frame) -> Vec<Seg> {
    let mut segs = vec![];
    for (fi, f) in body.face_ids().enumerate() {
        let _ = f;
        let Some(Some(fm)) = tess.faces.get(fi).map(|x| x.as_ref()) else { continue };
        let d: Vec<f64> = fm.pts.iter().map(|p| (*p - fr.o).dot(fr.z)).collect();
        for t in &fm.tris {
            let (i, j, k) = (t[0] as usize, t[1] as usize, t[2] as usize);
            let pos = [d[i] >= 0.0, d[j] >= 0.0, d[k] >= 0.0];
            if pos[0] == pos[1] && pos[1] == pos[2] {
                continue;
            }
            let v = [i, j, k];
            let mut c: Vec<(Vec3, Uv)> = vec![];
            for e in 0..3 {
                let (u, w) = (v[e], v[(e + 1) % 3]);
                if pos[e] != pos[(e + 1) % 3] {
                    let (uu, uw) = (fm.uv[u], fm.uv[w]);
                    c.push(cross_pt(fm.pts[u], d[u], (uu.x, uu.y), fm.pts[w], d[w], (uw.x, uw.y)));
                }
            }
            if c.len() != 2 || key(c[0].0) == key(c[1].0) {
                continue;
            }
            let n = (fm.pts[j] - fm.pts[i]).cross(fm.pts[k] - fm.pts[i]);
            let dir = fr.z.cross(n);
            let (a, b) = if (c[1].0 - c[0].0).dot(dir) >= 0.0 { (c[0], c[1]) } else { (c[1], c[0]) };
            segs.push(Seg { a: a.0, b: b.0, face: fi, ua: a.1, ub: b.1 });
        }
    }
    segs
}

/// Chains oriented segments into loops of `(point, face of the outgoing segment)`; returns (closed loops, open chain count).
fn chain(segs: &[Seg], diag: f64) -> (Vec<Vec<CPt>>, Vec<[Vec3; 2]>) {
    let mut start: HashMap<[u64; 3], Vec<usize>> = HashMap::new();
    for (i, s) in segs.iter().enumerate() {
        start.entry(key(s.a)).or_default().push(i);
    }
    let mut used = vec![false; segs.len()];
    let mut closed = vec![];
    let mut open: Vec<Vec<CPt>> = vec![];
    for s0 in 0..segs.len() {
        if used[s0] {
            continue;
        }
        used[s0] = true;
        let mut ch: Vec<CPt> = vec![(segs[s0].a, segs[s0].face, segs[s0].ua, segs[s0].ub)];
        let mut end = segs[s0].b;
        let first = key(segs[s0].a);
        loop {
            if key(end) == first {
                closed.push(ch);
                break;
            }
            let nx = start.get(&key(end)).and_then(|v| v.iter().copied().find(|&i| !used[i]));
            match nx {
                Some(i) => {
                    used[i] = true;
                    ch.push((segs[i].a, segs[i].face, segs[i].ua, segs[i].ub));
                    end = segs[i].b;
                }
                None => {
                    ch.push((end, usize::MAX, (0.0, 0.0), (0.0, 0.0)));
                    open.push(ch);
                    break;
                }
            }
        }
    }
    // join open chains whose ends meet within a small gap (mesh vertices that differ in the last bits)
    let gap = 1e-6 * diag;
    let mut n_open: Vec<[Vec3; 2]> = vec![];
    while let Some(mut c) = open.pop() {
        loop {
            let end = c.last().unwrap().0;
            if end.dist(c[0].0) <= gap {
                c.pop();
                let f = c.last().map(|x| x.1).unwrap_or(0);
                let _ = f;
                closed.push(c);
                break;
            }
            let j = open.iter().position(|o| o[0].0.dist(end) <= gap);
            match j {
                Some(j) => {
                    let o = open.remove(j);
                    c.pop();
                    c.extend(o);
                }
                None => {
                    n_open.push([c[0].0, c.last().unwrap().0]);
                    break;
                }
            }
        }
    }
    // the last point of a closed chain is the closing duplicate (never pushed), faces of open-joined chains are in place
    for c in closed.iter_mut() {
        if c.last().map_or(false, |l| l.1 == usize::MAX) {
            c.pop();
        }
    }
    (closed, n_open)
}

fn plane_gradient(s: &Surface, q: Vec3, hint: &mut Uv) -> (f64, Vec3) {
    let (u, v, cp) = s.closest_point_near(q, *hint);
    *hint = (u, v);
    let n = s.normal(u, v);
    ((q - cp).dot(n), n)
}

/// In-plane Newton: move `p` along the plane until it lies on surface `a` (and `b` when given).
fn snap(a: &Surface, b: Option<&Surface>, p: Vec3, fr: &Frame, lim: f64, ha_: Uv, hb_: Uv) -> Vec3 {
    let mut q = p;
    let (mut ia, mut ib) = (ha_, hb_);
    for _ in 0..8 {
        let (ha, na) = plane_gradient(a, q, &mut ia);
        let ga = na - fr.z * na.dot(fr.z);
        match b {
            None => {
                if ha.abs() < 1e-11 * lim.max(1.0) {
                    break;
                }
                let l2 = ga.len2();
                if l2 < 1e-6 {
                    return p;
                }
                q = q - ga * (ha / l2);
            }
            Some(b) => {
                let (hb, nb) = plane_gradient(b, q, &mut ib);
                if ha.abs().max(hb.abs()) < 1e-11 * lim.max(1.0) {
                    break;
                }
                let gb = nb - fr.z * nb.dot(fr.z);
                let (a11, a12, a21, a22) = (ga.dot(fr.x), ga.dot(fr.y), gb.dot(fr.x), gb.dot(fr.y));
                let det = a11 * a22 - a12 * a21;
                if det.abs() < 1e-4 {
                    return snap(a, None, p, fr, lim, ha_, hb_);
                }
                let (dx, dy) = ((-ha * a22 + hb * a12) / det, (-hb * a11 + ha * a21) / det);
                q = q + fr.x * dx + fr.y * dy;
            }
        }
    }
    if q.dist(p) > 0.05 * lim || !q.x.is_finite() || !q.y.is_finite() || !q.z.is_finite() {
        p
    } else {
        q
    }
}

fn is_plane(s: &Surface) -> bool {
    matches!(s, Surface::Plane { .. })
}

/// Bisect the segment (p, q) on a curved surface until the midpoint deviation is below `tol`; pushes interior points.
fn refine(s: &Surface, p: Vec3, q: Vec3, hp: Uv, hq: Uv, fr: &Frame, lim: f64, tol: f64, depth: u32, out: &mut Vec<Vec3>) {
    if depth == 0 {
        return;
    }
    let m = (p + q) * 0.5;
    let hm = ((hp.0 + hq.0) * 0.5, (hp.1 + hq.1) * 0.5);
    let ms = snap(s, None, m, fr, lim, hm, hm);
    let dev = ms.dist(m);
    if dev <= tol {
        return;
    }
    if dev > 0.3 * p.dist(q) + tol {
        return; // the snap jumped to another part of the surface
    }
    refine(s, p, ms, hp, hm, fr, lim, tol, depth - 1, out);
    out.push(ms);
    refine(s, ms, q, hm, hq, fr, lim, tol, depth - 1, out);
}

fn circle_fit(p: [[f64; 2]; 3]) -> Option<([f64; 2], f64)> {
    let (ax, ay, bx, by, cx, cy) = (p[0][0], p[0][1], p[1][0], p[1][1], p[2][0], p[2][1]);
    let d = 2.0 * (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by));
    if d.abs() < 1e-12 {
        return None;
    }
    let (a2, b2, c2) = (ax * ax + ay * ay, bx * bx + by * by, cx * cx + cy * cy);
    let ux = (a2 * (by - cy) + b2 * (cy - ay) + c2 * (ay - by)) / d;
    let uy = (a2 * (cx - bx) + b2 * (ax - cx) + c2 * (bx - ax)) / d;
    Some(([ux, uy], ((ax - ux).powi(2) + (ay - uy).powi(2)).sqrt()))
}

fn shoelace_open(p: &[[f64; 2]]) -> f64 {
    p.windows(2).map(|w| w[0][0] * w[1][1] - w[1][0] * w[0][1]).sum::<f64>() * 0.5
}

fn j2(p: [f64; 2]) -> Json {
    Json::Arr(vec![Json::Num(p[0]), Json::Num(p[1])])
}

/// Section of `body` at the plane through `o` with normal `n`.
pub fn section(body: &Body, tess: &Tess, o: Vec3, n: Vec3) -> Section {
    let fr = Frame::new(o, n);
    let (lo, hi) = body.bbox();
    let diag = (hi - lo).len().max(1e-9);
    let faces: Vec<FaceId> = body.face_ids().collect();
    let name = |fi: usize| body.face_id_str(faces[fi]);
    let segs = cut_segments(body, tess, &fr);
    let (chains, n_open) = chain(&segs, diag);
    let tol = SAG * diag;
    let mut loops: Vec<LoopOut> = vec![];
    let mut bb = [f64::INFINITY, f64::INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY];
    for ch in chains {
        let m = ch.len();
        if m < 2 {
            continue;
        }
        // snap the vertices to the exact surfaces of the two faces meeting there
        let mut pts: Vec<Vec3> = Vec::with_capacity(m);
        for k in 0..m {
            let fin = ch[(k + m - 1) % m].1;
            let fout = ch[k].1;
            let (sa, sb) = (&body.face(faces[fin]).surface, &body.face(faces[fout]).surface);
            let p = ch[k].0;
            let q = if fin == fout {
                if is_plane(sa) { p } else { snap(sa, None, p, &fr, diag, ch[k].2, ch[k].2) }
            } else if is_plane(sa) && is_plane(sb) {
                p
            } else {
                snap(sa, Some(sb), p, &fr, diag, ch[(k + m - 1) % m].3, ch[k].2)
            };
            pts.push(q);
        }
        // refine curved segments; keep the face of every point
        let mut rp: Vec<(Vec3, usize)> = vec![];
        for k in 0..m {
            rp.push((pts[k], ch[k].1));
            let f = ch[k].1;
            let s = &body.face(faces[f]).surface;
            if !is_plane(s) {
                let mut mid = vec![];
                refine(s, pts[k], pts[(k + 1) % m], ch[k].2, ch[k].3, &fr, diag, tol, 7, &mut mid);
                for q in mid {
                    rp.push((q, f));
                }
            }
        }
        // rotate to a face boundary and cut into runs
        let r = rp.len();
        let start = (0..r).find(|&i| rp[i].1 != rp[(i + r - 1) % r].1).unwrap_or(0);
        rp.rotate_left(start);
        let mut runs: Vec<Run> = vec![];
        for (p, f) in &rp {
            match runs.last_mut() {
                Some(last) if last.face == *f => last.pts.push(*p),
                _ => runs.push(Run { face: *f, pts: vec![*p] }),
            }
        }
        for i in 0..runs.len() {
            let nxt = runs[(i + 1) % runs.len()].pts[0];
            runs[i].pts.push(nxt);
        }
        let lp = Loop { runs };
        // 2D edges and area
        let mut edges: Vec<Json> = vec![];
        let mut pts2: Vec<[f64; 2]> = vec![];
        let mut area = 0.0;
        for run in &lp.runs {
            let p2: Vec<[f64; 2]> = run.pts.iter().map(|&p| fr.p2(p)).collect();
            let s = &body.face(faces[run.face]).surface;
            let e0 = Obj::new().put("face", name(run.face));
            let a = p2[0];
            let b = *p2.last().unwrap();
            if is_plane(s) {
                area += shoelace_open(&[a, b]);
                edges.push(e0.put("type", "line").put("from", j2(a)).put("to", j2(b)).build());
                pts2.push(a);
                continue;
            }
            // exact arc?
            let np = p2.len();
            let mut arc: Option<([f64; 2], f64)> = None;
            if np >= 4 {
                let closed = key(*run.pts.last().unwrap()) == key(run.pts[0]);
                let idx = if closed { [0, np / 3, 2 * np / 3] } else { [0, (np - 1) / 2, np - 1] };
                if let Some((c, rad)) = circle_fit([p2[idx[0]], p2[idx[1]], p2[idx[2]]]) {
                    let tol_c = 1e-6 * diag;
                    if rad < 1e4 * diag && p2.iter().all(|q| (((q[0] - c[0]).powi(2) + (q[1] - c[1]).powi(2)).sqrt() - rad).abs() <= tol_c) {
                        arc = Some((c, rad));
                    }
                }
            }
            if let Some((c, rad)) = arc {
                let ang = |q: [f64; 2]| (q[1] - c[1]).atan2(q[0] - c[0]);
                let mut sweep = 0.0;
                for w in p2.windows(2) {
                    let mut d = ang(w[1]) - ang(w[0]);
                    while d > std::f64::consts::PI {
                        d -= 2.0 * std::f64::consts::PI;
                    }
                    while d < -std::f64::consts::PI {
                        d += 2.0 * std::f64::consts::PI;
                    }
                    sweep += d;
                }
                let poly = (shoelace_open(&p2) * 2.0 + (b[0] * a[1] - a[0] * b[1])) * 0.5;
                let exact = 0.5 * rad * rad * (sweep - sweep.sin());
                area += shoelace_open(&[a, b]) + exact;
                let _ = poly;
                edges.push(
                    e0.put("type", "arc")
                        .put("centre", j2(c))
                        .put("radius", rad)
                        .put("start", j2(a))
                        .put("end", j2(b))
                        .put("startAngle", ang(a))
                        .put("sweep", sweep)
                        .put("ccw", sweep > 0.0)
                        .build(),
                );
                let ns = ((sweep.abs() / 0.0873).ceil() as usize).max(2);
                for i in 0..ns {
                    let t = ang(a) + sweep * i as f64 / ns as f64;
                    pts2.push([c[0] + rad * t.cos(), c[1] + rad * t.sin()]);
                }
            } else {
                area += shoelace_open(&p2);
                edges.push(
                    e0.put("type", "polyline").put("pts", Json::Arr(p2.iter().map(|&q| j2(q)).collect())).build(),
                );
                pts2.extend(p2[..np - 1].iter().copied());
            }
        }
        if area.abs() < 1e-9 * diag * diag {
            continue;
        }
        for q in &pts2 {
            bb = [bb[0].min(q[0]), bb[1].min(q[1]), bb[2].max(q[0]), bb[3].max(q[1])];
        }
        loops.push(LoopOut { outer: area > 0.0, area, edges, pts: pts2 });
    }
    loops.sort_by(|a, b| b.area.abs().partial_cmp(&a.area.abs()).unwrap_or(std::cmp::Ordering::Equal));
    let area = loops.iter().map(|l| l.area).sum();
    if loops.is_empty() {
        bb = [0.0; 4];
    }
    Section { frame: fr, loops, area, open: n_open, bbox2d: bb }
}

fn v3(v: Vec3) -> Json {
    Json::Arr(vec![Json::Num(v.x), Json::Num(v.y), Json::Num(v.z)])
}

impl Section {
    pub fn loop_count(&self) -> usize {
        self.loops.len()
    }
    pub fn to_json(&self) -> Json {
        let loops: Vec<Json> = self
            .loops
            .iter()
            .map(|l| {
                Obj::new()
                    .put("kind", if l.outer { "outer" } else { "inner" })
                    .put("closed", true)
                    .put("area", l.area.abs())
                    .put("edges", Json::Arr(l.edges.clone()))
                    .put("pts", Json::Arr(l.pts.iter().map(|&p| j2(p)).collect()))
                    .build()
            })
            .collect();
        let f = &self.frame;
        let mut o = Obj::new()
            .put("loops", Json::Arr(loops))
            .put("frame", Obj::new().put("o", v3(f.o)).put("x", v3(f.x)).put("y", v3(f.y)).put("z", v3(f.z)).build())
            .put("area", self.area)
            .put("bbox2d", Json::Arr(self.bbox2d.iter().map(|&x| Json::Num(x)).collect()));
        if !self.open.is_empty() {
            o = o.put("openChains", self.open.len() as f64);
            o = o.put("openEnds", Json::Arr(self.open.iter().map(|e| Json::Arr(vec![v3(e[0]), v3(e[1])])).collect()));
        }
        o.build()
    }
}

fn vec3_of(j: Option<&Json>, what: &str) -> Result<Vec3, String> {
    let a = j.and_then(|j| j.as_vec3()).ok_or_else(|| format!("{what} must be [x, y, z]"))?;
    if !a.iter().all(|x| x.is_finite()) {
        return Err(format!("{what} must be finite"));
    }
    Ok(Vec3::new(a[0], a[1], a[2]))
}

/// `{type:"section", plane:{origin, normal}}` reply.
pub fn section_reply(body: &Body, msg: &Json) -> Result<Json, String> {
    let pl = msg.get("plane").ok_or("section: plane {origin, normal} is required")?;
    let o = vec3_of(pl.get("origin"), "section: plane.origin")?;
    let n = vec3_of(pl.get("normal"), "section: plane.normal")?;
    if n.len() < 1e-12 {
        return Err("section: plane.normal must not be zero".into());
    }
    let tess = tessellate_memo(body);
    Ok(section(body, &tess, o, n).to_json())
}

/// `{type:"sections", axis, n | positions}` reply.
pub fn sections_reply(body: &Body, msg: &Json, exact_volume: f64) -> Result<Json, String> {
    let axis = match msg.get("axis") {
        None => Vec3::Z,
        Some(Json::Str(s)) => match s.as_str() {
            "x" => Vec3::X,
            "y" => Vec3::Y,
            "z" => Vec3::Z,
            _ => return Err("sections: axis must be 'x', 'y', 'z' or [nx, ny, nz]".into()),
        },
        Some(j) => vec3_of(Some(j), "sections: axis")?,
    };
    if axis.len() < 1e-12 {
        return Err("sections: axis must not be zero".into());
    }
    let axis = axis.normalize();
    let tess = tessellate_memo(body);
    let (mut lo, mut hi) = (f64::INFINITY, f64::NEG_INFINITY);
    for fm in tess.faces.iter().flatten() {
        for p in &fm.pts {
            let t = p.dot(axis);
            lo = lo.min(t);
            hi = hi.max(t);
        }
    }
    if !(lo < hi) {
        return Err("sections: the body has no extent along the axis".into());
    }
    let pad = 1e-9 * (hi - lo);
    let (positions, midpoint): (Vec<f64>, bool) = match msg.get("positions") {
        Some(Json::Arr(a)) => {
            let mut v: Vec<f64> = a.iter().map(|x| x.as_f64().ok_or("sections: positions must be numbers")).collect::<Result<_, _>>()?;
            if v.iter().any(|x| !x.is_finite()) {
                return Err("sections: positions must be finite".into());
            }
            v.sort_by(|a, b| a.partial_cmp(b).unwrap());
            (v, false)
        }
        Some(_) => return Err("sections: positions must be an array of numbers".into()),
        None => {
            let n = match msg.get("n") {
                None => 21,
                Some(j) => j.as_usize().filter(|&n| n >= 1).ok_or("sections: n must be a positive integer")?,
            };
            let h = (hi - lo + 2.0 * pad) / n as f64;
            ((0..n).map(|i| lo - pad + h * (i as f64 + 0.5)).collect(), true)
        }
    };
    if positions.is_empty() || positions.len() > MAX_SECTIONS {
        return Err(format!("sections: between 1 and {MAX_SECTIONS} positions"));
    }
    // a section is a plane through `axis * position` (a point on the axis line through the origin)
    let mut areas = vec![];
    let mut counts = vec![];
    for &t in &positions {
        let s = section(body, &tess, axis * t, axis);
        areas.push(s.area);
        counts.push(s.loop_count() as f64);
    }
    let vol = if midpoint {
        let h = (hi - lo + 2.0 * pad) / positions.len() as f64;
        areas.iter().sum::<f64>() * h
    } else {
        positions.windows(2).zip(areas.windows(2)).map(|(p, a)| (p[1] - p[0]) * 0.5 * (a[0] + a[1])).sum()
    };
    let err = if exact_volume.abs() > 0.0 { (vol - exact_volume).abs() / exact_volume.abs() } else { 0.0 };
    Ok(Obj::new()
        .put("axis", v3(axis))
        .put("range", Json::Arr(vec![Json::Num(lo), Json::Num(hi)]))
        .put("rule", if midpoint { "midpoint" } else { "trapezoid" })
        .put("positions", Json::Arr(positions.iter().map(|&x| Json::Num(x)).collect()))
        .put("areas", Json::Arr(areas.iter().map(|&x| Json::Num(x)).collect()))
        .put("loops", Json::Arr(counts.iter().map(|&x| Json::Num(x)).collect()))
        .put("volume", vol)
        .put("exactVolume", exact_volume)
        .put("volumeError", err)
        .build())
}

/// `{type:"probe", points?:[[x,y,z]], rays?:[{origin, dir}]}` reply.
pub fn probe_reply(body: &Body, msg: &Json) -> Result<Json, String> {
    let pts = msg.get("points");
    let rays = msg.get("rays");
    if pts.is_none() && rays.is_none() {
        return Err("probe: points [[x, y, z], ...] or rays [{origin, dir}, ...] is required".into());
    }
    let (lo, hi) = body.bbox();
    let diag = (hi - lo).len().max(1e-9);
    let tol = 1e-6;
    let c = Containment::new(body, tol);
    let mut o = Obj::new();
    if let Some(p) = pts {
        let a = p.as_arr().ok_or("probe: points must be an array")?;
        if a.len() > MAX_PROBES {
            return Err(format!("probe: at most {MAX_PROBES} points per request"));
        }
        let mut out = vec![];
        for q in a {
            let q = vec3_of(Some(q), "probe: each point")?;
            let loc = c.classify_point(q, tol);
            let r = Obj::new();
            out.push(match c.nearest(q) {
                Some((d, f, np)) => {
                    let (inside, sd): (Json, f64) = match loc {
                        crate::contain::Loc::In => (Json::Bool(true), -d),
                        crate::contain::Loc::Out => (Json::Bool(false), d),
                        crate::contain::Loc::On => (Json::Str("boundary".into()), 0.0),
                    };
                    r.put("inside", inside).put("distance", sd).put("face", body.face_id_str(f)).put("nearest", v3(np)).build()
                }
                None => r.put("inside", false).put("distance", Json::Null).put("face", Json::Null).put("nearest", Json::Null).build(),
            });
        }
        o = o.put("points", Json::Arr(out));
    }
    if let Some(r) = rays {
        let a = r.as_arr().ok_or("probe: rays must be an array")?;
        if a.len() > MAX_PROBES {
            return Err(format!("probe: at most {MAX_PROBES} rays per request"));
        }
        let mut out = vec![];
        for ray in a {
            let org = vec3_of(ray.get("origin"), "probe: ray.origin")?;
            let d = vec3_of(ray.get("dir"), "probe: ray.dir")?;
            if d.len() < 1e-12 {
                return Err("probe: ray.dir must not be zero".into());
            }
            let d = d.normalize();
            // distance from the origin to the far side of the (padded) box bounds the ray
            let far = (org - (lo + hi) * 0.5).len() + diag + 1.0;
            let hits: Vec<Json> = c
                .ray_hits(org, d, far, tol)
                .iter()
                .map(|h| {
                    let mut ho = Obj::new().put("t", h.t).put("point", v3(h.point)).put("face", body.face_id_str(h.face)).put("entering", h.entering);
                    if h.tangent {
                        ho = ho.put("tangent", true);
                    }
                    ho.build()
                })
                .collect();
            out.push(Obj::new().put("hits", Json::Arr(hits)).build());
        }
        o = o.put("rays", Json::Arr(out));
    }
    Ok(o.build())
}
