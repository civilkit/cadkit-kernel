//! Linear patterns, circular patterns and mirrors, read back from the faces the pattern left on the final body.
//!
//! A SolidWorks pattern tags the faces of its copies with the pattern's own feature id (the seed keeps its tag). The
//! feature list names no seed, and its spacing values are display-rounded, so everything is read off the geometry:
//!
//! * the transform: a candidate comes from a pair (a face of an earlier feature, a face of the pattern) of the same
//!   surface type and size: a translation along a world axis (linear), a rotation about a world axis through the
//!   origin (circular), a reflection in a plane (mirror). A candidate is accepted when EVERY face of the pattern is the
//!   image of a face of an earlier feature under one of the instance transforms (plane, cylinder, cone, sphere, torus
//!   compared as surfaces, so a copy trimmed differently from its seed still matches), and at least one image is a
//!   face that moved;
//! * the counts come from the file's dimensions (`D1`, and `D2` for a second direction): they are exact; the spacings
//!   (`D3`, `D4`) are rounded for display and are not used;
//! * the seeds: the earlier extrude / hole / pattern features, picked greedily by the number of pattern faces their
//!   moved faces explain (later features win ties), until every pattern face is explained.
//!
//! The pattern is emitted as our `lpattern` / `cpattern` / `mirror` feature over the seed features; the whole rebuild
//! is verified against the import as usual, so a wrong guess falls back and never ships. Limits: the direction of a
//! linear pattern is first tried along a world axis and a circular pattern about a world axis through the origin;
//! when no such transform fits, the general forms are tried (an arbitrary direction vector; an arbitrary axis given by
//! a point and a direction, found from the centres of the moved round faces). A mirror plane is any plane.

use crate::{Cx, TOL};
use ck_core::linalg::Plane;
use ck_core::xform::Xform;
use ck_geom::Vec3;
use ck_json::Obj;
use ck_recog::emit::n;
use ck_recog::model::{axis_vec, comp, par, Kind, Model};
use ck_sldprt::meta::SwFeature;
use ck_topo::body::FaceId;
use std::collections::{HashMap, HashSet};

/// What the pattern module needs of the rebuild so far.
pub struct Env<'a> {
    /// Faces of the final body per SolidWorks feature id.
    pub groups: &'a HashMap<i64, Vec<FaceId>>,
    /// Range of `Cx::tree.feats` each rebuilt SolidWorks feature emitted.
    pub made: &'a HashMap<i64, (usize, usize)>,
    /// The whole feature list (tree order).
    pub features: &'a [SwFeature],
}

#[derive(Clone, Copy, PartialEq)]
enum Pat {
    Linear,
    Circular,
    Mirror,
}

/// Rebuild a pattern or mirror feature; `None` when `f` is no such feature.
pub fn rebuild(cx: &mut Cx, f: &SwFeature, faces: &[FaceId], env: &Env) -> Option<Result<String, String>> {
    let kind = match f.kind.as_str() {
        "LPattern" => Pat::Linear,
        "CirPattern" => Pat::Circular,
        "Mirror" | "MirrorPattern" => Pat::Mirror,
        _ => return None,
    };
    Some(run(cx, f, faces, env, kind))
}

struct Seed {
    ids: Vec<String>,
    faces: Vec<FaceId>,
}

/// The transforms of a found pattern and the emitted feature's parameters.
struct Found {
    xs: Vec<Xform>,
    json: Box<dyn Fn(&str, &[String]) -> ck_json::Json>,
    note: String,
}

fn run(cx: &mut Cx, f: &SwFeature, fp: &[FaceId], env: &Env, kind: Pat) -> Result<String, String> {
    let first = run_faces(cx, f, fp, env, kind);
    let Err(why) = first else { return first };
    // The copies may carry holes whose seed is a hole feature that comes later in the tree (the file's order is not
    // the order of the solid): pattern what the earlier features explain, then cut the copies' holes as a hole feature.
    let holes = ck_recog::holes::find_holes_in(cx.m, &mut HashSet::new(), Some(&fp.iter().copied().collect()));
    let hole_faces: HashSet<FaceId> = holes.iter().flat_map(|h| h.faces.iter().copied()).collect();
    let rest: Vec<FaceId> = fp.iter().copied().filter(|g| !hole_faces.contains(g)).collect();
    if hole_faces.is_empty() || rest.is_empty() {
        return Err(why);
    }
    let mark = cx.tree.feats.len();
    let note = run_faces(cx, f, &rest, env, kind).map_err(|e| format!("{why}; without its holes: {e}"))?;
    let hole_faces: Vec<FaceId> = hole_faces.into_iter().collect();
    match crate::hole::hole_own(cx, f, &hole_faces) {
        Ok(h) => Ok(format!("{note}; the copies' holes as a hole feature ({h})")),
        Err(e) => {
            cx.tree.feats.truncate(mark);
            Err(format!("{why}; the copies' holes: {e}"))
        }
    }
}

fn run_faces(cx: &mut Cx, f: &SwFeature, fp: &[FaceId], env: &Env, kind: Pat) -> Result<String, String> {
    let m = cx.m;
    if let Some(&bad) = fp.iter().find(|&&p| matches!(m.kind(p), Kind::Other(_))) {
        return Err(format!("{} face in a pattern", m.kind(bad).name()));
    }
    let mut seeds: Vec<Seed> = vec![];
    for g in env.features {
        if g.id == f.id {
            break;
        }
        let Some(&(a, b)) = env.made.get(&g.id) else { continue };
        let ids: Vec<String> = cx.tree.feats[a..b]
            .iter()
            .filter(|j| matches!(j.get_str("type"), Some("extrude" | "revolve" | "hole" | "lpattern" | "cpattern" | "mirror")))
            .filter_map(|j| j.get_str("id").map(String::from))
            .collect();
        if !ids.is_empty() {
            seeds.push(Seed { ids, faces: env.groups.get(&g.id).cloned().unwrap_or_default() });
        }
    }
    if seeds.iter().all(|s| s.faces.is_empty()) {
        return Err("no earlier extrude / hole feature left faces to copy".into());
    }
    let sf: Vec<(usize, FaceId)> = seeds.iter().enumerate().flat_map(|(i, s)| s.faces.iter().map(move |&g| (i, g))).collect();
    let found = match kind {
        Pat::Linear => linear(cx, f, &sf, fp).or_else(|e| linear_general(cx, f, &sf, fp).map_err(|g| format!("{e}; {g}")))?,
        Pat::Circular => circular(cx, f, &sf, fp).or_else(|e| circular_general(cx, f, &sf, fp).map_err(|g| format!("{e}; {g}")))?,
        Pat::Mirror => mirror(cx, &sf, fp)?,
    };

    // the seeds: greedy cover of the pattern faces by the moved faces of one feature at a time
    let all_hits: Vec<Vec<Hit>> = fp.iter().map(|&p| hits_of(m, &sf, p, &found.xs)).collect();
    let mut unexplained: HashSet<usize> = (0..fp.len()).collect();
    let mut chosen: Vec<usize> = vec![];
    loop {
        let mut best: Option<(usize, usize)> = None;
        for si in 0..seeds.len() {
            if chosen.contains(&si) {
                continue;
            }
            let c = unexplained.iter().filter(|&&pi| all_hits[pi].iter().any(|h| h.seed == si && h.moved)).count();
            if c > 0 && best.map_or(true, |(_, bc)| c >= bc) {
                best = Some((si, c));
            }
        }
        let Some((si, _)) = best else { break };
        chosen.push(si);
        unexplained.retain(|&pi| !all_hits[pi].iter().any(|h| h.seed == si));
    }
    if !unexplained.is_empty() {
        return Err(format!("{} of {} pattern faces are not copies of one feature's faces", unexplained.len(), fp.len()));
    }
    chosen.sort();
    let seed_ids: Vec<String> = chosen.iter().flat_map(|&si| seeds[si].ids.iter().cloned()).collect();
    let pid = cx.tree.ids.next();
    let json = (found.json)(&pid, &seed_ids);
    cx.tree.push(json);

    // the copies' names in our document (`<seed face>@<pattern>.<k>`), so a later blend can pick them
    for (pi, &p) in fp.iter().enumerate() {
        if cx.names.contains_key(&p) {
            continue;
        }
        let mut hs: Vec<&Hit> = all_hits[pi].iter().filter(|h| chosen.contains(&h.seed)).collect();
        hs.sort_by_key(|h| !h.moved);
        if let Some(h) = hs.first() {
            if let Some(base) = cx.names.get(&h.face).cloned() {
                cx.names.insert(p, format!("{base}@{pid}.{}", h.xf + 1));
            }
        }
    }
    Ok(format!("{} of {} seed feature(s), {} pattern faces", found.note, chosen.len(), fp.len()))
}

// ---------- surface images ----------

fn apex(o: Vec3, a: Vec3, r0: f64, half: f64) -> Option<Vec3> {
    let t = half.tan();
    (t.abs() > 1e-9).then(|| o - a * (r0 / t))
}

/// The surface `a` carried by `x` is the surface `b` (planes with the outward normal; analytic surfaces by their
/// defining point / axis / radii).
fn img_matches(a: &Kind, x: &Xform, b: &Kind) -> bool {
    match (a, b) {
        (Kind::Plane { n: na, d: da }, Kind::Plane { n: nb, d: db }) => {
            let n2 = x.apply_vec(*na);
            let p2 = x.apply_point(*na * *da);
            n2.dot(*nb) > 1.0 - 1e-9 && (n2.dot(p2) - db).abs() < TOL
        }
        (Kind::Cyl { o: oa, a: aa, r: ra, convex: ca }, Kind::Cyl { o: ob, a: ab, r: rb, convex: cb }) => {
            let (o2, a2) = (x.apply_point(*oa), x.apply_vec(*aa));
            ca == cb && (ra - rb).abs() < 1e-6 && par(a2, *ab) && (o2 - *ob).cross(*ab).len() < TOL
        }
        (Kind::Sphere { c: ca, r: ra, convex: va }, Kind::Sphere { c: cb, r: rb, convex: vb }) => va == vb && (ra - rb).abs() < 1e-6 && x.apply_point(*ca).dist(*cb) < TOL,
        (Kind::Torus { c: ca, a: aa, major: ma, minor: ia, convex: va }, Kind::Torus { c: cb, a: ab, major: mb, minor: ib, convex: vb }) => {
            va == vb && (ma - mb).abs() < 1e-6 && (ia - ib).abs() < 1e-6 && par(x.apply_vec(*aa), *ab) && x.apply_point(*ca).dist(*cb) < TOL
        }
        (Kind::Cone { o: oa, a: aa, r0: ra, half: ha, .. }, Kind::Cone { o: ob, a: ab, r0: rb, half: hb, .. }) => {
            let (Some(pa), Some(pb)) = (apex(*oa, *aa, *ra, *ha), apex(*ob, *ab, *rb, *hb)) else { return false };
            (ha.abs() - hb.abs()).abs() < 1e-7 && par(x.apply_vec(*aa), *ab) && x.apply_point(pa).dist(pb) < TOL
        }
        _ => false,
    }
}

struct Hit {
    seed: usize,
    face: FaceId,
    /// Index of the instance transform.
    xf: usize,
    /// The image is a different surface from the seed face's own.
    moved: bool,
}

fn hits_of(m: &Model, sf: &[(usize, FaceId)], p: FaceId, xs: &[Xform]) -> Vec<Hit> {
    let kp = m.kind(p);
    let mut out = vec![];
    for &(seed, s) in sf {
        let ks = m.kind(s);
        for (xf, x) in xs.iter().enumerate() {
            if img_matches(ks, x, kp) {
                out.push(Hit { seed, face: s, xf, moved: !img_matches(ks, x, ks) });
            }
        }
    }
    out
}

struct Cover {
    /// Every pattern face is the image of some seed face.
    all: bool,
    /// Instance `k` has at least one moved face explained.
    per: Vec<bool>,
}

impl Cover {
    fn ok(&self) -> bool {
        self.all && self.per.iter().any(|&b| b)
    }
    fn tight(&self) -> bool {
        self.all && self.per.iter().all(|&b| b)
    }
}

fn cover(m: &Model, sf: &[(usize, FaceId)], fp: &[FaceId], xs: &[Xform]) -> Cover {
    let mut per = vec![false; xs.len()];
    for &p in fp {
        let kp = m.kind(p);
        let mut found = false;
        for &(_, s) in sf {
            let ks = m.kind(s);
            for (k, x) in xs.iter().enumerate() {
                if img_matches(ks, x, kp) {
                    found = true;
                    if !per[k] && !img_matches(ks, x, ks) {
                        per[k] = true;
                    }
                }
            }
        }
        if !found {
            return Cover { all: false, per };
        }
    }
    Cover { all: true, per }
}

/// Pick the first candidate transform set that covers the pattern, preferring one where every instance is seen.
fn choose<T: Clone>(cands: impl Iterator<Item = (T, Vec<Xform>)>, m: &Model, sf: &[(usize, FaceId)], fp: &[FaceId]) -> Option<(T, Vec<Xform>)> {
    let mut loose: Option<(T, Vec<Xform>)> = None;
    for (t, xs) in cands {
        let c = cover(m, sf, fp, &xs);
        if c.tight() {
            return Some((t, xs));
        }
        if loose.is_none() && c.ok() {
            loose = Some((t, xs));
        }
    }
    loose
}

/// Candidate values with a count, most frequent first (ties: smaller magnitude), at most `keep`.
fn rank(vals: Vec<f64>, keep: usize) -> Vec<f64> {
    let mut groups: Vec<(f64, usize)> = vec![];
    for v in vals {
        match groups.iter_mut().find(|g| (g.0 - v).abs() < 1e-6) {
            Some(g) => g.1 += 1,
            None => groups.push((v, 1)),
        }
    }
    groups.sort_by(|a, b| b.1.cmp(&a.1).then(a.0.abs().total_cmp(&b.0.abs())));
    groups.truncate(keep);
    groups.into_iter().map(|g| g.0).collect()
}

fn count_dim(f: &SwFeature, name: &str) -> Option<usize> {
    f.dims.iter().find(|d| d.name == name).and_then(|d| d.value).map(|v| v.round()).filter(|v| *v >= 1.0 && *v < 1000.0).map(|v| v as usize)
}

fn axis_json(i: usize) -> ck_json::Json {
    Obj::new().put("kind", "axis").put("axis", ["x", "y", "z"][i]).build()
}

fn comps(v: Vec3) -> [f64; 3] {
    [v.x, v.y, v.z]
}

fn reject(v: Vec3, a: Vec3) -> Vec3 {
    v - a * v.dot(a)
}

fn seeds_json(seeds: &[String]) -> ck_json::Json {
    ck_json::Json::Arr(seeds.iter().map(|s| ck_json::Json::Str(s.clone())).collect())
}

// ---------- linear ----------

/// Translation components along the world axes that carry face `a` onto face `b`.
fn lin_values(a: &Kind, b: &Kind, out: &mut Vec<(usize, f64)>) {
    let mut push = |i: usize, v: f64| {
        if v.abs() > 1e-7 {
            out.push((i, v));
        }
    };
    match (a, b) {
        (Kind::Plane { n: na, d: da }, Kind::Plane { n: nb, d: db }) if na.dot(*nb) > 1.0 - 1e-9 => {
            for i in 0..3 {
                let c = comp(*na, i);
                if c.abs() > 1.0 - 1e-9 {
                    push(i, (db - da) / c);
                }
            }
        }
        (Kind::Cyl { o: oa, a: aa, r: ra, convex: ca }, Kind::Cyl { o: ob, a: ab, r: rb, convex: cb }) if ca == cb && (ra - rb).abs() < 1e-6 && par(*aa, *ab) => {
            let w = reject(*ob - *oa, *ab);
            for (i, v) in comps(w).into_iter().enumerate() {
                push(i, v);
            }
        }
        (Kind::Sphere { c: ca, r: ra, .. }, Kind::Sphere { c: cb, r: rb, .. }) if (ra - rb).abs() < 1e-6 => {
            for (i, v) in comps(*cb - *ca).into_iter().enumerate() {
                push(i, v);
            }
        }
        (Kind::Torus { c: ca, a: aa, major: ma, minor: ia, .. }, Kind::Torus { c: cb, a: ab, major: mb, minor: ib, .. }) if (ma - mb).abs() < 1e-6 && (ia - ib).abs() < 1e-6 && par(*aa, *ab) => {
            for (i, v) in comps(*cb - *ca).into_iter().enumerate() {
                push(i, v);
            }
        }
        (Kind::Cone { o: oa, a: aa, r0: ra, half: ha, .. }, Kind::Cone { o: ob, a: ab, r0: rb, half: hb, .. }) if (ha.abs() - hb.abs()).abs() < 1e-7 && par(*aa, *ab) => {
            if let (Some(pa), Some(pb)) = (apex(*oa, *aa, *ra, *ha), apex(*ob, *ab, *rb, *hb)) {
                for (i, v) in comps(pb - pa).into_iter().enumerate() {
                    push(i, v);
                }
            }
        }
        _ => {}
    }
}

fn linear(cx: &Cx, f: &SwFeature, sf: &[(usize, FaceId)], fp: &[FaceId]) -> Result<Found, String> {
    let m = cx.m;
    let n1 = count_dim(f, "D1").filter(|&c| c >= 2).ok_or("pattern count (D1) not listed")?;
    let n2 = if f.dims.iter().any(|d| d.name == "D4") { count_dim(f, "D2").unwrap_or(1) } else { 1 };
    // candidate base translations along one world axis each
    let mut per_axis: [Vec<f64>; 3] = [vec![], vec![], vec![]];
    for &p in fp {
        for &(_, s) in sf {
            let mut v = vec![];
            lin_values(m.kind(s), m.kind(p), &mut v);
            for (i, x) in v {
                per_axis[i].push(x);
            }
        }
    }
    let mut cands: Vec<(usize, f64)> = vec![];
    for (i, vals) in per_axis.into_iter().enumerate() {
        for v in rank(vals, 24) {
            cands.push((i, v));
        }
    }
    cands.sort_by(|a, b| a.1.abs().total_cmp(&b.1.abs()));
    if cands.is_empty() {
        return Err("no translation along a world axis carries a seed face onto a pattern face".into());
    }
    // instance order of our `lpattern`: row by row, the seed (0, 0) left out
    let grid = |c1: (usize, f64), na: usize, c2: Option<(usize, f64)>, nb: usize| -> Vec<Xform> {
        let mut xs = vec![];
        for j in 0..nb {
            for i in 0..na {
                if i == 0 && j == 0 {
                    continue;
                }
                let mut t = axis_vec(c1.0) * (c1.1 * i as f64);
                if let Some(c2) = c2 {
                    t = t + axis_vec(c2.0) * (c2.1 * j as f64);
                }
                xs.push(Xform::translation(t));
            }
        }
        xs
    };
    let name = |c: (usize, f64)| format!("{}{}", if c.1 < 0.0 { "-" } else { "+" }, ["x", "y", "z"][c.0]);
    if n2 <= 1 {
        let (c, xs) = choose(cands.iter().map(|&c| (c, grid(c, n1, None, 1))), m, sf, fp).ok_or("no translation along a world axis reproduces the pattern's faces")?;
        let json = move |pid: &str, seeds: &[String]| {
            Obj::new()
                .put("type", "lpattern")
                .put("id", pid)
                .put("seeds", seeds_json(seeds))
                .put("dir", axis_json(c.0))
                .put("count", n(n1 as f64))
                .put("spacing", n(c.1.abs()))
                .put("reverse", c.1 < 0.0)
                .build()
        };
        let note = format!("linear pattern x{n1} along {} by {:.4}", name(c), c.1.abs());
        return Ok(Found { xs, json: Box::new(json), note });
    }
    let mut counts = vec![(n1, n2)];
    if n1 != n2 {
        counts.push((n2, n1));
    }
    let cs = &cands;
    let pairs = counts.into_iter().flat_map(move |(na, nb)| cs.iter().flat_map(move |&c1| cs.iter().filter(move |c2| c2.0 != c1.0).map(move |&c2| (c1, c2, na, nb))));
    let ((c1, c2, na, nb), xs) = choose(pairs.map(|t| (t, grid(t.0, t.2, Some(t.1), t.3))), m, sf, fp).ok_or("no pair of world-axis translations reproduces the pattern's faces")?;
    let json = move |pid: &str, seeds: &[String]| {
        Obj::new()
            .put("type", "lpattern")
            .put("id", pid)
            .put("seeds", seeds_json(seeds))
            .put("dir", axis_json(c1.0))
            .put("count", n(na as f64))
            .put("spacing", n(c1.1.abs()))
            .put("reverse", c1.1 < 0.0)
            .put("dir2", axis_json(c2.0))
            .put("count2", n(nb as f64))
            .put("spacing2", n(c2.1.abs()))
            .put("reverse2", c2.1 < 0.0)
            .build()
    };
    let note = format!("linear pattern {na} x {nb} ({} by {:.4}, {} by {:.4})", name(c1), c1.1.abs(), name(c2), c2.1.abs());
    Ok(Found { xs, json: Box::new(json), note })
}

/// Full translation vectors that carry face `a` onto face `b` (a cylinder only fixes the part across its axis).
fn lin_vectors(a: &Kind, b: &Kind, out: &mut Vec<Vec3>) {
    let mut push = |v: Vec3| {
        if v.len() > 1e-7 {
            out.push(v);
        }
    };
    match (a, b) {
        (Kind::Cyl { o: oa, a: aa, r: ra, convex: ca }, Kind::Cyl { o: ob, a: ab, r: rb, convex: cb }) if ca == cb && (ra - rb).abs() < 1e-6 && par(*aa, *ab) => push(reject(*ob - *oa, *ab)),
        (Kind::Sphere { c: ca, r: ra, .. }, Kind::Sphere { c: cb, r: rb, .. }) if (ra - rb).abs() < 1e-6 => push(*cb - *ca),
        (Kind::Torus { c: ca, a: aa, major: ma, minor: ia, .. }, Kind::Torus { c: cb, a: ab, major: mb, minor: ib, .. }) if (ma - mb).abs() < 1e-6 && (ia - ib).abs() < 1e-6 && par(*aa, *ab) => push(*cb - *ca),
        (Kind::Cone { o: oa, a: aa, r0: ra, half: ha, .. }, Kind::Cone { o: ob, a: ab, r0: rb, half: hb, .. }) if (ha.abs() - hb.abs()).abs() < 1e-7 && par(*aa, *ab) => {
            if let (Some(pa), Some(pb)) = (apex(*oa, *aa, *ra, *ha), apex(*ob, *ab, *rb, *hb)) {
                push(pb - pa)
            }
        }
        _ => {}
    }
}

/// Group vectors (within 1e-6), most frequent first, at most `keep`.
fn rank_vec(vals: Vec<Vec3>, keep: usize) -> Vec<Vec3> {
    let mut groups: Vec<(Vec3, usize)> = vec![];
    for v in vals {
        match groups.iter_mut().find(|g| g.0.dist(v) < 1e-6) {
            Some(g) => g.1 += 1,
            None => groups.push((v, 1)),
        }
    }
    groups.sort_by(|a, b| b.1.cmp(&a.1).then(a.0.len().total_cmp(&b.0.len())));
    groups.truncate(keep);
    groups.into_iter().map(|g| g.0).collect()
}

fn vec_json(kind: &str, key: &str, v: Vec3) -> ck_json::Json {
    Obj::new().put("kind", kind).put(key, ck_json::Json::Arr(vec![n(v.x), n(v.y), n(v.z)])).build()
}

/// Linear pattern along any direction: the base translation is a full vector read off a pair of round faces.
fn linear_general(cx: &Cx, f: &SwFeature, sf: &[(usize, FaceId)], fp: &[FaceId]) -> Result<Found, String> {
    let m = cx.m;
    let n1 = count_dim(f, "D1").filter(|&c| c >= 2).ok_or("pattern count (D1) not listed")?;
    let n2 = if f.dims.iter().any(|d| d.name == "D4") { count_dim(f, "D2").unwrap_or(1) } else { 1 };
    let mut raw = vec![];
    for &p in fp {
        for &(_, s) in sf {
            lin_vectors(m.kind(s), m.kind(p), &mut raw);
        }
    }
    let mut cands = rank_vec(raw, 12);
    // a pair may be several steps apart
    let base = cands.clone();
    for v in base {
        for j in 2..n1.max(n2) {
            cands.push(v * (1.0 / j as f64));
        }
    }
    cands.sort_by(|a, b| a.len().total_cmp(&b.len()));
    if cands.is_empty() {
        return Err("no translation vector carries a seed face onto a pattern face".into());
    }
    let grid = |t1: Vec3, na: usize, t2: Option<Vec3>, nb: usize| -> Vec<Xform> {
        let mut xs = vec![];
        for j in 0..nb {
            for i in 0..na {
                if i == 0 && j == 0 {
                    continue;
                }
                let t = t1 * i as f64 + t2.map_or(Vec3::ZERO, |t2| t2 * j as f64);
                xs.push(Xform::translation(t));
            }
        }
        xs
    };
    let one = |pid: &str, seeds: &[String], t: Vec3, na: usize| {
        Obj::new()
            .put("type", "lpattern")
            .put("id", pid)
            .put("seeds", seeds_json(seeds))
            .put("dir", vec_json("vector", "dir", t.normalize()))
            .put("count", n(na as f64))
            .put("spacing", n(t.len()))
            .put("reverse", false)
    };
    if n2 <= 1 {
        let (t, xs) = choose(cands.iter().map(|&t| (t, grid(t, n1, None, 1))), m, sf, fp).ok_or("no translation vector reproduces the pattern's faces")?;
        let note = format!("linear pattern x{n1} along a free direction by {:.4}", t.len());
        let json = move |pid: &str, seeds: &[String]| one(pid, seeds, t, n1).build();
        return Ok(Found { xs, json: Box::new(json), note });
    }
    let mut counts = vec![(n1, n2)];
    if n1 != n2 {
        counts.push((n2, n1));
    }
    let cs = &cands;
    let pairs = counts.into_iter().flat_map(move |(na, nb)| cs.iter().flat_map(move |&t1| cs.iter().filter(move |t2| t1.cross(**t2).len() > 1e-6).map(move |&t2| (t1, t2, na, nb))));
    let ((t1, t2, na, nb), xs) = choose(pairs.map(|t| (t, grid(t.0, t.2, Some(t.1), t.3))), m, sf, fp).ok_or("no pair of translation vectors reproduces the pattern's faces")?;
    let json = move |pid: &str, seeds: &[String]| {
        one(pid, seeds, t1, na)
            .put("dir2", vec_json("vector", "dir", t2.normalize()))
            .put("count2", n(nb as f64))
            .put("spacing2", n(t2.len()))
            .put("reverse2", false)
            .build()
    };
    let note = format!("linear pattern {na} x {nb} along free directions (by {:.4} and {:.4})", t1.len(), t2.len());
    Ok(Found { xs, json: Box::new(json), note })
}

// ---------- circular ----------

/// Rotation angles about world axis `ax` that carry face `a` onto face `b` (positional surfaces only).
fn rot_angles(a: &Kind, b: &Kind, ax: Vec3, out: &mut Vec<f64>) {
    let mut push = |u: Vec3, v: Vec3| {
        let (u, v) = (reject(u, ax), reject(v, ax));
        if u.len() > 1e-6 && (u.len() - v.len()).abs() < 1e-6 {
            let phi = u.signed_angle(v, ax);
            if phi.abs() > 1e-7 {
                out.push(phi);
            }
        }
    };
    match (a, b) {
        (Kind::Cyl { o: oa, a: aa, r: ra, .. }, Kind::Cyl { o: ob, a: ab, r: rb, .. }) if (ra - rb).abs() < 1e-6 && par(*aa, ax) && par(*ab, ax) => push(*oa, *ob),
        (Kind::Sphere { c: ca, r: ra, .. }, Kind::Sphere { c: cb, r: rb, .. }) if (ra - rb).abs() < 1e-6 => push(*ca, *cb),
        (Kind::Torus { c: ca, major: ma, minor: ia, .. }, Kind::Torus { c: cb, major: mb, minor: ib, .. }) if (ma - mb).abs() < 1e-6 && (ia - ib).abs() < 1e-6 => push(*ca, *cb),
        (Kind::Plane { n: na, .. }, Kind::Plane { n: nb, .. }) if na.dot(ax).abs() < 1e-9 && nb.dot(ax).abs() < 1e-9 => push(*na, *nb),
        _ => {}
    }
}

fn circular(cx: &Cx, f: &SwFeature, sf: &[(usize, FaceId)], fp: &[FaceId]) -> Result<Found, String> {
    let m = cx.m;
    let cnt = count_dim(f, "D1").filter(|&c| c >= 2).ok_or("pattern count (D1) not listed")?;
    let mut cands: Vec<(usize, f64)> = vec![];
    for ai in 0..3 {
        let ax = axis_vec(ai);
        let mut vals = vec![];
        for &p in fp {
            for &(_, s) in sf {
                rot_angles(m.kind(s), m.kind(p), ax, &mut vals);
            }
        }
        for v in rank(vals, 16) {
            // a pair may be several steps apart
            for j in 1..cnt {
                cands.push((ai, v / j as f64));
            }
        }
    }
    cands.sort_by(|a, b| a.1.abs().total_cmp(&b.1.abs()));
    if cands.is_empty() {
        return Err("no rotation about a world axis carries a seed face onto a pattern face (axis not a world axis through the origin)".into());
    }
    let xsof = |c: (usize, f64)| -> Vec<Xform> { (1..cnt).map(|k| Xform::rotation(Vec3::ZERO, axis_vec(c.0), c.1 * k as f64)).collect() };
    let (c, xs) = choose(cands.iter().map(|&c| (c, xsof(c))), m, sf, fp).ok_or("no rotation about a world axis through the origin reproduces the pattern's faces")?;
    let deg = c.1.to_degrees();
    let json = move |pid: &str, seeds: &[String]| {
        Obj::new()
            .put("type", "cpattern")
            .put("id", pid)
            .put("seeds", seeds_json(seeds))
            .put("axis", axis_json(c.0))
            .put("count", n(cnt as f64))
            .put("angle", n(deg.abs()))
            .put("equal", false)
            .put("reverse", deg < 0.0)
            .build()
    };
    let note = format!("circular pattern x{cnt} about {} by {:.4} deg", ["x", "y", "z"][c.0], deg);
    Ok(Found { xs, json: Box::new(json), note })
}

/// Rotation angles about the axis through `c` along `ax` that carry face `a` onto face `b` (positional surfaces only).
fn rot_angles_at(a: &Kind, b: &Kind, c: Vec3, ax: Vec3, out: &mut Vec<f64>) {
    let mut push = |u: Vec3, v: Vec3| {
        let (u, v) = (reject(u, ax), reject(v, ax));
        if u.len() > 1e-6 && (u.len() - v.len()).abs() < 1e-6 {
            let phi = u.signed_angle(v, ax);
            if phi.abs() > 1e-7 {
                out.push(phi);
            }
        }
    };
    match (a, b) {
        (Kind::Cyl { o: oa, a: aa, r: ra, .. }, Kind::Cyl { o: ob, a: ab, r: rb, .. }) if (ra - rb).abs() < 1e-6 && par(*aa, ax) && par(*ab, ax) => push(*oa - c, *ob - c),
        (Kind::Sphere { c: ca, r: ra, .. }, Kind::Sphere { c: cb, r: rb, .. }) if (ra - rb).abs() < 1e-6 => push(*ca - c, *cb - c),
        (Kind::Torus { c: ca, major: ma, minor: ia, .. }, Kind::Torus { c: cb, major: mb, minor: ib, .. }) if (ma - mb).abs() < 1e-6 && (ia - ib).abs() < 1e-6 => push(*ca - c, *cb - c),
        (Kind::Plane { n: na, .. }, Kind::Plane { n: nb, .. }) if na.dot(ax).abs() < 1e-9 && nb.dot(ax).abs() < 1e-9 => push(*na, *nb),
        _ => {}
    }
}

/// Centre of the circle through three points in the plane across `d` (None when nearly collinear).
fn circumcentre(p: Vec3, q: Vec3, r: Vec3) -> Option<Vec3> {
    let (a, b) = (q - p, r - p);
    let w = a.cross(b);
    let den = 2.0 * w.dot(w);
    (den > 1e-12).then(|| p + (w.cross(a) * b.dot(b) + b.cross(w) * a.dot(a)) * (1.0 / den))
}

/// Circular pattern about any axis (a point and a direction): the direction comes from the axes of round faces (or a
/// world axis), the centre from points that lie on one circle about it, or from a round face that stays put.
fn circular_general(cx: &Cx, f: &SwFeature, sf: &[(usize, FaceId)], fp: &[FaceId]) -> Result<Found, String> {
    let m = cx.m;
    let cnt = count_dim(f, "D1").filter(|&c| c >= 2).ok_or("pattern count (D1) not listed")?;
    let faces: Vec<FaceId> = sf.iter().map(|s| s.1).chain(fp.iter().copied()).collect();
    // candidate directions: world axes, then the axes of round faces (one per parallel family)
    let mut dirs: Vec<Vec3> = (0..3).map(axis_vec).collect();
    for &g in &faces {
        if let Kind::Cyl { a, .. } | Kind::Cone { a, .. } | Kind::Torus { a, .. } = m.kind(g) {
            if let Some(a) = a.try_normalize() {
                if !dirs.iter().any(|d| par(*d, a)) {
                    dirs.push(a);
                }
            }
        }
    }
    dirs.truncate(8);
    let mut tried = 0usize;
    for d in dirs {
        // points across the axis: positional faces of the pattern, and the earlier faces that look like one of them
        // (same kind and size); round faces of earlier features that stay put give centres of their own
        let mut pts: Vec<Vec3> = vec![];
        let mut fixed: Vec<Vec3> = vec![];
        let point_of = |g: FaceId| -> Option<Vec3> {
            match m.kind(g) {
                Kind::Cyl { o, a, .. } | Kind::Cone { o, a, .. } if par(*a, d) => Some(reject(*o, d)),
                Kind::Torus { c, a, .. } if par(*a, d) => Some(reject(*c, d)),
                Kind::Sphere { c, .. } => Some(reject(*c, d)),
                _ => None,
            }
        };
        for &g in &faces {
            let Some(p) = point_of(g) else { continue };
            let in_fp = fp.contains(&g);
            if (in_fp || fp.iter().any(|&q| same_round(m.kind(g), m.kind(q)))) && !pts.iter().any(|q| q.dist(p) < 1e-6) {
                pts.push(p);
            }
            if !in_fp && matches!(m.kind(g), Kind::Cyl { .. } | Kind::Torus { .. }) && !fixed.iter().any(|q| q.dist(p) < 1e-6) {
                fixed.push(p);
            }
        }
        pts.truncate(24);
        fixed.truncate(24);
        let mut raw: Vec<Vec3> = vec![];
        for i in 0..pts.len() {
            for j in i + 1..pts.len() {
                for k in j + 1..pts.len() {
                    if let Some(c) = circumcentre(pts[i], pts[j], pts[k]) {
                        raw.push(c);
                    }
                }
            }
        }
        let mut centres = rank_vec(raw, 8);
        for c in fixed {
            if !centres.iter().any(|q| q.dist(c) < 1e-6) {
                centres.push(c);
            }
        }
        for c in centres {
            let mut vals = vec![];
            for &p in fp {
                for &(_, s) in sf {
                    rot_angles_at(m.kind(s), m.kind(p), c, d, &mut vals);
                }
            }
            let mut cands: Vec<f64> = vec![];
            for v in rank(vals, 16) {
                for j in 1..cnt {
                    cands.push(v / j as f64);
                }
            }
            cands.sort_by(|a, b| a.abs().total_cmp(&b.abs()));
            tried += cands.len();
            let xsof = |ang: f64| -> Vec<Xform> { (1..cnt).map(|k| Xform::rotation(c, d, ang * k as f64)).collect() };
            if let Some((ang, xs)) = choose(cands.iter().map(|&a| (a, xsof(a))), m, sf, fp) {
                let deg = ang.to_degrees();
                let json = move |pid: &str, seeds: &[String]| {
                    Obj::new()
                        .put("type", "cpattern")
                        .put("id", pid)
                        .put("seeds", seeds_json(seeds))
                        .put("axis", Obj::new().put("kind", "axis").put("point", vec_json_arr(c)).put("dir", vec_json_arr(d)).build())
                        .put("count", n(cnt as f64))
                        .put("angle", n(deg.abs()))
                        .put("equal", false)
                        .put("reverse", deg < 0.0)
                        .build()
                };
                let note = format!("circular pattern x{cnt} about a free axis by {:.4} deg", deg);
                return Ok(Found { xs, json: Box::new(json), note });
            }
        }
    }
    Err(format!("no rotation about any axis reproduces the pattern's faces ({tried} angles tried)"))
}

/// Two round faces of the same kind and size.
fn same_round(a: &Kind, b: &Kind) -> bool {
    match (a, b) {
        (Kind::Cyl { r: ra, convex: ca, .. }, Kind::Cyl { r: rb, convex: cb, .. }) => ca == cb && (ra - rb).abs() < 1e-6,
        (Kind::Cone { half: ha, .. }, Kind::Cone { half: hb, .. }) => (ha.abs() - hb.abs()).abs() < 1e-7,
        (Kind::Torus { major: ma, minor: ia, .. }, Kind::Torus { major: mb, minor: ib, .. }) => (ma - mb).abs() < 1e-6 && (ia - ib).abs() < 1e-6,
        (Kind::Sphere { r: ra, .. }, Kind::Sphere { r: rb, .. }) => (ra - rb).abs() < 1e-6,
        _ => false,
    }
}

fn vec_json_arr(v: Vec3) -> ck_json::Json {
    ck_json::Json::Arr(vec![n(v.x), n(v.y), n(v.z)])
}

// ---------- mirror ----------

/// Mirror planes `(unit normal, offset)` that carry face `a` onto face `b`.
fn refl_planes(a: &Kind, b: &Kind, out: &mut Vec<(Vec3, f64)>) {
    match (a, b) {
        (Kind::Plane { n: na, d: da }, Kind::Plane { n: nb, d: db }) => {
            if (*na + *nb).len() < 1e-9 {
                out.push((*na, (da - db) / 2.0));
            } else if (*na - *nb).len() > 1e-9 {
                let mv = (*na - *nb).normalize();
                let den = 2.0 * nb.dot(mv);
                if den.abs() > 1e-9 {
                    let x0 = *na * *da;
                    out.push((mv, x0.dot(mv) - (nb.dot(x0) - db) / den));
                }
            }
        }
        (Kind::Cyl { o: oa, a: aa, r: ra, .. }, Kind::Cyl { o: ob, a: ab, r: rb, .. }) if (ra - rb).abs() < 1e-6 && par(*aa, *ab) => {
            let w = reject(*ob - *oa, *ab);
            if w.len() > 1e-7 {
                let mv = w.normalize();
                out.push((mv, (*oa + *ob).dot(mv) / 2.0));
            }
        }
        (Kind::Sphere { c: ca, r: ra, .. }, Kind::Sphere { c: cb, r: rb, .. }) if (ra - rb).abs() < 1e-6 && (*cb - *ca).len() > 1e-7 => {
            let mv = (*cb - *ca).normalize();
            out.push((mv, (*ca + *cb).dot(mv) / 2.0));
        }
        (Kind::Torus { c: ca, major: ma, minor: ia, .. }, Kind::Torus { c: cb, major: mb, minor: ib, .. }) if (ma - mb).abs() < 1e-6 && (ia - ib).abs() < 1e-6 && (*cb - *ca).len() > 1e-7 => {
            let mv = (*cb - *ca).normalize();
            out.push((mv, (*ca + *cb).dot(mv) / 2.0));
        }
        (Kind::Cone { o: oa, a: aa, r0: ra, half: ha, .. }, Kind::Cone { o: ob, a: ab, r0: rb, half: hb, .. }) if (ha.abs() - hb.abs()).abs() < 1e-7 => {
            if let (Some(pa), Some(pb)) = (apex(*oa, *aa, *ra, *ha), apex(*ob, *ab, *rb, *hb)) {
                if (pb - pa).len() > 1e-7 {
                    let mv = (pb - pa).normalize();
                    out.push((mv, (pa + pb).dot(mv) / 2.0));
                }
            }
        }
        _ => {}
    }
}

fn mirror(cx: &mut Cx, sf: &[(usize, FaceId)], fp: &[FaceId]) -> Result<Found, String> {
    let m = cx.m;
    let mut raw: Vec<(Vec3, f64)> = vec![];
    for &p in fp {
        for &(_, s) in sf {
            refl_planes(m.kind(s), m.kind(p), &mut raw);
        }
    }
    // canonical sign, then count equal planes
    let mut groups: Vec<((Vec3, f64), usize)> = vec![];
    for (mv, c) in raw {
        let first = comps(mv).into_iter().find(|v| v.abs() > 1e-9).unwrap_or(1.0);
        let (mv, c) = if first < 0.0 { (-mv, -c) } else { (mv, c) };
        match groups.iter_mut().find(|g| (g.0 .0 - mv).len() < 1e-7 && (g.0 .1 - c).abs() < 1e-5) {
            Some(g) => g.1 += 1,
            None => groups.push(((mv, c), 1)),
        }
    }
    groups.sort_by(|a, b| b.1.cmp(&a.1));
    groups.truncate(24);
    let xof = |pl: (Vec3, f64)| vec![Xform::mirror(&Plane::new(pl.0 * pl.1, pl.0))];
    let (pl, xs) = choose(groups.iter().map(|g| (g.0, xof(g.0))), m, sf, fp).ok_or("no mirror plane reproduces the pattern's faces")?;
    let (mv, c) = pl;
    let plane = cx.frame(mv, mv * c).plane;
    let json = move |pid: &str, seeds: &[String]| Obj::new().put("type", "mirror").put("id", pid).put("seeds", seeds_json(seeds)).put("plane", plane.clone()).put("body", false).build();
    let note = format!("mirror in the plane n=({:.4}, {:.4}, {:.4}) at {:.4}", mv.x, mv.y, mv.z, c);
    Ok(Found { xs, json: Box::new(json), note })
}

#[cfg(test)]
mod nan_tests {
    use super::*;

    #[test]
    fn rank_survives_nan() {
        let r = rank(vec![f64::NAN, 2.0, 2.0, f64::NAN, -1.0], 3);
        assert_eq!(r[0], 2.0);
        assert_eq!(r.len(), 3);
    }

    #[test]
    fn rank_vec_survives_nan() {
        let n = Vec3::new(f64::NAN, 0.0, 0.0);
        let v = Vec3::new(1.0, 0.0, 0.0);
        let r = rank_vec(vec![n, v, v], 2);
        assert_eq!(r[0], v);
    }
}
