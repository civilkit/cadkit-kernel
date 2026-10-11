//! Assemble (ck-ops/src/boolean/assemble.rs): walk fragment loops in each
//! imprinted face, select fragments by operation, merge coincident
//! same-facing fragments, and build the result `Body`.
//!
//! Loop walk: half-edges = boundary pieces (one direction) + inner cut
//! edges (both directions); at every vertex the next half-edge is the
//! first one clockwise (about the outward normal) from the reversed
//! incoming direction, so every cycle has its face on the left (ck-topo
//! convention: loops CCW about the outward normal). Cycles with positive
//! oriented uv area are outer loops; negative ones are holes, assigned to
//! the smallest containing outer loop.
//!
//! Oracle-pinned output conventions (probes box−cylinder through/blind,
//! slot split, box∪box overlapping/flush; see boolean/README notes in
//! mod.rs):
//! - face order: A untouched, A modified, B untouched, B modified, then
//!   faces created by merging coincident fragments (`F2.C0` of a flush
//!   union comes last);
//! - a face split into k > 1 kept fragments gets `~1..~k`, numbered by
//!   lexicographic (x, y, z) order of the fragment centroid (GUESS: fits
//!   all probes; listed order = walk order, e.g. `F2.S.e3~2` before `~1`);
//! - edges are numbered in face-walk order of the output faces.
//!   GUESS/known mismatch: the oracle numbers kept whole edges first, then
//!   split pieces, then new intersection edges, per face group.

use super::classify::{in_loops, in_polygon, loop_polygon, outward, poly_area, Class, Solid};
use super::imprint::{ArrFace, Arrangement};
use super::Op;
use crate::intersect::ss::{self, SsResult};
use ck_geom::curve::Curve;
use ck_geom::curve2::Curve2;
use ck_geom::pcurve::{exact_pcurve, pcurve_of};
use ck_geom::surface::Surface;
use ck_geom::{Vec2, Vec3};
use ck_topo::body::{Body, FaceName};
use std::f64::consts::TAU;

/// Directed arrangement edge.
pub type He = (usize, bool);

/// One fragment of an imprinted face: loops of directed arrangement edges
/// (first = outer).
#[derive(Debug, Clone, PartialEq)]
pub struct Fragment {
    /// index into `Arrangement::faces`
    pub af: usize,
    pub loops: Vec<Vec<He>>,
    /// position of the first original boundary piece it contains
    pub order: usize,
    /// the whole original face (untouched)
    pub whole: bool,
    pub class: Option<Class>,
}

fn he_start(arr: &Arrangement, h: He) -> usize {
    let e = &arr.edges[h.0];
    if h.1 {
        e.v1
    } else {
        e.v0
    }
}
fn he_end(arr: &Arrangement, h: He) -> usize {
    let e = &arr.edges[h.0];
    if h.1 {
        e.v0
    } else {
        e.v1
    }
}
fn he_params(arr: &Arrangement, h: He) -> (f64, f64) {
    let e = &arr.edges[h.0];
    if h.1 {
        (e.t1, e.t0)
    } else {
        (e.t0, e.t1)
    }
}

/// Chord direction leaving the start of `h`.
fn he_out_dir(arr: &Arrangement, h: He) -> Vec3 {
    let e = &arr.edges[h.0];
    let (ta, tb) = he_params(arr, h);
    let p0 = e.curve.eval(ta);
    let d = e.curve.deriv(ta) * (tb - ta).signum();
    // chord over 0.1% of the piece: equals the tangent to O(1e-3 rad) and
    // still orders curves that are tangent at the vertex
    let p1 = e.curve.eval(ta + (tb - ta) * 1e-3);
    (p1 - p0).try_normalize().unwrap_or_else(|| d.normalize())
}

fn pieces<'a>(arr: &'a Arrangement, l: &[He]) -> Vec<(&'a Curve, f64, f64)> {
    l.iter()
        .map(|&h| {
            let (a, b) = he_params(arr, h);
            (&arr.edges[h.0].curve, a, b)
        })
        .collect()
}

/// Walk cycles over `hes` on (surface, sense).
pub fn walk(arr: &Arrangement, s: &Surface, sense: bool, hes: &[He]) -> Vec<Vec<He>> {
    let n = hes.len();
    let mut outs: std::collections::BTreeMap<usize, Vec<usize>> = Default::default();
    for (i, &h) in hes.iter().enumerate() {
        outs.entry(he_start(arr, h)).or_default().push(i);
    }
    let mut used = vec![false; n];
    let mut cycles = vec![];
    for start in 0..n {
        if used[start] {
            continue;
        }
        let mut cyc = vec![];
        let mut cur = start;
        let mut ok = false;
        for _ in 0..=n {
            used[cur] = true;
            cyc.push(hes[cur]);
            let h = hes[cur];
            let v = he_end(arr, h);
            let cands = outs.get(&v).cloned().unwrap_or_default();
            let p = arr.verts[v];
            let nrm = outward(s, sense, p);
            // reversed incoming direction
            let rin = he_out_dir(arr, (h.0, !h.1));
            let e1 = (rin - nrm * nrm.dot(rin)).try_normalize().unwrap_or(rin);
            let mut best: Option<(f64, usize)> = None;
            for &k in &cands {
                let g = hes[k];
                let d = he_out_dir(arr, g);
                let dp = (d - nrm * nrm.dot(d)).try_normalize().unwrap_or(d);
                let ccw = nrm.dot(e1.cross(dp)).atan2(e1.dot(dp));
                let mut cw = (-ccw).rem_euclid(TAU);
                if g.0 == h.0 && g.1 != h.1 || cw < 1e-9 {
                    cw = TAU + cw; // twin / same direction: last resort
                }
                if best.map_or(true, |b| cw < b.0) {
                    best = Some((cw, k));
                }
            }
            let Some((_, nx)) = best else { break };
            if nx == start {
                ok = true;
                break;
            }
            if used[nx] {
                break;
            }
            cur = nx;
        }
        if ok {
            cycles.push(cyc);
        }
    }
    cycles
}

/// Group cycles into fragments (outer + holes).
pub fn group(arr: &Arrangement, s: &Surface, sense: bool, cycles: Vec<Vec<He>>) -> Vec<Vec<Vec<He>>> {
    let sg = if sense { 1.0 } else { -1.0 };
    let polys: Vec<Vec<Vec2>> = cycles.iter().map(|c| loop_polygon(s, &pieces(arr, c))).collect();
    let areas: Vec<f64> = polys.iter().map(|p| poly_area(p) * sg).collect();
    let mut frags: Vec<Vec<Vec<He>>> = vec![];
    let mut outer_of: Vec<usize> = vec![]; // cycle index of each fragment's outer
    for (i, c) in cycles.iter().enumerate() {
        if areas[i] > 0.0 {
            frags.push(vec![c.clone()]);
            outer_of.push(i);
        }
    }
    for (i, c) in cycles.iter().enumerate() {
        if areas[i] > 0.0 {
            continue;
        }
        // probe just on the material side of the hole's first segment
        // (left about the outward normal); a vertex of the hole itself is
        // ambiguous against an outer with the same boundary (its twin disk)
        let (p0, p1) = (polys[i][0], polys[i][1 % polys[i].len()]);
        let d = (p1 - p0).normalize();
        let size = polys[i].iter().fold(0.0f64, |m, q| m.max(q.dist(p0))).max(1e-9);
        let probe = (p0 + p1) * 0.5 + d.perp() * (sg * 1e-6 * size);
        let host = outer_of
            .iter()
            .enumerate()
            .filter(|(_, &o)| in_polygon(s, probe, &polys[o]))
            .min_by(|a, b| areas[*a.1].partial_cmp(&areas[*b.1]).unwrap())
            .map(|(k, _)| k);
        match host {
            Some(k) => frags[k].push(c.clone()),
            None if !frags.is_empty() => frags[0].push(c.clone()), // GUESS fallback
            None => frags.push(vec![c.clone()]),
        }
    }
    frags
}

/// Fragments of arrangement face `fi`.
pub fn fragments(arr: &Arrangement, fi: usize) -> Vec<Fragment> {
    let af = &arr.faces[fi];
    if !af.touched() {
        return vec![Fragment { af: fi, loops: af.bound.clone(), order: 0, whole: true, class: None }];
    }
    let mut hes: Vec<He> = af.bound.iter().flatten().copied().collect();
    // inner edges, minus dangling ones
    let mut inner = af.inner.clone();
    loop {
        let mut deg: std::collections::HashMap<usize, usize> = Default::default();
        for &h in &hes {
            *deg.entry(he_start(arr, h)).or_default() += 1;
            *deg.entry(he_end(arr, h)).or_default() += 1;
        }
        for &e in &inner {
            *deg.entry(arr.edges[e].v0).or_default() += 2;
            *deg.entry(arr.edges[e].v1).or_default() += 2;
        }
        let before = inner.len();
        inner.retain(|&e| {
            let ed = &arr.edges[e];
            ed.v0 == ed.v1 || (deg[&ed.v0] > 2 && deg[&ed.v1] > 2)
        });
        if inner.len() == before {
            break;
        }
    }
    if inner.is_empty() {
        // only boundary edges split: still the whole face, at its own
        // position in the edge walk (oracle bracket prefix 8: F2.C1, whose
        // C1|S.e9 / C1|S.e8 edges a coplanar gusset splits, keeps #6..#11)
        return vec![Fragment { af: fi, loops: af.bound.clone(), order: 0, whole: true, class: None }];
    }
    for &e in &inner {
        hes.push((e, false));
        hes.push((e, true));
    }
    let cycles = walk(arr, &af.surface, af.sense, &hes);
    let groups = group(arr, &af.surface, af.sense, cycles);
    let flat: Vec<He> = af.bound.iter().flatten().copied().collect();
    let mut out: Vec<Fragment> = groups
        .into_iter()
        .enumerate()
        .map(|(k, loops)| {
            let order = loops.iter().flatten().filter_map(|h| flat.iter().position(|x| x == h)).min().unwrap_or(usize::MAX / 2 + k);
            Fragment { af: fi, loops, order, whole: false, class: None }
        })
        .collect();
    out.sort_by_key(|f| f.order);
    // rotate each loop to start at its first original boundary piece
    for f in &mut out {
        for l in &mut f.loops {
            if let Some((pos, _)) =
                l.iter().enumerate().filter_map(|(i, h)| flat.iter().position(|x| x == h).map(|p| (i, p))).min_by_key(|x| x.1)
            {
                l.rotate_left(pos);
            }
        }
    }
    out
}

/// An interior point of the fragment (on its surface) and the outward
/// normal there.
pub fn sample_point(arr: &Arrangement, f: &Fragment) -> Option<(Vec3, Vec3)> {
    let af = &arr.faces[f.af];
    let s = &af.surface;
    let seam_only = |l: &Vec<He>| l.iter().all(|&(e, r)| l.contains(&(e, !r)));
    if f.loops.iter().all(|l| l.is_empty() || seam_only(l)) {
        // closed surface without boundary (full sphere / torus, or a
        // revolve sphere bounded only by its meridian seam)
        let ((u0, u1), (v0, v1)) = s.domain();
        let (u, v) = (u0 + 0.37 * (u1 - u0), v0 + 0.41 * (v1 - v0));
        let q = s.eval(u, v);
        return Some((q, outward(s, af.sense, q)));
    }
    let polys: Vec<Vec<Vec2>> = f.loops.iter().map(|l| loop_polygon(s, &pieces(arr, l))).collect();
    let mut cand: Vec<He> = f.loops[0].clone();
    let len = |h: &He| {
        let e = &arr.edges[h.0];
        e.curve.eval(e.t0).dist(e.curve.eval(e.t1)).max(e.curve.eval(e.t0).dist(e.curve.eval(0.5 * (e.t0 + e.t1))))
    };
    cand.sort_by(|a, b| len(b).partial_cmp(&len(a)).unwrap());
    let all: Vec<Vec3> = f.loops.iter().flatten().map(|h| arr.verts[he_start(arr, *h)]).collect();
    let mut size = 0.0f64;
    for p in &all {
        for q in &all {
            size = size.max(p.dist(*q));
        }
    }
    let size = size.max(len(&cand[0]));
    for h in &cand {
        let e = &arr.edges[h.0];
        let (ta, tb) = he_params(arr, *h);
        for frac in [0.5, 0.37, 0.63] {
            let tm = ta + (tb - ta) * frac;
            let pm = e.curve.eval(tm);
            let td = (e.curve.deriv(tm) * (tb - ta).signum()).normalize();
            let n = outward(s, af.sense, pm);
            let left = n.cross(td).normalize();
            for eps in [3e-2, 1e-2, 3e-3, 1e-3, 1e-4] {
                let q0 = pm + left * (eps * size);
                let (u, v, q) = s.closest_point(q0);
                if !in_loops(s, Vec2::new(u, v), &polys) {
                    continue;
                }
                // keep away from the other boundary pieces
                let near = f.loops.iter().flatten().any(|g| {
                    let ge = &arr.edges[g.0];
                    super::imprint::dist_in_range(&ge.curve, (ge.t0, ge.t1), q).1 < eps * size * 0.25
                });
                if near {
                    continue;
                }
                return Some((q, outward(s, af.sense, q)));
            }
        }
    }
    // degenerate loops (a revolve sphere: one meridian seam between two
    // poles): a grid point of the loops' uv box that is inside
    let (mut lo, mut hi) = (Vec2::new(f64::INFINITY, f64::INFINITY), Vec2::new(f64::NEG_INFINITY, f64::NEG_INFINITY));
    for p in polys.iter().flatten() {
        lo = Vec2::new(lo.x.min(p.x), lo.y.min(p.y));
        hi = Vec2::new(hi.x.max(p.x), hi.y.max(p.y));
    }
    if lo.x.is_finite() && hi.x.is_finite() {
        for (fu, fv) in [(0.37, 0.41), (0.61, 0.53), (0.23, 0.67), (0.5, 0.5)] {
            let uv = Vec2::new(lo.x + (hi.x - lo.x) * fu, lo.y + (hi.y - lo.y) * fv);
            if in_loops(s, uv, &polys) {
                let q = s.eval(uv.x, uv.y);
                return Some((q, outward(s, af.sense, q)));
            }
        }
    }
    None
}

/// Keep / flip decision for a fragment of side `side`.
fn keep(op: Op, side: usize, c: Class) -> Option<bool> {
    use Class::*;
    match (op, side, c) {
        (Op::Union, _, Out) | (Op::Union, 0, OnSame) => Some(false),
        (Op::Intersect, _, In) | (Op::Intersect, 0, OnSame) => Some(false),
        (Op::Cut, 0, Out) | (Op::Cut, 0, OnOpp) => Some(false),
        (Op::Cut, 1, In) => Some(true),
        _ => None,
    }
}

fn coincident(a: &Surface, b: &Surface, tol: f64) -> bool {
    match (a, b) {
        (Surface::Plane { frame: f }, Surface::Plane { frame: g }) => {
            f.z.cross(g.z).len() < 1e-9 && (g.o - f.o).dot(f.z).abs() <= tol
        }
        _ => ss::intersect(a, b, None, tol) == SsResult::Coincident,
    }
}

/// Result face before body construction.
#[derive(Debug, Clone)]
pub struct OutFace {
    pub surface: Surface,
    pub sense: bool,
    pub name: FaceName,
    pub loops: Vec<Vec<He>>,
    /// output group (0 A untouched, 1 A modified, 2 B untouched, 3 B modified, 4 merged)
    pub group: usize,
    pub key: (usize, usize, usize),
    /// Edge-numbering walk items: (key, loops). Merged faces contribute
    /// their member fragments at the members' own positions (the oracle
    /// numbers edges before merging coincident faces; probe `union`).
    pub walk: Vec<((usize, usize, usize), Vec<Vec<He>>)>,
}

/// Classify fragments against the other body and select/flip them.
pub fn select(arr: &Arrangement, bodies: [&Body; 2], op: Op) -> Result<Vec<OutFace>, String> {
    let tol = arr.tol;
    let mut kept: Vec<(Fragment, bool)> = vec![];
    let mut n_frags: Vec<usize> = vec![0; arr.faces.len()];
    let solids = [Solid::new(bodies[0], tol), Solid::new(bodies[1], tol)];
    for fi in 0..arr.faces.len() {
        let frs = fragments(arr, fi);
        n_frags[fi] = frs.len();
        for mut fr in frs {
            let side = arr.faces[fi].side;
            let (q, n) = sample_point(arr, &fr).ok_or_else(|| "booleanFailed".to_string())?;
            let c = solids[1 - side].classify(q, n);
            fr.class = Some(c);
            if let Some(flip) = keep(op, side, c) {
                kept.push((fr, flip));
            }
        }
    }
    // directed loops after flipping
    let loops_of = |fr: &Fragment, flip: bool| -> Vec<Vec<He>> {
        if !flip {
            return fr.loops.clone();
        }
        // plain reversal; the edge walk of the boolean itself still uses
        // the unflipped loops (oracle bracket slot cut: kept tool side
        // F6.S.e3 numbers F6.S.e3|F6.S.e6 #44 before F6.S.e10|F6.S.e3 #45;
        // the next boolean walks the stored, reversed loop: e10|e3 first)
        fr.loops.iter().map(|l| l.iter().rev().map(|&(e, r)| (e, !r)).collect()).collect()
    };
    // ---- merge coincident same-facing fragments sharing an edge
    let n = kept.len();
    let mut parent: Vec<usize> = (0..n).collect();
    fn find(p: &mut Vec<usize>, i: usize) -> usize {
        let mut i = i;
        while p[i] != i {
            p[i] = p[p[i]];
            i = p[i];
        }
        i
    }
    let edges_of: Vec<Vec<usize>> = kept.iter().map(|(f, _)| f.loops.iter().flatten().map(|h| h.0).collect()).collect();
    for i in 0..n {
        for j in i + 1..n {
            if !edges_of[i].iter().any(|e| edges_of[j].contains(e)) {
                continue;
            }
            let (fi, fj) = (&arr.faces[kept[i].0.af], &arr.faces[kept[j].0.af]);
            let si = fi.sense ^ kept[i].1;
            let sj = fj.sense ^ kept[j].1;
            if !coincident(&fi.surface, &fj.surface, tol) {
                continue;
            }
            let p = arr.verts[he_start(arr, kept[i].0.loops[0][0])];
            let ni = outward(&fi.surface, si, p);
            let nj = outward(&fj.surface, sj, p);
            if ni.dot(nj) <= 0.0 {
                continue;
            }
            let (a, b) = (find(&mut parent, i), find(&mut parent, j));
            if a != b {
                parent[b.max(a)] = a.min(b);
            }
        }
    }
    // ---- region suffixes: count kept, unmerged fragments per original face
    let mut roots: Vec<usize> = (0..n).map(|i| find(&mut parent, i)).collect();
    let merged: Vec<bool> = (0..n).map(|i| roots.iter().filter(|&&r| r == roots[i]).count() > 1).collect();
    let mut out: Vec<OutFace> = vec![];
    let mut per_face: std::collections::BTreeMap<usize, Vec<usize>> = Default::default();
    for i in 0..n {
        if !merged[i] {
            per_face.entry(kept[i].0.af).or_default().push(i);
        }
    }
    let mut region: Vec<Option<u32>> = vec![None; n];
    for (_, idx) in per_face.iter() {
        if idx.len() < 2 {
            continue;
        }
        let mut cs: Vec<(Vec3, usize)> = idx
            .iter()
            .map(|&i| {
                let pts: Vec<Vec3> = kept[i].0.loops[0].iter().map(|h| arr.verts[he_start(arr, *h)]).collect();
                let c = pts.iter().fold(Vec3::ZERO, |a, &p| a + p) / pts.len() as f64;
                (c, i)
            })
            .collect();
        let key = |v: Vec3| (v.x, v.y, v.z);
        cs.sort_by(|a, b| {
            let (x, y) = (key(a.0), key(b.0));
            let cmp = |p: f64, q: f64| if (p - q).abs() <= 1e-9 { std::cmp::Ordering::Equal } else { p.partial_cmp(&q).unwrap() };
            cmp(x.0, y.0).then(cmp(x.1, y.1)).then(cmp(x.2, y.2))
        });
        for (k, (_, i)) in cs.into_iter().enumerate() {
            region[i] = Some(k as u32 + 1);
        }
    }
    for i in 0..n {
        if merged[i] {
            continue;
        }
        let (fr, flip) = &kept[i];
        let af: &ArrFace = &arr.faces[fr.af];
        let mut name = af.name.clone();
        if let (Some(k), None) = (region[i], name.region) {
            name.region = Some(k); // GUESS: nested regions keep the original
        }
        let group = af.side * 2 + if fr.whole { 0 } else { 1 };
        out.push(OutFace {
            surface: af.surface.clone(),
            sense: af.sense ^ flip,
            name,
            loops: loops_of(fr, *flip),
            group,
            key: (group, fr.af, fr.order),
            walk: vec![((group, fr.af, fr.order), fr.loops.clone())],
        });
    }
    // merged groups
    roots.dedup();
    let mut done = vec![];
    for i in 0..n {
        let r = find(&mut parent, i);
        if !merged[i] || done.contains(&r) {
            continue;
        }
        done.push(r);
        let members: Vec<usize> = (0..n).filter(|&j| find(&mut parent, j) == r).collect();
        let lead = members[0];
        let af = &arr.faces[kept[lead].0.af];
        let sense = af.sense ^ kept[lead].1;
        // all fragments of ONE face rejoined (e.g. a touching tool imprint
        // that is kept on both sides): restore the original face
        if members.iter().all(|&m| kept[m].0.af == kept[lead].0.af) && members.len() == n_frags[kept[lead].0.af] {
            let flip = kept[lead].1;
            let whole = Fragment { af: kept[lead].0.af, loops: af.bound.clone(), order: 0, whole: true, class: None };
            // listed with the merged faces (oracle mirrorTwo: F2.C0, which
            // a boss standing on it splits and rejoins, comes last); its
            // edges are still walked at the face's own position
            let group = af.side * 2;
            out.push(OutFace {
                surface: af.surface.clone(),
                sense,
                name: af.name.clone(),
                loops: loops_of(&whole, flip),
                group: 4,
                key: (4, whole.af, 0),
                walk: vec![((group, whole.af, 0), whole.loops.clone())],
            });
            continue;
        }
        let mut hes: Vec<He> = vec![];
        for &m in &members {
            hes.extend(loops_of(&kept[m].0, kept[m].1).into_iter().flatten());
        }
        let all = hes.clone();
        hes.retain(|&(e, r)| !all.contains(&(e, !r)));
        let cycles = walk(arr, &af.surface, sense, &hes);
        let walk_items: Vec<_> = members
            .iter()
            .map(|&m| {
                let fr = &kept[m].0;
                let g = arr.faces[fr.af].side * 2 + if fr.whole { 0 } else { 1 };
                ((g, fr.af, fr.order), fr.loops.clone())
            })
            .collect();
        for (k, loops) in group(arr, &af.surface, sense, cycles).into_iter().enumerate() {
            out.push(OutFace {
                surface: af.surface.clone(),
                sense,
                name: af.name.clone(),
                loops,
                group: 4,
                key: (4, kept[lead].0.af, k),
                walk: if k == 0 { walk_items.clone() } else { vec![] },
            });
        }
    }
    out.sort_by_key(|f| f.key);
    Ok(out)
}

pub(crate) fn shift2(c: &Curve2, d: Vec2) -> Curve2 {
    match c {
        Curve2::Line { o, d: dd } => Curve2::Line { o: *o + d, d: *dd },
        Curve2::Circle { c, x, r, ccw } => Curve2::Circle { c: *c + d, x: *x, r: *r, ccw: *ccw },
        Curve2::Ellipse { c, x, a, b, ccw } => Curve2::Ellipse { c: *c + d, x: *x, a: *a, b: *b, ccw: *ccw },
        Curve2::BSpline(b) => Curve2::BSpline(b.map(|p| p + d)),
    }
}

/// Build the result body (one lump per connected shell; voids attach to
/// their host via ck-topo `rebuild_lumps`).
/// Join collinear line edges left meeting at a 2-valent vertex between the
/// same two faces (a coplanar merge removed what separated them), the
/// original's simplify pass: walking edges in numbering order, an edge
/// absorbs the other edge at its END vertex (oracle mirrorBody: box with
/// its mirror across x = 0 gives C0|S.e3 [40,0,-20]->[-40,0,-20] as ONE
/// edge #6, while C0|S.e1 [0,0,0]->[40,0,0] and its mirror
/// [0,0,0]->[-40,0,0], which only START there, stay two).
fn join_collinear(arr: &Arrangement, faces: &[OutFace]) -> (Arrangement, Vec<OutFace>) {
    let mut arr = arr.clone();
    let mut faces = faces.to_vec();
    let n = arr.edges.len();
    let mut users: Vec<Vec<usize>> = vec![vec![]; n];
    for (k, of) in faces.iter().enumerate() {
        for &(e, _) in of.loops.iter().flatten() {
            if !users[e].contains(&k) {
                users[e].push(k);
            }
        }
    }
    let mut inc: std::collections::HashMap<usize, Vec<usize>> = Default::default();
    for e in 0..n {
        if users[e].is_empty() {
            continue;
        }
        inc.entry(arr.edges[e].v0).or_default().push(e);
        inc.entry(arr.edges[e].v1).or_default().push(e);
    }
    let mut items: Vec<&((usize, usize, usize), Vec<Vec<He>>)> = faces.iter().flat_map(|f| f.walk.iter()).collect();
    items.sort_by_key(|x| x.0);
    let mut order: Vec<usize> = vec![];
    for (_, loops) in items {
        for &(e, _) in loops.iter().flatten() {
            if !users[e].is_empty() && !order.contains(&e) {
                order.push(e);
            }
        }
    }
    let line = |c: &Curve| match c {
        Curve::Line { o, d } => Some((*o, *d)),
        _ => None,
    };
    let mut dead = vec![false; n];
    let mut joined: Vec<(usize, usize)> = vec![];
    for &e in &order {
        if dead[e] {
            continue;
        }
        loop {
            let v = arr.edges[e].v1;
            let at = &inc[&v];
            if at.len() != 2 || arr.edges[e].v0 == v {
                break;
            }
            let f = if at[0] == e { at[1] } else { at[0] };
            if f == e || dead[f] {
                break;
            }
            let (Some((o, d)), Some((_, d2))) = (line(&arr.edges[e].curve), line(&arr.edges[f].curve)) else { break };
            let mut ue = users[e].clone();
            let mut uf = users[f].clone();
            ue.sort();
            uf.sort();
            if ue != uf || d.cross(d2).len() > 1e-12 || (arr.verts[arr.edges[f].v0] - o).cross(d).len() > arr.tol {
                break;
            }
            let far = if arr.edges[f].v0 == v { arr.edges[f].v1 } else { arr.edges[f].v0 };
            let t = (arr.verts[far] - o).dot(d);
            if t <= arr.edges[e].t1 {
                break;
            }
            arr.edges[e].t1 = t;
            arr.edges[e].v1 = far;
            dead[f] = true;
            joined.push((e, f));
            // the far vertex now meets e instead of f
            let list = inc.get_mut(&far).unwrap();
            for x in list.iter_mut() {
                if *x == f {
                    *x = e;
                }
            }
        }
    }
    if joined.is_empty() {
        return (arr, faces);
    }
    for of in &mut faces {
        for l in of.loops.iter_mut() {
            l.retain(|h| !dead[h.0]);
        }
        for (_, ls) in of.walk.iter_mut() {
            for l in ls.iter_mut() {
                l.retain(|h| !dead[h.0]);
            }
        }
    }
    (arr, faces)
}

pub fn build(arr: &Arrangement, faces: &[OutFace]) -> Body {
    let (arr2, faces2) = join_collinear(arr, faces);
    let (arr, faces) = (&arr2, &faces2[..]);
    let tol = arr.tol;
    let mut b = Body::new();
    let lump = b.add_lump();
    let shell = b.add_shell(lump);
    let mut vmap: Vec<Option<ck_topo::body::VertexId>> = vec![None; arr.verts.len()];
    let mut emap: Vec<Option<ck_topo::body::EdgeId>> = vec![None; arr.edges.len()];
    // edge (and vertex) creation order = walk order
    let mut used = vec![false; arr.edges.len()];
    for of in faces {
        for &(e, _) in of.loops.iter().flatten() {
            used[e] = true;
        }
    }
    let mut items: Vec<&((usize, usize, usize), Vec<Vec<He>>)> = faces.iter().flat_map(|f| f.walk.iter()).collect();
    items.sort_by_key(|x| x.0);
    for (_, loops) in items {
        for &(ei, _) in loops.iter().flatten() {
            if !used[ei] || emap[ei].is_some() {
                continue;
            }
            let ae = &arr.edges[ei];
            let mut vid = |vi: usize, b: &mut Body| match vmap[vi] {
                Some(v) => v,
                None => {
                    let v = b.add_vertex(arr.verts[vi]);
                    vmap[vi] = Some(v);
                    v
                }
            };
            let v0 = vid(ae.v0, &mut b);
            let v1 = vid(ae.v1, &mut b);
            emap[ei] = Some(b.add_edge(ae.curve.clone(), ae.t0, ae.t1, v0, v1));
        }
    }
    for of in faces {
        let f = b.add_face(shell, of.surface.clone(), of.sense, of.name.clone());
        let (pu, pv) = of.surface.periods();
        for l in &of.loops {
            let lp = b.add_loop(f);
            let mut prev_end: Option<Vec2> = None;
            for &(ei, rev) in l {
                let ae = &arr.edges[ei];
                let eid = match emap[ei] {
                    Some(e) => e,
                    None => {
                        let mut vid = |vi: usize, b: &mut Body| match vmap[vi] {
                            Some(v) => v,
                            None => {
                                let v = b.add_vertex(arr.verts[vi]);
                                vmap[vi] = Some(v);
                                v
                            }
                        };
                        let v0 = vid(ae.v0, &mut b);
                        let v1 = vid(ae.v1, &mut b);
                        let e = b.add_edge(ae.curve.clone(), ae.t0, ae.t1, v0, v1);
                        emap[ei] = Some(e);
                        e
                    }
                };
                let pc = exact_pcurve(&of.surface, &ae.curve, tol * 10.0)
                    .or_else(|| pcurve_of(&of.surface, &ae.curve, ae.t0, ae.t1, tol).ok());
                let pc = pc.map(|pc| {
                    let (ta, tb) = if rev { (ae.t1, ae.t0) } else { (ae.t0, ae.t1) };
                    let mut pc = pc;
                    if let Some(prev) = prev_end {
                        let s = pc.eval(ta);
                        let du = pu.map_or(0.0, |p| ((prev.x - s.x) / p).round() * p);
                        let dv = pv.map_or(0.0, |p| ((prev.y - s.y) / p).round() * p);
                        if du != 0.0 || dv != 0.0 {
                            pc = shift2(&pc, Vec2::new(du, dv));
                        }
                    }
                    prev_end = Some(pc.eval(tb));
                    pc
                });
                b.add_coedge(lp, eid, rev, pc);
            }
            // a loop made only of seam twins (revolve sphere meridian): the
            // twins' pcurves must lie one period apart, the loop CCW about
            // the outward normal (positive uv area when sense)
            if !l.is_empty() && l.iter().all(|&(e, r)| l.contains(&(e, !r))) {
                if let Some(p) = pu.or(pv) {
                    let cs = b.lp(lp).coedges.clone();
                    let mut seen: Vec<usize> = vec![];
                    for (k, &(e, _)) in l.iter().enumerate() {
                        if seen.contains(&e) {
                            if let Some(pc) = b.coedges[cs[k].idx()].pcurve.clone() {
                                let d = if pu.is_some() { Vec2::new(p, 0.0) } else { Vec2::new(0.0, p) };
                                b.coedges[cs[k].idx()].pcurve = Some(shift2(&pc, d));
                            }
                        } else {
                            seen.push(e);
                        }
                    }
                    if (ck_topo::orient::loop_uv_area(&b, lp) > 0.0) != of.sense {
                        for (k, &(e, _)) in l.iter().enumerate() {
                            if l.iter().position(|&(x, _)| x == e) != Some(k) {
                                if let Some(pc) = b.coedges[cs[k].idx()].pcurve.clone() {
                                    let d = if pu.is_some() { Vec2::new(-2.0 * p, 0.0) } else { Vec2::new(0.0, -2.0 * p) };
                                    b.coedges[cs[k].idx()].pcurve = Some(shift2(&pc, d));
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    ck_topo::region::rebuild_lumps(&mut b);
    b
}

/// Wire-level edge chains (seam-split arcs = one oracle edge; probe
/// `boss`: the boss's bottom arc outside the box is a single edge
/// `F2.C0|F4.S.e1` running through the seam point (43,0,-10)). Moved to
/// ck-topo [`Body::wire_chains`], which `Body::edges_json` now uses.
pub fn wire_chains(b: &Body) -> Vec<Vec<ck_topo::body::EdgeId>> {
    b.wire_chains()
}

/// Copy of `b` with faces in `order` (live faces not listed are dropped);
/// vertices and edges are re-created in face-walk order, pcurves kept.
pub fn rebuild_in_order(b: &Body, order: &[ck_topo::body::FaceId]) -> Body {
    let mut out = Body::new();
    let lump = out.add_lump();
    let shell = out.add_shell(lump);
    let mut vmap = vec![None; b.vertices.len()];
    let mut emap = vec![None; b.edges.len()];
    for &f in order {
        let face = b.face(f);
        let nf = out.add_face(shell, face.surface.clone(), face.sense, face.name.clone());
        for &l in &face.loops {
            let nl = out.add_loop(nf);
            for &c in &b.lp(l).coedges {
                let ce = b.coedge(c).clone();
                let e = ce.edge;
                let ne = match emap[e.idx()] {
                    Some(x) => x,
                    None => {
                        let ed = b.edge(e).clone();
                        let mut vid = |v: ck_topo::body::VertexId, o: &mut Body| match vmap[v.idx()] {
                            Some(x) => x,
                            None => {
                                let x = o.add_vertex(b.vertex(v).p);
                                vmap[v.idx()] = Some(x);
                                x
                            }
                        };
                        let v0 = vid(ed.v0, &mut out);
                        let v1 = vid(ed.v1, &mut out);
                        let x = out.add_edge(ed.curve, ed.t0, ed.t1, v0, v1);
                        emap[e.idx()] = Some(x);
                        x
                    }
                };
                out.add_coedge(nl, ne, ce.rev, ce.pcurve);
            }
        }
    }
    ck_topo::region::rebuild_lumps(&mut out);
    out
}
