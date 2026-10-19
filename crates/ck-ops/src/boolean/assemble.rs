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
use ck_geom::pcurve::{exact_pcurve, pcurve_of, pcurve_of_dense};
use ck_geom::surface::Surface;
use ck_geom::{Vec2, Vec3};
use ck_topo::body::{Body, FaceName, VertexId};
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
        } else {
            super::bool_dbg!("walk: open chain of {} half-edges dropped: {:?}", cyc.len(), cyc);
        }
    }
    cycles
}

/// Group cycles into fragments (outer + holes).
pub fn group(arr: &Arrangement, s: &Surface, sense: bool, cycles: Vec<Vec<He>>) -> Vec<Vec<Vec<He>>> {
    let sg = if sense { 1.0 } else { -1.0 };
    // A seam-only cycle (a revolved sphere's meridian, every half-edge walked
    // both ways) is the outer boundary of the face's whole parameter domain; as
    // sampled it is a zero-area spike, so it would never host the circles that
    // cut the sphere (the cap and the rest would merge into one fragment).
    let ((du0, du1), (dv0, dv1)) = s.domain();
    let whole = [du0, du1, dv0, dv1].iter().all(|x| x.is_finite());
    let seam_cycle = |c: &Vec<He>| whole && !c.is_empty() && c.iter().all(|&(e, r)| c.contains(&(e, !r)));
    let polys: Vec<Vec<Vec2>> = cycles
        .iter()
        .map(|c| {
            if seam_cycle(c) {
                vec![Vec2::new(du0, dv0), Vec2::new(du1, dv0), Vec2::new(du1, dv1), Vec2::new(du0, dv1)]
            } else {
                loop_polygon(s, &pieces(arr, c), sense)
            }
        })
        .collect();
    let areas: Vec<f64> = polys.iter().zip(&cycles).map(|(p, c)| if seam_cycle(c) { poly_area(p).abs() } else { poly_area(p) * sg }).collect();
    super::bool_dbg!("group: {} cycles, areas {:?}", cycles.len(), areas);
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
#[inline(never)]
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
            let keep = ed.v0 == ed.v1 || (deg[&ed.v0] > 2 && deg[&ed.v1] > 2);
            if !keep {
                let (p0, p1) = (arr.verts[ed.v0], arr.verts[ed.v1]);
                let near = |v: usize| {
                    let p = arr.verts[v];
                    arr.verts.iter().enumerate().filter(|(i, _)| *i != v).map(|(i, q)| (q.dist(p), i)).fold((f64::INFINITY, 0), |a, b| if b.0 < a.0 { b } else { a })
                };
                super::bool_dbg!(
                    "face {fi}: dangling inner edge {e} dropped (v{} {:?} deg {} nearest {:?}; v{} {:?} deg {} nearest {:?})",
                    ed.v0, p0, deg[&ed.v0], near(ed.v0), ed.v1, p1, deg[&ed.v1], near(ed.v1)
                );
            }
            keep
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

/// [`sample_point`] of a whole (untouched) face, memoised by the exact content of everything it reads: the
/// surface and sense, and per loop the directed pieces (curve, range, end vertices; repeated edges by their
/// first position). An untouched face reaches the next boolean of a chain with the same edges, so it is
/// sampled once (its uv polygons are the costly part); a split fragment is sampled as before.
fn sample_point_whole(arr: &Arrangement, f: &Fragment) -> Option<(Vec3, Vec3)> {
    if !f.whole {
        return sample_point(arr, f);
    }
    thread_local! {
        static MEMO: std::cell::RefCell<ck_core::memo::Memo<Option<(Vec3, Vec3)>>> = std::cell::RefCell::new(ck_core::memo::Memo::new(1 << 21));
    }
    let af = &arr.faces[f.af];
    let mut key = Vec::with_capacity(64);
    af.surface.push_key(&mut key);
    key.extend([af.sense as u64, f.loops.len() as u64]);
    let mut seen: Vec<usize> = vec![];
    for l in &f.loops {
        key.push(l.len() as u64);
        for &(e, rev) in l {
            match seen.iter().position(|&x| x == e) {
                Some(i) => key.push((i as u64) << 2 | 2 | rev as u64),
                None => {
                    key.push((seen.len() as u64) << 2 | rev as u64);
                    seen.push(e);
                    let ed = &arr.edges[e];
                    ed.curve.push_key(&mut key);
                    key.extend([ed.t0.to_bits(), ed.t1.to_bits()]);
                    ck_geom::curve::push_vec3(&mut key, arr.verts[ed.v0]);
                    ck_geom::curve::push_vec3(&mut key, arr.verts[ed.v1]);
                }
            }
        }
    }
    ck_core::memo::cached(&MEMO, key, || sample_point(arr, f))
}

/// An interior point of the fragment (on its surface) and the outward
/// normal there.
#[inline(never)]
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
    // A seam-only loop (a revolved sphere's meridian) is the outer boundary of
    // a face that spans its whole parameter domain: as a polygon it is a
    // spike that encloses nothing, so a fragment "whole sphere minus a cap"
    // (seam loop + the cap's circle) would count the CAP as its inside under
    // the even-odd rule. Use the domain rectangle for it.
    let ((du0, du1), (dv0, dv1)) = s.domain();
    let whole = [du0, du1, dv0, dv1].iter().all(|x| x.is_finite());
    let polys: Vec<Vec<Vec2>> = f
        .loops
        .iter()
        .map(|l| {
            if whole && !l.is_empty() && seam_only(l) {
                vec![Vec2::new(du0, dv0), Vec2::new(du1, dv0), Vec2::new(du1, dv1), Vec2::new(du0, dv1)]
            } else {
                loop_polygon(s, &pieces(arr, l), af.sense)
            }
        })
        .collect();
    let mut cand: Vec<He> = f.loops.iter().find(|l| !l.is_empty() && !seam_only(l)).unwrap_or(&f.loops[0]).clone();
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
                    let thr = eps * size * 0.25;
                    ge.curve.may_be_within(q, thr) && super::imprint::dist_in_range(&ge.curve, (ge.t0, ge.t1), q).1 < thr
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
    /// [`ck_topo::body::Face::absorbed`] carried forward (member names merged
    /// into this face plus whatever the members had absorbed).
    pub absorbed: Vec<String>,
    pub loops: Vec<Vec<He>>,
    /// output group (0 A untouched, 1 A modified, 2 B untouched, 3 B modified, 4 merged)
    pub group: usize,
    pub key: (usize, usize, usize),
    /// Edge-numbering walk items: (key, loops). Merged faces contribute
    /// their member fragments at the members' own positions (the oracle
    /// numbers edges before merging coincident faces; probe `union`).
    pub walk: Vec<((usize, usize, usize), Vec<Vec<He>>)>,
}

/// The edge is traversed in both directions by one boundary loop of the
/// original face: the seam of a periodic face, wherever `relocate_seams` put it
/// (a seam moved off u = 0 is not recognised by its parameter).
fn bound_twice(af: &super::imprint::ArrFace, e: usize) -> bool {
    af.bound.iter().any(|l| l.contains(&(e, false)) && l.contains(&(e, true)))
}

/// Is arrangement edge `e` the seam (u = 0 / 2π generator) of periodic `s`?
fn is_seam_edge(arr: &Arrangement, s: &Surface, e: usize) -> bool {
    let ed = &arr.edges[e];
    if !matches!(ed.curve, Curve::Line { .. }) || !matches!(s, Surface::Cylinder { .. } | Surface::Cone { .. }) {
        return false;
    }
    [0.25, 0.5, 0.75].iter().all(|&k| {
        let u = s.closest_param(ed.curve.eval(ed.t0 + (ed.t1 - ed.t0) * k)).0;
        u.min(TAU - u) < 1e-6
    })
}

/// Classify fragments against the other body and select/flip them.
pub fn select(arr: &Arrangement, bodies: [&Body; 2], op: Op) -> Result<Vec<OutFace>, String> {
    select_class_tol(arr, bodies, op, arr.tol)
}

/// [`select`] classifying fragments with `tol` as the "on the other body's surface" band instead of the
/// arrangement's welding tolerance. A blend tool built at a loose welding tolerance whose rolling-ball surface
/// is tangent to the body's faces along the contact curves (the free-form fillet) would otherwise read every
/// sliver fragment near a contact as lying ON the tool.
pub fn select_class_tol(arr: &Arrangement, bodies: [&Body; 2], op: Op, tol: f64) -> Result<Vec<OutFace>, String> {
    let solids = [Solid::new(bodies[0], tol), Solid::new(bodies[1], tol)];
    select_with(arr, true, None, |side, _fi, q, n| {
        let c = solids[1 - side].classify(q, n);
        (c, keep(op, side, c))
    })
}

/// [`select`] with the decision supplied by the caller: `decide(side, face
/// index in the arrangement, interior sample point, outward normal)` returns
/// the fragment's class (diagnostics only) and `Some(flip)` to keep it.
/// `merge = false` skips the re-fusing of coincident same-facing fragments
/// (a split-only pass, e.g. the pipe cut, must keep the pieces apart).
/// Fragments of `skip_side` are dropped without a sample point.
pub fn select_with(
    arr: &Arrangement,
    merge: bool,
    skip_side: Option<usize>,
    mut decide: impl FnMut(usize, usize, Vec3, Vec3) -> (Class, Option<bool>),
) -> Result<Vec<OutFace>, String> {
    let tol = arr.tol;
    let mut kept: Vec<(Fragment, bool)> = vec![];
    let mut n_frags: Vec<usize> = vec![0; arr.faces.len()];
    for fi in 0..arr.faces.len() {
        let frs = fragments(arr, fi);
        n_frags[fi] = frs.len();
        for mut fr in frs {
            let side = arr.faces[fi].side;
            if skip_side == Some(side) {
                continue;
            }
            let (q, n) = sample_point_whole(arr, &fr).ok_or_else(|| {
                super::bool_dbg!("no sample point for a fragment of face {fi} (side {side}, {}, {} loops)", arr.faces[fi].name.id(), fr.loops.len());
                "booleanFailed".to_string()
            })?;
            let (c, k) = decide(side, fi, q, n);
            fr.class = Some(c);
            super::bool_dbg!("fragment of face {fi} (side {side}, {}, {} loops): sample {q:?} class {c:?} keep {:?}", arr.faces[fi].name.id(), fr.loops.len(), k);
            if let Some(flip) = k {
                kept.push((fr, flip));
            }
        }
    }
    // ---- membranes: two kept fragments of the same surface, facing opposite ways and bounded by the same
    // edges, enclose no volume (an operand that touches itself along a wall: a disc extruded into the hole
    // of its own plate, both walls surviving). Both go, the material on either side fuses.
    if merge {
        let sets: Vec<Vec<usize>> = kept
            .iter()
            .map(|(f, _)| {
                let mut v: Vec<usize> = f.loops.iter().flatten().map(|h| h.0).collect();
                v.sort_unstable();
                v.dedup();
                v
            })
            .collect();
        let mut by_set: std::collections::BTreeMap<&Vec<usize>, Vec<usize>> = Default::default();
        for (i, s) in sets.iter().enumerate() {
            by_set.entry(s).or_default().push(i);
        }
        let mut gone = vec![false; kept.len()];
        for idx in by_set.values().filter(|v| v.len() > 1) {
            for (a, &i) in idx.iter().enumerate() {
                for &j in &idx[a + 1..] {
                    if gone[i] || gone[j] {
                        continue;
                    }
                    let (fi, fj) = (&arr.faces[kept[i].0.af], &arr.faces[kept[j].0.af]);
                    if !coincident(&fi.surface, &fj.surface, tol) {
                        continue;
                    }
                    let p = arr.verts[he_start(arr, kept[i].0.loops[0][0])];
                    let ni = outward(&fi.surface, fi.sense ^ kept[i].1, p);
                    let nj = outward(&fj.surface, fj.sense ^ kept[j].1, p);
                    if ni.dot(nj) < -0.5 {
                        super::bool_dbg!("membrane: fragments of faces {} and {} cancel", kept[i].0.af, kept[j].0.af);
                        gone[i] = true;
                        gone[j] = true;
                    }
                }
            }
        }
        if gone.iter().any(|&g| g) {
            let mut k = 0;
            kept.retain(|_| {
                k += 1;
                !gone[k - 1]
            });
        }
    }
    // directed loops after flipping
    let loops_of =|fr: &Fragment, flip: bool| -> Vec<Vec<He>> {
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
    // fragments sharing an edge (edge -> fragment indices, ascending): only
    // those pairs can merge, so no all-pairs scan on many-fragment bodies
    let mut by_edge: ck_core::fxhash::FxHashMap<usize, Vec<usize>> = Default::default();
    for (i, es) in edges_of.iter().enumerate() {
        for &e in es {
            let v = by_edge.entry(e).or_default();
            if v.last() != Some(&i) {
                v.push(i);
            }
        }
    }
    for i in 0..n {
        if !merge {
            break;
        }
        let mut js: Vec<usize> = edges_of[i].iter().flat_map(|e| by_edge[e].iter().copied()).filter(|&j| j > i).collect();
        js.sort_unstable();
        js.dedup();
        for j in js {
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
    let mut root_count = vec![0usize; n];
    for &r in &roots {
        root_count[r] += 1;
    }
    let merged: Vec<bool> = (0..n).map(|i| root_count[roots[i]] > 1).collect();
    let mut members_of: Vec<Vec<usize>> = vec![vec![]; n];
    for (j, &r) in roots.iter().enumerate() {
        members_of[r].push(j);
    }
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
            absorbed: af.absorbed.clone(),
            loops: loops_of(fr, *flip),
            group,
            key: (group, fr.af, fr.order),
            walk: vec![((group, fr.af, fr.order), fr.loops.clone())],
        });
    }
    // merged groups
    roots.dedup();
    let mut done = vec![false; n];
    for i in 0..n {
        let r = find(&mut parent, i);
        if !merged[i] || done[r] {
            continue;
        }
        done[r] = true;
        let members: Vec<usize> = std::mem::take(&mut members_of[r]);
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
                absorbed: af.absorbed.clone(),
                loops: loops_of(&whole, flip),
                group: 4,
                key: (4, whole.af, 0),
                walk: vec![((group, whole.af, 0), whole.loops.clone())],
            });
            continue;
        }
        let mut tagged: Vec<(usize, He)> = vec![];
        for (mi, &m) in members.iter().enumerate() {
            tagged.extend(loops_of(&kept[m].0, kept[m].1).into_iter().flatten().map(|h| (mi, h)));
        }
        // shared edges cancel (the fragments fuse across them) - except the
        // seam of a periodic face once its two sides are already fused: the
        // seam must stay in a loop to keep the face's uv region simply
        // connected (a ribbon wrapping a cylinder cut by a helical groove)
        let same_face = members.iter().all(|&m| kept[m].0.af == kept[lead].0.af);
        let periodic = af.surface.periods().0.is_some();
        let lead_name = af.name.clone();
        let mut comp: Vec<usize> = (0..members.len()).collect();
        let mut cancelled: Vec<usize> = vec![];
        // Seam edges are decided after every other shared edge: whether the
        // two sides of a seam are "already fused" depends on all the other
        // cancellations, not on the order the half-edges happen to be listed
        // (a cylinder band kept on both sides of its seam above a cut, joined
        // to the part below it only along cut edges listed later, lost the
        // seam: a dangling slit and a second loop, lateral area 0).
        for seam_pass in [false, true] {
            for &(mi, h) in &tagged {
                if h.1 {
                    continue;
                }
                let Some(&(mj, _)) = tagged.iter().find(|x| x.1 == (h.0, true)) else { continue };
                // both coedges inside ONE fragment: the seam twin of a periodic
                // face, which is no shared edge (cancelling it would drop the
                // seam of a merged cylinder: its lateral area integrates to 0)
                if mi == mj && periodic {
                    continue;
                }
                // `is_seam_edge` stays ungated: a cross-face merge of two thread walls
                // (motor+motor F20/F25) needs its parametric seam kept even though the
                // members come from different faces. `bound_twice` stays `same_face`-gated:
                // a near-coincident cut double-edge (re-drill, `redrill A`) bounds one
                // merged face twice without being a seam - keeping it splits the wall's
                // uv domain open and the tessellator leaves a hole.
                let seam = periodic && (is_seam_edge(arr, &af.surface, h.0) || (same_face && bound_twice(af, h.0)));
                if seam != seam_pass {
                    continue;
                }
                let (ri, rj) = (find(&mut comp, mi), find(&mut comp, mj));
                if seam && ri == rj {
                    continue;
                }
                comp[ri.max(rj)] = ri.min(rj);
                cancelled.push(h.0);
            }
        }
        let hes: Vec<He> = tagged.iter().map(|x| x.1).filter(|h| !cancelled.contains(&h.0)).collect();
        let cycles = walk(arr, &af.surface, sense, &hes);
        super::bool_dbg!(
            "merged face {}: members {:?} (faces {:?}), cancelled edges {:?}, remaining {:?}, {} cycles",
            af.name.id(),
            members,
            members.iter().map(|&m| kept[m].0.af).collect::<Vec<_>>(),
            cancelled,
            hes,
            cycles.len()
        );
        for h in &hes {
            let (a, b) = (arr.verts[he_start(arr, *h)], arr.verts[he_end(arr, *h)]);
            super::bool_dbg!("  he {:?}: ({:.3},{:.3},{:.3}) -> ({:.3},{:.3},{:.3}) member {}", h, a.x, a.y, a.z, b.x, b.y, b.z, tagged.iter().find(|x| x.1 == *h).map_or(99, |x| x.0));
        }
        let walk_items: Vec<_> = members
            .iter()
            .map(|&m| {
                let fr = &kept[m].0;
                let g = arr.faces[fr.af].side * 2 + if fr.whole { 0 } else { 1 };
                ((g, fr.af, fr.order), fr.loops.clone())
            })
            .collect();
        // fragments of one periodic face fused across its seam (a ribbon left
        // by a helical groove) stay that face, listed as a modified face of
        // its body (oracle: the bore of a nut cut by a thread)
        let (grp, ord) = if same_face && periodic {
            (af.side * 2 + 1, members.iter().map(|&m| kept[m].0.order).min().unwrap_or(0))
        } else {
            (4, 0)
        };
        let mut absorbed: Vec<String> = arr.faces[kept[lead].0.af].absorbed.clone();
        for &m in &members {
            let maf = &arr.faces[kept[m].0.af];
            let id = maf.name.id();
            if id != lead_name.id() && !absorbed.contains(&id) {
                absorbed.push(id);
            }
            for a in &maf.absorbed {
                if !absorbed.contains(a) {
                    absorbed.push(a.clone());
                }
            }
        }
        for (k, loops) in group(arr, &af.surface, sense, cycles).into_iter().enumerate() {
            out.push(OutFace {
                surface: af.surface.clone(),
                sense,
                name: lead_name.clone(),
                absorbed: absorbed.clone(),
                loops,
                group: grp,
                key: (grp, kept[lead].0.af, ord + k),
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
        if !of.absorbed.is_empty() {
            b.faces[f.idx()].absorbed = of.absorbed.clone();
        }
        let (pu, pv) = of.surface.periods();
        for l in &of.loops {
            let lp = b.add_loop(f);
            let mut prev_end: Option<Vec2> = None;
            // (start, end) uv of every coedge's pcurve, for the pole fix below
            let mut ends: Vec<Option<(Vec2, Vec2)>> = vec![];
            // seam edges walked both ways here must have their pcurves one period apart (the copies
            // `loop_polygon` chose); chaining by continuity would collapse the spike. A loop made
            // only of seam twins is the whole parameter boundary and is handled below.
            let twin_only = !l.is_empty() && l.iter().all(|&(e, r)| l.contains(&(e, !r)));
            let forced = if twin_only { vec![] } else { super::classify::seam_spike_u(&of.surface, &pieces(arr, l), of.sense) };
            for (k, &(ei, rev)) in l.iter().enumerate() {
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
                let pc = edge_pcurve(&of.surface, &ae.curve, ae.t0, ae.t1, tol);
                let pc = pc.map(|pc| {
                    let (ta, tb) = if rev { (ae.t1, ae.t0) } else { (ae.t0, ae.t1) };
                    let mut pc = pc;
                    if let Some(target) = forced.get(k).copied().flatten() {
                        let du = pu.map_or(0.0, |p| ((target - pc.eval(ta).x) / p).round() * p);
                        if du != 0.0 {
                            pc = shift2(&pc, Vec2::new(du, 0.0));
                        }
                    } else if let Some(prev) = prev_end {
                        let s = pc.eval(ta);
                        let du = pu.map_or(0.0, |p| ((prev.x - s.x) / p).round() * p);
                        let dv = pv.map_or(0.0, |p| ((prev.y - s.y) / p).round() * p);
                        if du != 0.0 || dv != 0.0 {
                            pc = shift2(&pc, Vec2::new(du, dv));
                        }
                    }
                    prev_end = Some(pc.eval(tb));
                    ends.push(Some((pc.eval(ta), pc.eval(tb))));
                    pc
                });
                if pc.is_none() {
                    ends.push(None);
                }
                b.add_coedge(lp, eid, rev, pc);
            }
            // the forced copies already close the loop across the pole line
            if !forced.iter().any(|x| x.is_some()) {
                fix_pole_jump(&mut b, lp, &of.surface, &ends);
            }
            // a loop made only of seam twins (revolve sphere meridian): the
            // twins' pcurves must lie one period apart, the loop CCW about
            // the outward normal (positive uv area when sense)
            if !l.is_empty() && l.iter().all(|&(e, r)| l.contains(&(e, !r))) {
                // (a torus has two seams: the continuity chaining above already
                // places the four twins one period apart, so it is left alone)
                if let (Some(p), None) | (None, Some(p)) = (pu, pv) {
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
    split_pinch_vertices(&mut b);
    ck_topo::region::rebuild_lumps(&mut b);
    b
}

/// The pcurve of edge curve `c` over `[t0, t1]` on surface `s` (before any period shift), memoised by the exact
/// inputs: a boolean chain rebuilds every coedge of every result, and the pcurve fits of the edges it did not
/// touch were the bulk of `build` (a pure function of these inputs, so a repeat returns the same curve).
fn edge_pcurve(s: &Surface, c: &Curve, t0: f64, t1: f64, tol: f64) -> Option<Curve2> {
    thread_local! {
        static MEMO: std::cell::RefCell<ck_core::memo::Memo<Option<Curve2>>> = std::cell::RefCell::new(ck_core::memo::Memo::new(1 << 21));
    }
    let mut key = Vec::with_capacity(48);
    s.push_key(&mut key);
    c.push_key(&mut key);
    key.extend([t0.to_bits(), t1.to_bits(), tol.to_bits()]);
    ck_core::memo::cached(&MEMO, key, || {
        exact_pcurve(s, c, tol * 10.0).or_else(|| {
            let conic = matches!(c, Curve::Circle { .. } | Curve::Ellipse { .. });
            if matches!(s, Surface::BSpline(_) | Surface::Screw { .. } | Surface::Plane { .. }) && matches!(c, Curve::BSpline(_)) {
                // (a long cut curve turning round inside a sweep face is ~1 mm off its
                // 32-sample cubic fit at the tip: refine until it reproduces the curve)
                pcurve_of_dense(s, c, t0, t1, tol, 1e-5).ok()
            } else if conic && !matches!(s, Surface::Plane { .. }) {
                // a great circle on a sphere that does not pass through its poles (a
                // rotated sphere cut by an axis plane) has no exact uv line: the
                // 32-sample fit deviates ~1e-4, more than the validator's tolerance
                pcurve_of_dense(s, c, t0, t1, tol, 1e-6).ok()
            } else {
                pcurve_of(s, c, t0, t1, tol).ok()
            }
        })
    })
}

/// A vertex where the face loops pair only part of its incident edges is a pinch:
/// two boundary branches touch at one point (an exact tangency, e.g. a tool wall
/// grazing the host wall and opening two holes that meet at the tangent point).
/// Split it into one coincident vertex per pairing component, so each loop is
/// manifold again; the geometry is unchanged, only the incidence.
fn split_pinch_vertices(b: &mut Body) {
    use std::collections::HashMap;
    let nv = b.vertices.len();
    let mut inc: HashMap<usize, Vec<usize>> = HashMap::default();
    for e in b.edge_ids() {
        let ed = b.edge(e);
        inc.entry(ed.v0.idx()).or_default().push(e.idx());
        if ed.v1.idx() != ed.v0.idx() {
            inc.entry(ed.v1.idx()).or_default().push(e.idx());
        }
    }
    fn find(up: &mut HashMap<usize, usize>, x: usize) -> usize {
        let p = up[&x];
        if p == x {
            x
        } else {
            let r = find(up, p);
            up.insert(x, r);
            r
        }
    }
    // edges that consecutive loop coedges join at a vertex must keep one vertex:
    // collect those pairs per vertex in one pass over the loops
    let mut joins: HashMap<usize, Vec<(usize, usize)>> = HashMap::default();
    for f in b.face_ids() {
        for &l in &b.face(f).loops {
            let cs = &b.lp(l).coedges;
            let n = cs.len();
            for i in 0..n {
                let (c, d) = (cs[i], cs[(i + 1) % n]);
                let v = b.coedge_end(c).idx();
                joins.entry(v).or_default().push((b.coedge(c).edge.idx(), b.coedge(d).edge.idx()));
            }
        }
    }
    for v in 0..nv {
        let Some(es) = inc.get(&v) else { continue };
        if es.len() < 2 {
            continue;
        }
        let mut up: HashMap<usize, usize> = es.iter().map(|&e| (e, e)).collect();
        if let Some(js) = joins.get(&v) {
            for &(e1, e2) in js {
                if !up.contains_key(&e1) || !up.contains_key(&e2) {
                    continue;
                }
                let (r1, r2) = (find(&mut up, e1), find(&mut up, e2));
                if r1 != r2 {
                    up.insert(r1.max(r2), r1.min(r2));
                }
            }
        }
        let mut groups: HashMap<usize, Vec<usize>> = HashMap::default();
        for &e in es {
            let r = find(&mut up, e);
            groups.entry(r).or_default().push(e);
        }
        if groups.len() < 2 {
            continue;
        }
        let p = b.vertices[v].p;
        let mut gs: Vec<Vec<usize>> = groups.into_values().collect();
        gs.sort();
        let mut rest = gs.into_iter();
        // the edge curve ends of a component do not meet exactly at the pinched point
        // (the SSI is numerically fuzzy there): give the vertex the gap as its tolerance
        let gap = |b: &Body, g: &[usize]| -> f64 {
            g.iter()
                .map(|&e| {
                    let ed = &b.edges[e];
                    let t = if ed.v0.idx() == v { ed.t0 } else { ed.t1 };
                    p.dist(ed.curve.eval(t))
                })
                .fold(0.0, f64::max)
        };
        if let Some(g) = rest.next() {
            b.vertices[v].tol = b.vertices[v].tol.max(gap(b, &g)); // the first component keeps `v`
        }
        for g in rest {
            let nvid: VertexId = b.add_vertex(p);
            b.vertices[nvid.idx()].tol = gap(b, &g);
            for e in g {
                if b.edges[e].v0.idx() == v {
                    b.edges[e].v0 = nvid;
                }
                if b.edges[e].v1.idx() == v {
                    b.edges[e].v1 = nvid;
                }
            }
        }
    }
}

/// A loop through a pole of a periodic surface (a sphere cap bounded by one
/// circle: circle, seam up, seam down) jumps in u at the pole: u is free
/// there, and the loop must close in uv. The pcurves were chained by
/// continuity (u unchanged across the pole), so the coedges after the pole
/// are shifted by the whole periods that make the last end meet the first
/// start.
fn fix_pole_jump(b: &mut Body, lp: ck_topo::body::LoopId, s: &Surface, ends: &[Option<(Vec2, Vec2)>]) {
    let Some(per) = s.periods().0 else { return };
    let n = ends.len();
    if n < 3 || ends.iter().any(|e| e.is_none()) {
        return;
    }
    let is_pole = |q: Vec2| s.is_pole(q.x, q.y);
    let ends: Vec<(Vec2, Vec2)> = ends.iter().map(|e| e.unwrap()).collect();
    let Some(k) = (0..n).find(|&k| is_pole(ends[k].1) && is_pole(ends[(k + 1) % n].0) && k + 1 < n) else { return };
    let gap = ends[0].0.x - ends[n - 1].1.x;
    let d = (gap / per).round() * per;
    if d == 0.0 || (gap - d).abs() > 1e-6 * per {
        return;
    }
    let cs = b.lp(lp).coedges.clone();
    for &c in &cs[k + 1..] {
        if let Some(pc) = b.coedges[c.idx()].pcurve.clone() {
            b.coedges[c.idx()].pcurve = Some(shift2(&pc, Vec2::new(d, 0.0)));
        }
    }
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
        out.faces[nf.idx()].absorbed = face.absorbed.clone();
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
